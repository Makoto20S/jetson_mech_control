#pragma once

// Native counterpart of ref01_mujoco_kinematics.py and the zero-velocity
// projection in ref01_dynamics_api.py. One instance per control thread.
// Model loading allocates; evaluate uses only fixed-size local storage and
// MuJoCo's preallocated mjData workspace. No file I/O or Python in evaluate.
#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

namespace ph_exo {

struct ModelState {
  std::array<double, 2> q{};        // [psi, beta], rad
  std::array<double, 2> dq{};       // [psi_dot, beta_dot], rad/s
  std::array<double, 2> gravity{};  // generalized gravity torque, N m
  std::array<double, 4> A{};        // row-major; alpha_dot = A * q_dot
  double residual{};               // max absolute connect residual, m
  int iterations{};
  std::array<double, 4> mass{};   // N.T M N in [psi,beta], kg m^2
};

namespace detail {

// Partial-pivot LU for the two six-coordinate dependent systems. The matrix
// infinity condition number is obtained by six solves, with bounded work.
// It differs from NumPy's 2-norm condition but uses the same 1e8 rejection cap.
class Lu6 {
 public:
  bool factor(const std::array<double, 36>& matrix) noexcept {
    lu_ = matrix;
    norm_ = 0.;
    for (int i = 0; i < 6; ++i) {
      double row = 0.;
      for (int j = 0; j < 6; ++j) {
        if (!std::isfinite(lu_[i * 6 + j])) return false;
        row += std::abs(lu_[i * 6 + j]);
      }
      norm_ = std::max(norm_, row);
    }
    if (!(norm_ > 0.) || !std::isfinite(norm_)) return false;
    for (int k = 0; k < 6; ++k) {
      int p = k;
      for (int i = k + 1; i < 6; ++i)
        if (std::abs(lu_[i * 6 + k]) > std::abs(lu_[p * 6 + k])) p = i;
      if (std::abs(lu_[p * 6 + k]) <= norm_ * 1e-14) return false;
      pivots_[k] = p;
      if (p != k)
        for (int j = 0; j < 6; ++j) std::swap(lu_[k * 6 + j], lu_[p * 6 + j]);
      for (int i = k + 1; i < 6; ++i) {
        lu_[i * 6 + k] /= lu_[k * 6 + k];
        for (int j = k + 1; j < 6; ++j)
          lu_[i * 6 + j] -= lu_[i * 6 + k] * lu_[k * 6 + j];
      }
    }
    return true;
  }

  bool solve(std::array<double, 6>& rhs) const noexcept {
    // Apply the row permutation before triangular substitution: later pivots
    // also permuted already-computed L columns during factorization.
    for (int k = 0; k < 6; ++k) if (pivots_[k] != k) std::swap(rhs[k], rhs[pivots_[k]]);
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < i; ++j) rhs[i] -= lu_[i * 6 + j] * rhs[j];
    for (int i = 5; i >= 0; --i) {
      for (int j = i + 1; j < 6; ++j) rhs[i] -= lu_[i * 6 + j] * rhs[j];
      rhs[i] /= lu_[i * 6 + i];
      if (!std::isfinite(rhs[i])) return false;
    }
    return true;
  }

  bool conditioned() const noexcept {
    std::array<double, 6> row_sums{};
    for (int column = 0; column < 6; ++column) {
      std::array<double, 6> rhs{};
      rhs[column] = 1.;
      if (!solve(rhs)) return false;
      for (int row = 0; row < 6; ++row) row_sums[row] += std::abs(rhs[row]);
    }
    const double condition = norm_ * *std::max_element(row_sums.begin(), row_sums.end());
    return std::isfinite(condition) && condition <= 1e8;
  }

 private:
  std::array<double, 36> lu_{};
  std::array<int, 6> pivots_{};
  double norm_{};
};

struct ModelDeleter { void operator()(mjModel* m) const noexcept { mj_deleteModel(m); } };
struct DataDeleter { void operator()(mjData* d) const noexcept { mj_deleteData(d); } };

}  // namespace detail

class Ref01Model {
 public:
  explicit Ref01Model(const std::string& xml_path) {
    std::array<char, 1024> error{};
    model_.reset(mj_loadXML(xml_path.c_str(), nullptr, error.data(), static_cast<int>(error.size())));
    if (!model_) throw std::runtime_error(std::string("REF01 load: ") + error.data());
    if (model_->nq != 8 || model_->nv != 8 || model_->neq != 2 || model_->njnt != 8)
      throw std::runtime_error("REF01 requires eight hinge coordinates and two connect equalities");
    for (int i = 0; i < 8; ++i) {
      if (model_->jnt_type[i] != mjJNT_HINGE || model_->jnt_qposadr[i] != i ||
          model_->jnt_dofadr[i] != i || model_->dof_frictionloss[i] != 0.)
        throw std::runtime_error("Unsupported REF01 coordinate or dry friction layout");
    }
    for (int i = 0; i < 2; ++i)
      if (model_->eq_type[i] != mjEQ_CONNECT)
        throw std::runtime_error("REF01 equality must be connect");
    model_->opt.jacobian = mjJAC_DENSE;
    active_ = {coordinate("alpha1"), coordinate("alpha2")};
    output_ = {coordinate("psi"), coordinate("beta")};
    int a = 0, q = 0;
    for (int i = 0; i < 8; ++i) {
      if (i != active_[0] && i != active_[1]) dependent_alpha_[a++] = i;
      if (i != output_[0] && i != output_[1]) dependent_q_[q++] = i;
    }
    data_.reset(mj_makeData(model_.get()));
    if (!data_) throw std::runtime_error("Cannot allocate REF01 data");
    reset();
  }

  Ref01Model(const Ref01Model&) = delete;
  Ref01Model& operator=(const Ref01Model&) = delete;
  Ref01Model(Ref01Model&&) = delete;
  Ref01Model& operator=(Ref01Model&&) = delete;
  ~Ref01Model() = default;

  mjModel* model() noexcept { return model_.get(); }
  mjData* data() noexcept { return data_.get(); }
  const mjModel* model() const noexcept { return model_.get(); }
  const mjData* data() const noexcept { return data_.get(); }

  void reset() noexcept {
    mj_resetData(model_.get(), data_.get());
    std::copy_n(model_->qpos0, 8, seed_.begin());
  }

  bool evaluate(std::array<double, 2> alpha, std::array<double, 2> alpha_velocity,
                ModelState& out) noexcept {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    out = {{nan, nan}, {nan, nan}, {nan, nan}, {nan, nan, nan, nan}, nan, 0};
    for (int i = 0; i < 2; ++i)
      if (!std::isfinite(alpha[i]) || std::abs(alpha[i]) > pi_ ||
          !std::isfinite(alpha_velocity[i])) return false;
    std::copy(seed_.begin(), seed_.end(), data_->qpos);
    std::fill_n(data_->qvel, 8, 0.);
    const std::array<double, 2> start{seed_[active_[0]], seed_[active_[1]]};
    const double distance = std::max(std::abs(alpha[0] - start[0]), std::abs(alpha[1] - start[1]));
    if (!std::isfinite(distance) || distance > 2. * pi_) return fail();
    const int count = std::max(1, static_cast<int>(std::ceil(distance / .15)));
    int iterations = 0;
    double residual = 0.;
    for (int step = 1; step <= count; ++step) {
      const double fraction = static_cast<double>(step) / count;
      const std::array<double, 2> target{
          start[0] + (alpha[0] - start[0]) * fraction,
          start[1] + (alpha[1] - start[1]) * fraction};
      if (!project(target, residual, iterations)) return fail();
    }
    const std::array<double, 2> q{data_->qpos[output_[0]], data_->qpos[output_[1]]};
    // The supported IK branch reaches beta=-11 deg at this input phi.
    // The other side of the fold can yield the same output beta.
    constexpr double min_phi = 8.726909068309409 * pi_ / 180.;
    const double phi = pi_ / 2. - (alpha[0] + alpha[1]) / 2.;
    if (!(phi >= min_phi - 1e-8 && std::abs(q[0]) <= pi_ / 3. + 1e-8 &&
          q[1] >= -11. * pi_ / 180. - 1e-8 && q[1] <= pi_ / 4. + 1e-8)) return fail();

    // project leaves efc_J at the accepted position, with the active angles
    // held fixed. Construct both null-space charts from that one Jacobian.
    std::array<double, 48> jac{};
    std::copy_n(data_->efc_J, 48, jac.begin());
    std::array<double, 36> matrix{};
    submatrix(jac, dependent_alpha_, matrix);
    detail::Lu6 active_lu;
    if (!active_lu.factor(matrix) || !active_lu.conditioned()) return fail();
    std::array<double, 6> rhs{};
    for (int row = 0; row < 6; ++row)
      rhs[row] = -jac[row * 8 + active_[0]] * alpha_velocity[0]
                 -jac[row * 8 + active_[1]] * alpha_velocity[1];
    if (!active_lu.solve(rhs)) return fail();
    std::array<double, 8> full_velocity{};
    full_velocity[active_[0]] = alpha_velocity[0];
    full_velocity[active_[1]] = alpha_velocity[1];
    for (int row = 0; row < 6; ++row) full_velocity[dependent_alpha_[row]] = rhs[row];

    submatrix(jac, dependent_q_, matrix);
    detail::Lu6 output_lu;
    if (!output_lu.factor(matrix) || !output_lu.conditioned()) return fail();
    std::array<double, 16> nullspace{};
    nullspace[output_[0] * 2] = 1.;
    nullspace[output_[1] * 2 + 1] = 1.;
    for (int column = 0; column < 2; ++column) {
      for (int row = 0; row < 6; ++row) rhs[row] = -jac[row * 8 + output_[column]];
      if (!output_lu.solve(rhs)) return fail();
      for (int row = 0; row < 6; ++row) nullspace[dependent_q_[row] * 2 + column] = rhs[row];
    }

    // qvel=0 makes Jdot*v and viscous damping zero. N.T*(bias-passive)
    // therefore equals dynamics_terms(q, (0,0))["bias"] without full M/Jdot.
    mj_forward(model_.get(), data_.get());
    std::array<double, 2> gravity{};
    for (int row = 0; row < 8; ++row) {
      const double bias = data_->qfrc_bias[row] - data_->qfrc_passive[row];
      for (int column = 0; column < 2; ++column)
        gravity[column] += nullspace[row * 2 + column] * bias;
    }
    const std::array<double, 4> A{nullspace[active_[0] * 2], nullspace[active_[0] * 2 + 1],
                                nullspace[active_[1] * 2], nullspace[active_[1] * 2 + 1]};
    for (double value : A) if (!std::isfinite(value)) return fail();
    for (double value : gravity) if (!std::isfinite(value)) return fail();
    const double determinant = A[0] * A[3] - A[1] * A[2];
    const double norm = std::max(std::abs(A[0]) + std::abs(A[1]), std::abs(A[2]) + std::abs(A[3]));
    const double adj_norm = std::max(std::abs(A[3]) + std::abs(A[1]), std::abs(A[2]) + std::abs(A[0]));
    if (!std::isfinite(determinant) || determinant == 0. ||
        !std::isfinite(norm * adj_norm / std::abs(determinant)) ||
        norm * adj_norm / std::abs(determinant) > 1e8) return fail();
    std::array<double, 64> full_mass{};
    std::array<double, 4> mass{};
    mj_fullM(model_.get(), data_.get(), full_mass.data());
    for (int i = 0; i < 2; ++i)
      for (int j = 0; j < 2; ++j)
        for (int r = 0; r < 8; ++r)
          for (int c = 0; c < 8; ++c)
            mass[2*i+j] += nullspace[2*r+i] * full_mass[8*r+c] * nullspace[2*c+j];
    for (double value : mass) if (!std::isfinite(value)) return fail();
    if (mass[0] <= 0. || mass[3] <= 0. || mass[0]*mass[3]-mass[1]*mass[2] <= 0.) return fail();
    out = {q, {full_velocity[output_[0]], full_velocity[output_[1]]}, gravity, A, residual, iterations, mass};
    std::copy_n(data_->qpos, 8, seed_.begin());
    return true;
  }

 private:
  static constexpr double pi_ = 3.14159265358979323846;
  std::unique_ptr<mjModel, detail::ModelDeleter> model_;
  std::unique_ptr<mjData, detail::DataDeleter> data_;
  std::array<int, 2> active_{};
  std::array<int, 2> output_{};
  std::array<int, 6> dependent_alpha_{};
  std::array<int, 6> dependent_q_{};
  std::array<double, 8> seed_{};

  int coordinate(const char* name) const {
    const int id = mj_name2id(model_.get(), mjOBJ_JOINT, name);
    if (id < 0) throw std::runtime_error(std::string("Missing REF01 joint: ") + name);
    return model_->jnt_qposadr[id];
  }

  bool fail() noexcept {
    std::copy(seed_.begin(), seed_.end(), data_->qpos);
    return false;
  }

  static void submatrix(const std::array<double, 48>& jac, const std::array<int, 6>& columns,
                        std::array<double, 36>& result) noexcept {
    for (int row = 0; row < 6; ++row)
      for (int column = 0; column < 6; ++column)
        result[row * 6 + column] = jac[row * 8 + columns[column]];
  }

  bool constraints(double& norm) noexcept {
    mj_fwdPosition(model_.get(), data_.get());
    if (data_->nefc != 6) return false;
    norm = 0.;
    for (int i = 0; i < 6; ++i) {
      if (!std::isfinite(data_->efc_pos[i])) return false;
      norm = std::max(norm, std::abs(data_->efc_pos[i]));
    }
    return true;
  }

  bool project(const std::array<double, 2>& alpha, double& residual, int& iterations) noexcept {
    data_->qpos[active_[0]] = alpha[0];
    data_->qpos[active_[1]] = alpha[1];
    for (int iteration = 0; iteration <= 20; ++iteration) {
      if (!constraints(residual)) return false;
      if (residual <= 1e-11) return true;
      if (iteration == 20) return false;
      std::array<double, 36> matrix{};
      std::array<double, 6> delta{};
      for (int row = 0; row < 6; ++row) {
        delta[row] = -data_->efc_pos[row];
        for (int column = 0; column < 6; ++column)
          matrix[row * 6 + column] = data_->efc_J[row * 8 + dependent_alpha_[column]];
      }
      detail::Lu6 lu;
      if (!lu.factor(matrix) || !lu.solve(delta)) return false;
      double largest = 1e-15;
      for (double value : delta) largest = std::max(largest, std::abs(value));
      const double scale = std::min(1., .25 / largest);
      std::array<double, 8> previous{};
      std::copy_n(data_->qpos, 8, previous.begin());
      bool accepted = false;
      double backtrack = 1.;
      for (int power = 0; power < 8; ++power, backtrack *= .5) {
        for (int i = 0; i < 6; ++i)
          data_->qpos[dependent_alpha_[i]] = previous[dependent_alpha_[i]] + delta[i] * scale * backtrack;
        double trial = 0.;
        if (!constraints(trial)) return false;
        if (trial < residual) { accepted = true; break; }
      }
      if (!accepted) return false;
      ++iterations;
    }
    return false;
  }
};

}  // namespace ph_exo
