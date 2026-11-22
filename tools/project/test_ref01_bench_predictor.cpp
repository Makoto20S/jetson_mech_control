// Portable behavioral checks; no ROS, serial ports, files, or hardware.
// Checks stay active when built with -DNDEBUG.
#include "ref01_bench_predictor.hpp"
#include "mech_bringup/ak30_kinematic_lease.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace core = mech::mech_controllers::ref01;
using core::BenchPredictor;
using core::ModelState;
using core::Pair;
using Matrix = std::array<double, 4>;

int checks = 0;
void check(bool condition, const char* message) {
  ++checks;
  if (!condition) throw std::runtime_error(message);
}
bool near(double a, double b, double tolerance = 1e-11) {
  return std::abs(a - b) <= tolerance;
}
Matrix mass() { return {.006, 0., 0., .004}; }
ModelState measured() {
  return {{0., .3}, {0., 0.}, {0., .1}, {1., 1., 1., -1.}, 0., 0};
}
core::Plan plan() {
  return {true, true, 1., 1., 1., {0., .3}, {10., 10.}, {1., 1.}, {5., 5.}};
}

void observation_and_prediction() {
  BenchPredictor predictor;
  check(predictor.observe(measured(), mass(), 1.), "initial observation");
  core::Torque torque;
  check(predictor.command(plan(), torque), "equilibrium command");
  check(near(torque.motor_nm[0], -.05) && near(torque.motor_nm[1], .05),
        "zero-error equilibrium supplies gravity in motor104/105 order");
  const auto original_source = predictor.source();
  const auto original_q = predictor.state().q;
  // These are representable 12-bit, +/-15 Nm wire torques; prediction must
  // follow the torque actually sent, not the original unquantized request.
  const Pair wire{-15. + 2041. * 30. / 4095., -15. + 2054. * 30. / 4095.};
  check(predictor.advance(wire, .005), "advance encoded torque");
  check(predictor.source() == original_source, "prediction does not refresh source");
  check(predictor.state().q != original_q, "wire quantization affects predicted state");
  for (double value : predictor.state().q) check(std::isfinite(value), "finite predicted pose");
  for (double value : predictor.state().dq) check(std::isfinite(value), "finite predicted speed");
  check(predictor.command(plan(), torque), "command after prediction");
  check(predictor.source() == original_source, "command does not refresh source");
}

void observation_rejection_latches() {
  for (double source : {1., .99}) {
    BenchPredictor predictor;
    check(predictor.observe(measured(), mass(), 1.), "seed before replay");
    const auto before = predictor.state().q;
    check(!predictor.observe(measured(), mass(), source), "duplicate or rollback rejected");
    check(predictor.failed(), "replayed observation latches failure");
    check(predictor.state().q == before && predictor.source() == 1.,
          "rejected observation does not mutate source or state");
    check(!predictor.observe(measured(), mass(), 1.02), "fresh source cannot auto-recover");
    core::Torque torque;
    torque.motor_nm = {1., -1.};
    check(!predictor.command(plan(), torque), "latched failure blocks command");
    check(torque.motor_nm == Pair{}, "blocked command clears torque result");
  }
}

void invalid_inputs_and_reset() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const Matrix bad : {Matrix{0., 0., 0., .004}, Matrix{-.006, 0., 0., .004},
                           Matrix{.006, .006, .006, .006}, Matrix{.006, .001, .002, .004},
                           Matrix{nan, 0., 0., .004}}) {
    BenchPredictor predictor;
    check(!predictor.observe(measured(), bad, 1.), "invalid inertia rejected");
    check(predictor.failed(), "invalid inertia latches failure");
  }
  for (int field = 0; field < 3; ++field) {
    auto observation = measured();
    if (field == 0) observation.q[0] = nan;
    if (field == 1) observation.dq[1] = nan;
    if (field == 2) observation.A[0] = nan;
    BenchPredictor predictor;
    check(!predictor.observe(observation, mass(), 1.), "nonfinite observation rejected");
  }
  for (double cap : {0., -1., nan}) {
    BenchPredictor predictor;
    check(predictor.observe(measured(), mass(), 1.), "seed before bad cap");
    auto command_plan = plan();
    command_plan.caps[0] = cap;
    core::Torque torque;
    check(!predictor.command(command_plan, torque), "bad cap rejected");
    check(predictor.failed() && torque.motor_nm == Pair{}, "bad cap blocks output");
  }
  for (double dt : {0., -.005, .010, .020, nan}) {
    BenchPredictor predictor;
    check(predictor.observe(measured(), mass(), 1.), "seed before bad dt");
    check(!predictor.advance({0., 0.}, dt), "invalid prediction period rejected");
  }
  BenchPredictor predictor;
  check(predictor.observe(measured(), mass(), 1.), "seed before disturbance estimate");
  auto changed = measured();
  changed.dq = {.2, -.1};
  check(predictor.observe(changed, mass(), 1.02), "velocity innovation accepted");
  check(std::abs(predictor.external()[0]) + std::abs(predictor.external()[1]) > 0.,
        "innovation creates nonzero disturbance estimate");
  check(!predictor.advance({nan, 0.}, .005), "NaN applied torque rejected");
  predictor.reset();
  check(!predictor.failed() && !predictor.initialized() && predictor.source() == 0.,
        "reset clears lifecycle and source");
  check(predictor.external() == Pair{}, "reset clears disturbance estimate");
  check(predictor.observe(measured(), mass(), 2.), "explicit reset permits fresh observation");
}

void reversal_and_common_saturation() {
  BenchPredictor predictor;
  auto observation = measured();
  observation.gravity = {0., 0.};
  observation.q[0] = -.1;
  check(predictor.observe(observation, mass(), 1.), "seed positive demand");
  core::Torque positive, negative;
  check(predictor.command(plan(), positive), "positive command");
  check(positive.motor_nm[0] > 0. && positive.motor_nm[1] > 0., "positive restoring direction");
  observation.q[0] = .1;
  check(predictor.observe(observation, mass(), 1.02), "opposite-error observation");
  check(predictor.command(plan(), negative), "reverse command");
  check(negative.motor_nm[0] < 0. && negative.motor_nm[1] < 0.,
        "one command removes prior sign without slew tail");

  predictor.reset();
  observation.q = {.1, .45};
  check(predictor.observe(observation, mass(), 2.), "seed coupled demand");
  core::Torque full, limited;
  auto p = plan();
  check(predictor.command(p, full), "unsaturated coupled command");
  p.caps = {.01, .02};  // alpha1, alpha2: asymmetric caps, one common scale.
  check(predictor.command(p, limited), "saturated coupled command");
  check(limited.scale > 0. && limited.scale < 1., "common saturation active");
  check(std::abs(limited.alpha_nm[0]) <= .01 + 1e-12 &&
        std::abs(limited.alpha_nm[1]) <= .02 + 1e-12, "both actuator caps observed");
  const Pair full_q{full.alpha_nm[0] + full.alpha_nm[1], full.alpha_nm[0] - full.alpha_nm[1]};
  const Pair limited_q{limited.alpha_nm[0] + limited.alpha_nm[1], limited.alpha_nm[0] - limited.alpha_nm[1]};
  check(near(full_q[0] * limited_q[1], full_q[1] * limited_q[0]),
        "saturation preserves generalized force direction");
  check(near(limited.motor_nm[0], full.motor_nm[0] * limited.scale) &&
        near(limited.motor_nm[1], full.motor_nm[1] * limited.scale),
        "same scale applies to both motor targets");
}

void constant_external_load_retains_static_stiffness() {
  // Independent exact constant-acceleration, two-DOF linear plant. Its 20 ms
  // observations correct the 5 ms predictor; no MuJoCo/codec acceptance claim.
  // The 0.3 Nm load should settle at 0.03 rad for K=10, not the much softer
  // equilibrium produced by simply regularizing the commanded torque.
  BenchPredictor predictor;
  auto state = measured();
  const Pair external{.3, 0.};
  core::Torque torque;
  for (int tick = 0; tick < 1600; ++tick) {
    if (tick % 4 == 0)
      check(predictor.observe(state, mass(), 1. + tick * .005), "linear plant observation");
    check(predictor.command(plan(), torque), "linear plant command");
    check(predictor.advance(torque.motor_nm, .005), "linear plant prediction");
    const Pair acceleration{
        (torque.alpha_nm[0] + torque.alpha_nm[1] - state.gravity[0] + external[0]) / .006,
        (torque.alpha_nm[0] - torque.alpha_nm[1] - state.gravity[1] + external[1]) / .004};
    for (unsigned i = 0; i < 2; ++i) {
      state.q[i] += .005 * state.dq[i] + .5 * .005 * .005 * acceleration[i];
      state.dq[i] += .005 * acceleration[i];
    }
  }
  check(near(state.q[0], .03, .003), "constant load yields nominal K10 deflection");
  check(near(state.q[1], .3, .003), "constant load does not bias other coordinate");
  check(std::max(std::abs(state.dq[0]), std::abs(state.dq[1])) < .01,
        "constant-load equilibrium settles");
}

int main() {
  try {
    mech::mech_bringup::KinematicBenchEnvelope direct;
    direct.reset(80., mech::mech_bringup::KinematicBenchEnvelope::EffortPolicy::Direct);
    check(direct.update(true,false,.4,80.,0.,0.,40.,1000000000LL,.005), "direct first command");
    check(near(direct.effort(),.4), "direct target has no startup slew");
    check(direct.update(true,false,-2.,80.,0.,0.,40.,1005000000LL,.005), "direct reversal");
    check(near(direct.effort(),-2.), "0.4 to -2 Nm reversed within one 5 ms tick");
    check(direct.update(true,true,-2.,80.,0.,0.,40.,1010000000LL,.005), "single cancellation zero");
    check(direct.effort()==0. && direct.stopped(), "cancellation latched");
    check(!direct.update(true,false,.4,80.,0.,0.,40.,1015000000LL,.005), "no writes after cancel");
    observation_and_prediction();
    observation_rejection_latches();
    invalid_inputs_and_reset();
    reversal_and_common_saturation();
    constant_external_load_retains_static_stiffness();
    std::cout << "predictor contract: " << checks << " checks passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "predictor contract failed after " << checks << " checks: " << error.what() << '\n';
    return 1;
  }
}
