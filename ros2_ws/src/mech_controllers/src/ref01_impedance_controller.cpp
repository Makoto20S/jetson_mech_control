#include "mech_controllers/ref01_impedance_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iomanip>
#include <locale>
#include <sstream>

#include "ref01_model.hpp"

namespace mech::mech_controllers {
namespace {
using Callback = controller_interface::CallbackReturn;
using Configuration = controller_interface::InterfaceConfiguration;
using ArrayMessage = std_msgs::msg::Float64MultiArray;
double monotonic_seconds() noexcept {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
constexpr std::array<const char*, 4> state_fields{
    "position", "velocity", "sample_time_s", "sequence"};
void append_json_number(std::ostream& stream, double value) {
  if (std::isfinite(value)) stream << value;
  else stream << "null";
}
}  // namespace

Ref01ImpedanceController::Ref01ImpedanceController() = default;
Ref01ImpedanceController::~Ref01ImpedanceController() = default;

Callback Ref01ImpedanceController::on_init() {
  try {
    auto_declare<bool>("simulation_only", false);
    auto_declare<std::string>("command_source", "legacy_array");
    auto_declare<std::string>("bench_profile", "");
    auto_declare<bool>("predictive_impedance", false);
    auto_declare<double>("receive_time_uncertainty_s", .007);
    auto_declare<std::string>("model_path", "");
    auto_declare<std::vector<std::string>>("joints", joints_);
  } catch (const std::exception&) { return Callback::ERROR; }
  return Callback::SUCCESS;
}

Callback Ref01ImpedanceController::on_configure(const rclcpp_lifecycle::State&) {
  epoch_.store(0.);
  session_.deactivate();
  predictive_session_.deactivate();
  subscription_.reset();
  typed_subscription_.reset();ref_subscription_.reset();typed_session_=0;dynamic_=false;
  status_timer_.reset();status_publisher_.reset();
  status_service_.reset();
  stop_service_.reset();
  stop_epoch_.store(0.);
  model_.reset();
  refresh_software_status(monotonic_seconds());
  try {
    auto node = get_node();
    bench_=node->get_parameter("bench_profile").as_string()=="ref01_supported_v1";
    predictive_=node->get_parameter("predictive_impedance").as_bool();
    predictive_session_={};
    if(predictive_ && (!bench_ || node->get_parameter("receive_time_uncertainty_s").as_double()!=.007 ||
       !predictive_session_.set_receive_time_bound(.007)))return Callback::ERROR;
    if (node->get_parameter("simulation_only").as_bool()==bench_ ||
        node->get_parameter("use_sim_time").as_bool()) return Callback::ERROR;
    stop_service_=node->create_service<std_srvs::srv::Trigger>("~/stop",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          stop_epoch_.store(epoch_.load());
          response->success=true;response->message="stop requested";
        });
    joints_ = node->get_parameter("joints").as_string_array();
    if (joints_.size() != 2 || joints_[0].empty() || joints_[1].empty() || joints_[0] == joints_[1])
      return Callback::ERROR;
    const auto path = node->get_parameter("model_path").as_string();
    if (path.empty()) return Callback::ERROR;
    model_ = std::make_unique<ph_exo::Ref01Model>(path);
    const auto source=node->get_parameter("command_source").as_string();
    if(source!="legacy_array" && source!="fixed_pose_v1" && source!="dynamic_ref_v1")return Callback::ERROR;
    dynamic_=source=="dynamic_ref_v1";
    if(source!="legacy_array" && predictive_)return Callback::ERROR;
    if(source=="legacy_array") subscription_ = node->create_subscription<ArrayMessage>("~/plan", rclcpp::QoS(1),
        [this](const ArrayMessage::SharedPtr message) {
          if (!message || message->data.size() != 12 || !message->layout.dim.empty() ||
              message->layout.data_offset != 0) return;
          std::array<double, 12> frame{};
          std::copy(message->data.begin(), message->data.end(), frame.begin());
          (void)set_plan(frame);
        });
    if(source=="fixed_pose_v1") {
      typed_subscription_=node->create_subscription<ph_exo_interfaces::msg::FixedPosePlan>(
        "~/fixed_pose_plan",rclcpp::QoS(1),
        [this](const ph_exo_interfaces::msg::FixedPosePlan::SharedPtr m) {
          if(!m || m->schema_version!=1 || !m->session_id ||
             m->sequence==0 || m->sequence>9007199254740991ULL ||
             (typed_session_ && m->session_id!=typed_session_))return;
          const std::array<double,12> f{1.,static_cast<double>(m->sequence),
            m->issued_at_monotonic_s,m->enabled?1.:0.,
            m->reference_rad[0],m->reference_rad[1],
            m->stiffness_nm_per_rad[0],m->stiffness_nm_per_rad[1],
            m->damping_nm_s_per_rad[0],m->damping_nm_s_per_rad[1],
            m->motor_torque_cap_nm[1],m->motor_torque_cap_nm[0]};
          if(set_plan(f))typed_session_=m->session_id;
        });
    }
    if(dynamic_) {
      ref_subscription_=node->create_subscription<ph_exo_interfaces::msg::ImpedanceRef>(
        "~/ref",rclcpp::QoS(1),
        [this](const ph_exo_interfaces::msg::ImpedanceRef::SharedPtr m) {
          if(!m || !m->id || !m->seq || m->seq>9007199254740991ULL ||
             (typed_session_ && m->id!=typed_session_))return;
          const std::array<double,12> f{1.,static_cast<double>(m->seq),m->t,m->on?1.:0.,
            m->q[0],m->q[1],m->k[0],m->k[1],m->b[0],m->b[1],m->tau[1],m->tau[0]};
          if(set_plan(f,m->dq,true))typed_session_=m->id;
        });
    }
    status_publisher_=node->create_publisher<ph_exo_interfaces::msg::ImpedanceStatus>("~/status",rclcpp::QoS(1));
    status_timer_=node->create_wall_timer(std::chrono::milliseconds(50),[this]() {
      SoftwareStatus snapshot;
      {std::lock_guard<std::mutex> lock(status_mutex_);snapshot=software_status_;}
      ph_exo_interfaces::msg::ImpedanceStatus message;
      message.session_id=typed_session_.load();
      message.mode=epoch_.load()>0.?static_cast<unsigned>(snapshot.observation.mode):0;
      message.reason_code=static_cast<unsigned>(snapshot.observation.reason);
      message.reason=ref01::reason_name(snapshot.observation.reason);
      message.observed_at_monotonic_s=snapshot.observed_at_monotonic_s;
      message.state_valid=message.mode==2;
      message.state_is_predicted=predictive_;
      message.position_rad=snapshot.observation.state.q;
      message.velocity_rad_s=snapshot.observation.state.dq;
      message.motor_target_nm=snapshot.observation.torque.motor_nm;
      // This is the oldest source used by the paired calculation, not two sample clocks.
      message.paired_source_time_monotonic_s=snapshot.observation.source_time_s;
      message.stop_requested=epoch_.load()>0. && stop_epoch_.load()==epoch_.load();
      {
        std::lock_guard<std::mutex> lock(plan_mutex_);
        message.accepted_sequence=static_cast<std::uint64_t>(last_plan_sequence_);
        message.reference_rad=last_reference_;
        message.dq_ref=last_reference_velocity_;
        message.dynamic=dynamic_;
      }
      status_publisher_->publish(message);
    });
    status_service_ = node->create_service<std_srvs::srv::Trigger>("~/software_status",
        [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          SoftwareStatus snapshot;
          {
            std::lock_guard<std::mutex> lock(status_mutex_);
            snapshot = software_status_;
          }
          // Only the atomic activation epoch is read outside the snapshot. In
          // particular, a skipped deactivate snapshot cannot report Following.
          if (epoch_.load() <= 0.) {
            snapshot.observation.mode = ref01::Mode::Inactive;
            snapshot.observation.reason = ref01::Reason::Inactive;
            snapshot.observation.refresh = false;
          }
          const auto& observed = snapshot.observation;
          std::ostringstream json;
          json.imbue(std::locale::classic());
          json << std::setprecision(17) << "{\"schema_version\":1,\"mode\":"
               << static_cast<int>(observed.mode) << ",\"reason_name\":\""
               << ref01::reason_name(observed.reason) << "\",\"generation\":"
               << observed.generation << ",\"refresh\":"
               << (observed.refresh ? "true" : "false") << ",\"source_time_s\":";
          append_json_number(json, observed.source_time_s);
          json << ",\"observed_at_monotonic_s\":";
          append_json_number(json, snapshot.observed_at_monotonic_s);
          json << ",\"scale\":";
          append_json_number(json, observed.torque.scale);
          json << ",\"compute_ms\":";
          append_json_number(json, 1000. * snapshot.compute_seconds);
          json << ",\"first_fault_timing\":{\"at_s\":";
          append_json_number(json,snapshot.first_fault.at_s);
          json << ",\"manager_period_s\":";
          append_json_number(json,snapshot.first_fault.manager_period_s);
          json << ",\"update_gap_s\":";
          append_json_number(json,snapshot.first_fault.update_gap_s);
          json << ",\"feedback_age_s\":[";
          append_json_number(json,snapshot.first_fault.feedback_age_s[0]);
          json << ',';
          append_json_number(json,snapshot.first_fault.feedback_age_s[1]);
          json << "]}";
          json << ",\"q\":[";
          append_json_number(json, observed.state.q[0]);
          json << ',';
          append_json_number(json, observed.state.q[1]);
          json << "],\"dq\":[";
          append_json_number(json, observed.state.dq[0]);
          json << ',';
          append_json_number(json, observed.state.dq[1]);
          json << "],\"closure_residual\":";
          append_json_number(json, observed.state.residual);
          json << ",\"state_is_predicted\":" << (predictive_?"true":"false")
               << ",\"source_time_basis\":\"" << (predictive_?"estimated_capture":"host_receive") << "\""
               << ",\"receive_time_uncertainty_s\":" << (predictive_?.007:0.) << '}';

          response->success = true;  // JSON schema availability, not control mode.
          response->message = json.str();
        });
  } catch (const std::exception&) { return Callback::ERROR; }
  return Callback::SUCCESS;
}

Configuration Ref01ImpedanceController::state_interface_configuration() const {
  Configuration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto& joint : joints_) {
    for (const auto field : state_fields) result.names.push_back(joint + "/" + field);
    if(predictive_)for(const auto* field:{"accepted_effort","accepted_time_s","accepted_sequence"})
      result.names.push_back(joint+"/"+field);
  }
  return result;
}

Configuration Ref01ImpedanceController::command_interface_configuration() const {
  Configuration result{controller_interface::interface_configuration_type::INDIVIDUAL, {}};
  for (const auto& joint : joints_) {
    result.names.push_back(joint + "/effort");
    result.names.push_back(joint + "/command_generation");
  }
  return result;
}

ref01::Feedback Ref01ImpedanceController::feedback() const noexcept {
  ref01::Feedback result{};
  const unsigned stride=predictive_?7:4;
  for (unsigned i = 0; i < 2; ++i)
    result[i] = {state_interfaces_[stride * i].get_value(), state_interfaces_[stride * i + 1].get_value(),
                 state_interfaces_[stride * i + 2].get_value(), state_interfaces_[stride * i + 3].get_value()};
  return result;
}

void Ref01ImpedanceController::zero_effort() noexcept {
  if (command_interfaces_.size() == 4) {
    command_interfaces_[0].set_value(0.);
    command_interfaces_[2].set_value(0.);
  }
}

void Ref01ImpedanceController::refresh_software_status(double observed_at) noexcept {
  std::unique_lock<std::mutex> lock(status_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) return;
  software_status_ = {observation(), observed_at, compute_seconds_, first_fault_};
}

Callback Ref01ImpedanceController::on_activate(const rclcpp_lifecycle::State&) {
  epoch_.store(0.);
  first_fault_={};previous_update_s_=0.;
  session_.deactivate();
  predictive_session_.deactivate();
  if (!model_) return Callback::ERROR;
  const auto states = state_interface_configuration().names;
  const auto commands = command_interface_configuration().names;
  if (state_interfaces_.size() != states.size() || command_interfaces_.size() != commands.size())
    return Callback::ERROR;
  for (unsigned i = 0; i < states.size(); ++i)
    if (state_interfaces_[i].get_name() != states[i]) return Callback::ERROR;
  for (unsigned i = 0; i < commands.size(); ++i)
    if (command_interfaces_[i].get_name() != commands[i]) return Callback::ERROR;
  zero_effort();
  const auto current = feedback();
  const double now = monotonic_seconds();
  for (unsigned i = 0; i < 2; ++i) {
    const auto& f = current[i];
    if (!std::isfinite(f.position) || !std::isfinite(f.velocity) ||
        !std::isfinite(f.source_time_s) || f.source_time_s <= 0. || f.source_time_s > now ||
        now - f.source_time_s >= ref01::feedback_ttl_s || !std::isfinite(f.sequence) ||
        f.sequence <= 0. || f.sequence > ref01::max_integer || std::floor(f.sequence) != f.sequence)
      return Callback::ERROR;
    generation_[i] = command_interfaces_[2 * i + 1].get_value();
    if (!std::isfinite(generation_[i]) || generation_[i] < 0. ||
        generation_[i] >= ref01::max_integer || std::floor(generation_[i]) != generation_[i])
      return Callback::ERROR;
  }
  if (std::abs(current[0].source_time_s - current[1].source_time_s) >= ref01::max_pair_skew_s)
    return Callback::ERROR;
  previous_alpha_ = {current[1].position - ref01::pi / 4., current[0].position - ref01::pi / 4.};
  ph_exo::ModelState initial{};
  // Bounded continuation and first-time MuJoCo work are warmed outside update.
  if (!model_->evaluate(previous_alpha_, {current[1].velocity, current[0].velocity}, initial))
    return Callback::ERROR;
  {std::lock_guard<std::mutex> lock(plan_mutex_);
    last_plan_sequence_=0.;last_plan_source_=0.;last_reference_={};last_reference_velocity_={};typed_session_=0;
    inbox_.writeFromNonRT(ref01::Plan{});
  }
  const double activation = monotonic_seconds();
  session_.activate(activation);
  predictive_session_.activate(activation);accepted_sequences_={};
  compute_seconds_ = 0.;
  epoch_.store(activation);
  refresh_software_status(activation);
  return Callback::SUCCESS;
}

Callback Ref01ImpedanceController::on_deactivate(const rclcpp_lifecycle::State&) {
  epoch_.store(0.);
  session_.deactivate();
  predictive_session_.deactivate();
  zero_effort();
  refresh_software_status(monotonic_seconds());
  return Callback::SUCCESS;
}

bool Ref01ImpedanceController::set_plan(const std::array<double, 12>& frame,
                                         const ref01::Pair& velocity, bool dynamic) {
  if(dynamic != dynamic_)return false;
  std::lock_guard<std::mutex> lock(plan_mutex_);
  const double epoch = epoch_.load();
  ref01::Plan plan;
  if (!ref01::valid_plan(frame, monotonic_seconds(), epoch,
                         last_plan_sequence_, last_plan_source_, plan, bench_?10.:.2, bench_, dynamic, velocity)) return false;
  if (epoch_.load() != epoch) return false;
  last_plan_sequence_ = plan.sequence;
  last_plan_source_ = plan.issued_at;
  last_reference_ = plan.reference;
  last_reference_velocity_ = plan.reference_velocity;
  inbox_.writeFromNonRT(plan);
  return true;
}

controller_interface::return_type Ref01ImpedanceController::update(
    const rclcpp::Time&, const rclcpp::Duration& period) {
  const double start = monotonic_seconds();
  if (epoch_.load() <= 0.) {
    refresh_software_status(start);
    return controller_interface::return_type::OK;
  }
  const auto input_feedback=feedback();
  const double update_gap=previous_update_s_>0.?start-previous_update_s_:0.;
  previous_update_s_=start;
  const auto plan = *inbox_.readFromRT();
  if (stop_epoch_.load()==epoch_.load()) reject(ref01::Reason::Disabled);
  if(predictive_) {
    for(unsigned i=0;i<2;++i) {
      const double effort=state_interfaces_[7*i+4].get_value();
      const double time=state_interfaces_[7*i+5].get_value();
      const double sequence=state_interfaces_[7*i+6].get_value();
      if(!std::isfinite(sequence)||sequence<0.||sequence>ref01::max_integer||std::floor(sequence)!=sequence ||
         !std::isfinite(effort)||!std::isfinite(time)) {reject(ref01::Reason::InvalidTorque);break;}
      if(sequence==0.) {
        if(accepted_sequences_[i]>0.)reject(ref01::Reason::InvalidTorque);
        continue;
      }
      if(time<epoch_.load() && accepted_sequences_[i]==0.)continue;
      if(sequence<accepted_sequences_[i] ||
         !predictive_session_.receipt(i,{time,effort,static_cast<std::uint64_t>(sequence)})) {
        reject(ref01::Reason::InvalidTorque);break;
      }
      accepted_sequences_[i]=sequence;
    }
    predictive_session_.update(input_feedback,plan,start,period.seconds(),
      [this](const ref01::Pair& alpha,const ref01::Pair& velocity,ref01::ModelState& result,
             std::array<double,4>& mass) noexcept {
        for(unsigned i=0;i<2;++i)if(std::abs(alpha[i]-previous_alpha_[i])>.15)return false;
        ph_exo::ModelState native{};
        if(!model_->evaluate(alpha,velocity,native))return false;
        result={native.q,native.dq,native.gravity,native.A,native.residual,native.iterations};
        mass=native.mass;previous_alpha_=alpha;return true;
      });
  } else {
  session_.update(input_feedback, plan, start, period.seconds(),
      [this](const ref01::Pair& alpha, const ref01::Pair& velocity, ref01::ModelState& result) noexcept {
        for (unsigned i = 0; i < 2; ++i)
          if (std::abs(alpha[i] - previous_alpha_[i]) > .15) return false;
        ph_exo::ModelState native{};
        if (!model_->evaluate(alpha, velocity, native)) return false;
        result = {native.q, native.dq, native.gravity, native.A, native.residual, native.iterations};
        previous_alpha_ = alpha;
        return true;
      });
  }
  const double finished = monotonic_seconds();
  compute_seconds_ = finished - start;
  // Reject a late result before publishing command generation. This gate is
  // measured monotonic compute time, independent of manager's reported period.
  if (compute_seconds_ >= .005) reject(ref01::Reason::ComputeDeadline);
  else if (observation().refresh &&
           finished - observation().source_time_s >= ref01::feedback_ttl_s)
    reject(ref01::Reason::FeedbackExpired);
  else if (observation().refresh && finished - plan.issued_at >= ref01::plan_ttl_s)
    reject(ref01::Reason::PlanExpired);
  const auto& output = observation();
  if(output.mode==ref01::Mode::Faulted && first_fault_.at_s==0.) {
    first_fault_={start,period.seconds(),update_gap,
      {start-input_feedback[0].source_time_s,start-input_feedback[1].source_time_s}};
  }
  if (output.mode == ref01::Mode::Faulted) {
    zero_effort();
  } else if (output.refresh) {
    if (generation_[0] >= ref01::max_integer || generation_[1] >= ref01::max_integer) {
      reject(ref01::Reason::InvalidTorque);
      zero_effort();
    } else {
      for (unsigned i = 0; i < 2; ++i) {
        // Choose an interior input for the existing truncating wire codec.
        // The encoded-grid target is recorded separately by the hardware receipt.
        const double value=predictive_?
            std::min(plan.caps[1-i],output.torque.motor_nm[i]+.25*30./4095.):output.torque.motor_nm[i];
        command_interfaces_[2 * i].set_value(value);
        command_interfaces_[2 * i + 1].set_value(++generation_[i]);
      }
    }
  }
  refresh_software_status(monotonic_seconds());
  return controller_interface::return_type::OK;
}
}  // namespace mech::mech_controllers
