#include "mech_bringup/ak30_system.hpp"
#include "mech_bringup/ak30_kinematic_lease.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include "pluginlib/class_list_macros.hpp"

namespace mech::mech_bringup {
// Reuses production CDC, codec, session and TTL machinery. Physical endpoints
// require the explicit supported-bench profile; simulation remains PTY-only.
class Ak30KinematicSystem final : public Ak30System {
 public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override {
    using R=hardware_interface::CallbackReturn;
    try {
      auto forwarded=info;
      auto& p=forwarded.hardware_parameters;
      bench_=p.count("bench_profile") && p.at("bench_profile")=="ref01_supported_v1";
      const bool sim=p.at("simulation_only")=="true";
      const bool rehearsal=p.count("bench_rehearsal") && p.at("bench_rehearsal")=="true";
      if (sim==bench_ || p.at("sub_mode")!="torque" ||
          std::stod(p.at("kp"))!=0 || std::stod(p.at("kd"))!=0 || info.joints.size()!=1) return R::ERROR;
      const auto path=p.at("device_path");
      if(sim || rehearsal) {
        if(path.rfind("/dev/pts/",0)!=0) return R::ERROR;
        const auto suffix=path.substr(9);
        if(suffix.empty() || !std::all_of(suffix.begin(),suffix.end(),[](char c){return c>='0' && c<='9';}))
          return R::ERROR;
      } else {
        const auto id=p.at("drive_id");
        if((id!="104" && id!="105") || p.at("logical_bus")!=(id=="104"?"2":"1") ||
           path!="/dev/serial/by-id/usb-Livelybot_Livelybot_Communication_Board_2024051701-"+std::string(id=="104"?"if02":"if00"))
          return R::ERROR;
      }
      if(bench_) {
        auto limit=[&p](const char* key,double fallback) {
          if(!p.count(key))return fallback;
          std::size_t used{};double value=std::stod(p.at(key),&used);
          if(used!=p.at(key).size())throw std::invalid_argument(key);
          p.erase(key);return value;
        };
        limits_.torque_nm=limit("bench_torque_limit_nm",10.);
        limits_.current_a=limit("bench_current_limit_a",6.);
        limits_.travel_deg=limit("bench_travel_limit_deg",90.);
        limits_.startup_s=limit("bench_startup_s",0.);
        if(p.count("bench_manual_stop")) {
          const auto value=p.at("bench_manual_stop");
          if(value!="true" && value!="false")return R::ERROR;
          limits_.duration_ns=value=="true"?0:30000000000LL;
          p.erase("bench_manual_stop");
        }
        if(!limits_.valid())return R::ERROR;
        std::size_t used{};
        reference_=std::stod(p.at("bench_reference_deg"),&used);
        if(used!=p.at("bench_reference_deg").size() || !std::isfinite(reference_) ||
           reference_<=2. || reference_>=208. || (p.count("simulation_freeze_after_s") && !rehearsal)) return R::ERROR;
      }
      p.erase("bench_profile");p.erase("bench_rehearsal");p.erase("bench_reference_deg");
      std::size_t end{};
      scale_=std::stod(p.at("kinematic_erpm_to_rad_s"), &end);
      if (end!=p.at("kinematic_erpm_to_rad_s").size() || !std::isfinite(scale_) || scale_<=0 || scale_>.1)
        return R::ERROR;
      p.erase("simulation_only"); p.erase("kinematic_erpm_to_rad_s");
      if (p.count("simulation_freeze_after_s")) {
        freeze_after_=std::stod(p.at("simulation_freeze_after_s"));
        freeze_duration_=std::stod(p.at("simulation_freeze_duration_s"));
        if (!std::isfinite(freeze_after_) || !std::isfinite(freeze_duration_) ||
            freeze_after_<=0 || freeze_duration_<=0 || freeze_duration_>1.) return R::ERROR;
        p.erase("simulation_freeze_after_s"); p.erase("simulation_freeze_duration_s");
      }
      joint_=info.joints.front().name;
      auto& fields=forwarded.joints.front().state_interfaces;
      unsigned receipt_fields=0;
      for(auto name:{"accepted_effort","accepted_time_s","accepted_sequence"})
        receipt_fields+=std::count_if(fields.begin(),fields.end(),[name](const auto& f){return f.name==name;});
      if(receipt_fields!=0 && receipt_fields!=3)return R::ERROR;
      receipt_interfaces_=receipt_fields==3;
      if(receipt_interfaces_)for(auto name:{"accepted_effort","accepted_time_s","accepted_sequence"}) {
        if(std::count_if(fields.begin(),fields.end(),[name](const auto& f){return f.name==name;})!=1)return R::ERROR;
        fields.erase(std::remove_if(fields.begin(),fields.end(),[name](const auto& f){return f.name==name;}),fields.end());
      }
      for (auto name : {"sample_time_s","sequence"}) {
        if (std::count_if(fields.begin(),fields.end(),[name](const auto& f){return f.name==name;})!=1)
          return R::ERROR;
        fields.erase(std::remove_if(fields.begin(),fields.end(),[name](const auto& f){return f.name==name;}),fields.end());
      }
      for (auto name : {"position","velocity","mit_rx_sequence","mit_rx_us","mit_valid","mit_erpm","mit_position_deg"}) {
        if (std::count_if(fields.begin(),fields.end(),[name](const auto& f){return f.name==name;})!=1)
          return R::ERROR;
      }
      return Ak30System::on_init(forwarded);
    } catch (const std::exception&) { return R::ERROR; }
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    auto original=Ak30System::export_state_interfaces();
    std::vector<hardware_interface::StateInterface> result;
    result.reserve(original.size()+2);
    for(auto& item:original) {
      if(item.get_interface_name()!="position" && item.get_interface_name()!="velocity")
        result.emplace_back(std::move(item));
    }
    const char* names[]={"position","velocity","sample_time_s","sequence"};
    for (unsigned i=0;i<4;++i) result.emplace_back(joint_,names[i],&state_[i]);
    if(receipt_interfaces_) {
      const char* receipt_names[]={"accepted_effort","accepted_time_s","accepted_sequence"};
      for(unsigned i=0;i<3;++i)result.emplace_back(joint_,receipt_names[i],&accepted_[i]);
    }
    return result;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    internal_commands_=Ak30System::export_command_interfaces();
    std::vector<hardware_interface::CommandInterface> result;
    result.reserve(2);
    result.emplace_back(joint_,"effort",&external_effort_);
    result.emplace_back(joint_,"command_generation",&external_generation_);
    return result;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous) override {
    if(internal_commands_.size()!=2 || internal_commands_[0].get_interface_name()!="effort" ||
       internal_commands_[1].get_interface_name()!="command_generation")
      return hardware_interface::CallbackReturn::ERROR;
    lease_.reset(external_generation_,bench_?limits_.torque_nm:.2);
    // Supported bench: follow the controller target without a secondary slow ramp.
    envelope_.reset(reference_, KinematicBenchEnvelope::EffortPolicy::Direct, limits_);
    first_command_ns_=0;accepted_={};
    return Ak30System::on_activate(previous);
  }
  hardware_interface::return_type read(const rclcpp::Time& time,const rclcpp::Duration& period) override {
    if(auto* trace=timing_trace()) {
      ++trace->cycle;trace->record("kinematic_read_begin",0,0,period.seconds());
    }
    auto result=Ak30System::read(time,period);
    const auto& raw=kinematic_raw_feedback();
    if(receipt_interfaces_) {
      const auto receipt=kinematic_accepted_torque();
      accepted_={receipt.effort_nm,receipt.host_time_s,static_cast<double>(receipt.sequence)};
    }
    valid_=raw[3]==1. && raw[8]==0. && std::abs(raw[6])<=(bench_?limits_.current_a:.5) && raw[7]<=60. &&
           std::isfinite(raw[4]) && std::isfinite(raw[5]);
    const double elapsed=first_command_ns_ ? (std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count()-first_command_ns_)*1e-9 : 0.;
    const bool frozen=freeze_after_>0 && elapsed>=freeze_after_ && elapsed<freeze_after_+freeze_duration_;
    if (valid_ && !frozen) state_={raw[4]*3.14159265358979323846/180., raw[5]*scale_, raw[2]*1e-6, raw[1]};
    if(auto* trace=timing_trace()) trace->record("kinematic_read_end",0,
      static_cast<std::uint64_t>(state_[3]),state_[2],raw[2]*1e-6,
      frozen?1:0,0,static_cast<int>(result));
    // Keep the ORIGINAL source fields on an invalid/unchanged observation.
    // write() independently cancels its hold; base propagates runtime errors.
    return result;
  }
  hardware_interface::return_type write(const rclcpp::Time& time,const rclcpp::Duration& period) override {
    const auto now=std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    if (internal_commands_.size()!=2) return hardware_interface::return_type::ERROR;
    bool refresh=lease_.update(external_effort_,external_generation_,state_[2],valid_,now);
    if(auto* trace=timing_trace()) trace->record("kinematic_lease",0,
      (std::isfinite(external_generation_) && external_generation_>=0. &&
       external_generation_<=9007199254740991.)?static_cast<std::uint64_t>(external_generation_):0,
      state_[2],external_effort_,
      lease_.faulted()?1:0,0,refresh?1:0);
    double effort=lease_.effort();
    if(bench_) {
      const auto& raw=kinematic_raw_feedback();
      refresh=envelope_.update(refresh,lease_.faulted(),effort,raw[4],raw[5],raw[6],raw[7],now,period.seconds());
      effort=envelope_.effort();
    }
    if(refresh) {
      internal_commands_[0].set_value(effort);
      if (!first_command_ns_) first_command_ns_=now;
      internal_commands_[1].set_value(internal_commands_[1].get_value()+1.);
    }
    const auto result=Ak30System::write(time,period);
    if(result!=hardware_interface::return_type::OK)return result;
    // Lease/envelope and base claim/generation checks run before same-cycle TX.
    if(!submit_kinematic_torque())return hardware_interface::return_type::ERROR;
    return result;
  }
 private:
  std::vector<hardware_interface::CommandInterface> internal_commands_;
  std::string joint_;
  double scale_{},external_generation_{},external_effort_{};
  std::array<double,4> state_{{0.,0.,0.,0.}};
  bool valid_{false},receipt_interfaces_{false};
  std::array<double,3> accepted_{};
  KinematicCommandLease lease_;
  KinematicBenchEnvelope envelope_;
  KinematicBenchEnvelope::Limits limits_{};
  bool bench_{};
  double reference_{};
  std::int64_t first_command_ns_{};
  double freeze_after_{},freeze_duration_{};
};
}  // namespace mech::mech_bringup
PLUGINLIB_EXPORT_CLASS(mech::mech_bringup::Ak30KinematicSystem, hardware_interface::SystemInterface)
