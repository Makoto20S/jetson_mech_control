#include "mech_bringup/ak30_force_runtime.hpp"
#include "mech_simulation/fake_transport.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <iostream>
using namespace mech;
int main(){
 for(int scenario=0;scenario<7;++scenario){
  mech_simulation::FakeTransport transport(16);
  std::int64_t ns=2000000;
  auto now=[&](){return mech_control_core::MonotonicTime::from_nanoseconds(ns).value();};
  mech_bringup::Ak30RuntimeConfig cfg;
  cfg.sub_mode=mech_protocol_cubemars::ForceControlSubMode::Torque;
  cfg.feedback_period_nanoseconds=1000000;cfg.feedback_ttl_nanoseconds=6000000;
  mech_bringup::Ak30ForceControlRuntime rt(transport,now,cfg);
  assert(rt.configure(1));assert(rt.start());
  mech_hardware_ros2_control::CanonicalState state{};
  assert(rt.submit_pending_torque()); // no command, no startup feedback
  std::array<std::uint8_t,64> bytes{};bytes[0]=3;bytes[1]=132;bytes[6]=40;
  auto frame=mech_control_core::RawCanFrame::create(1,
   mech_control_core::CanId::create(mech_protocol_cubemars::feedback_can_id(104),mech_control_core::CanFrameFormat::Extended).value(),
   mech_control_core::CanFrameType::Classic,mech_control_core::FrameDirection::Rx,8,bytes,now()).value();
  assert(transport.inject_receive(frame)==mech_control_core::TransportResult::Ok);
  assert(rt.read(&state,1));assert(transport.pending_transmit()==0);
  mech_hardware_ros2_control::CommandDispatch cmd{};cmd.command.effort=2;cmd.authorized=true;cmd.fresh=true;
  if(scenario==1)cmd.authorized=false;
  if(scenario==2)cmd.fresh=false;
  assert(rt.write(&cmd,1));assert(transport.pending_transmit()==0);
  if(scenario==3)rt.cancel_pending(0);
  if(scenario==4)ns+=6000001;
  if(scenario==5)transport.force_next_send_results({mech_control_core::TransportResult::WouldBlock});
  if(scenario==6)rt.stop();
  const bool ok=scenario==6?rt.submit_pending_torque():rt.submit_active_torque();
  if(scenario==4 || scenario==6){
    assert(!ok);assert(transport.pending_transmit()==0);
    if(scenario==6){
      for(int i=0;i<20;++i){ns+=5000000;assert(rt.submit_active_torque());}
      assert(transport.pending_transmit()==0);
      assert(rt.configure(1));assert(rt.start());assert(rt.submit_active_torque());
      assert(transport.pending_transmit()==0); // Restart never replays old effort.
    } // Scenario 4 already checked the first active expired submission.
    continue;
  }
  assert(ok);
  if(scenario==0){
    assert(transport.pending_transmit()==1);
    assert(rt.accepted_torque().host_time_s==.002);
    assert(rt.submit_pending_torque());assert(transport.pending_transmit()==1);
  }else if(scenario==5){
    assert(transport.pending_transmit()==0);ns+=1000000;
    assert(rt.submit_pending_torque());assert(transport.pending_transmit()==1);
  }else assert(transport.pending_transmit()==0);
 }
 std::cout<<"7 same-cycle runtime scenarios + inactive/restart PASS\n";
}