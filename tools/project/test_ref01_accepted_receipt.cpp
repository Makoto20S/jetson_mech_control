#include "ak30_test_fixtures.hpp"
#include <iostream>
#include <cmath>
#include <limits>
namespace core=mech::mech_control_core;
namespace w=mech::mech_protocol_cubemars;
namespace f=w::testing;
int count=0,errors=0;
void check(bool value,const char* name){++count;if(!value){++errors;std::cerr<<name<<'\n';}}
int main(){
 f::RecordingTransport transport(f::classic_extended_capabilities());
 w::Ak30ForceControlSession session(transport,f::valid_session_config());
 check(session.configure(f::valid_device_config(),f::classic_extended_capabilities())==core::AdapterResult::Ok,"configure");
 check(!session.last_accepted_frame(),"no invented receipt");
 check(session.activate()==core::AdapterResult::Ok,"activate");
 check(!session.last_accepted_frame()&&session.accepted_command_sequence()==0,"enable is not torque acceptance");
 core::CanonicalDeviceCommand command{};command.effort=.123;command.deadline=f::at(1004000000);
 check(session.submit(command,f::at(1000000000))==core::AdapterResult::Ok,"accepted command");
 auto frame=*session.last_accepted_frame();
 check(session.accepted_command_sequence()==1,"accepted sequence");
 check(frame.host_arrival.nanoseconds()==1000000000,"accept time");
 check((((unsigned(frame.payload[6])&15)<<8)|frame.payload[7])==2064,"independent torque code");
 for(auto result:{core::TransportResult::WouldBlock,core::TransportResult::QueueFull,core::TransportResult::Disconnected}){
  transport.inject_send_result(result);
  check(session.submit(command,f::at(1001000000))!=core::AdapterResult::Ok,"failed send not accepted");
  check(session.accepted_command_sequence()==1&&session.last_accepted_frame()->host_arrival==frame.host_arrival,"failed send retains old receipt");
 }
 transport.inject_send_result(core::TransportResult::Ok);
 check(session.submit(command,f::at(1002000000))==core::AdapterResult::Ok,"retry accepted");
 check(session.accepted_command_sequence()==2&&session.last_accepted_frame()->host_arrival.nanoseconds()==1002000000,"retry gets actual host time");
 check(session.submit(command,f::at(1004000000))==core::AdapterResult::InvalidCommand,"expired command rejected");
 check(session.accepted_command_sequence()==2,"expiry not receipt");
 command.effort=std::numeric_limits<double>::quiet_NaN();
 check(session.submit(command,f::at(1003000000))!=core::AdapterResult::Ok&&session.accepted_command_sequence()==2,"invalid command not receipt");
 session.deactivate();check(!session.last_accepted_frame(),"deactivate clears live receipt");
 check(session.activate()==core::AdapterResult::Ok&&!session.last_accepted_frame(),"reactivation no old receipt");
 // Every target selected inside the bench cap must round-trip without ever
 // requesting an input outside that cap, including the exact +5 Nm boundary.
 for(unsigned code=1365;code<=2730;++code){
  const double selected=w::dequantize(code,-15.,15.,12);
  w::ForceControlCommand cmd;cmd.torque_nm=std::min(5.,selected+.25*30./4095.);
  w::ForceControlPayload payload;
  check(w::encode_force_control(cmd,w::ake60_8_ranges(),payload),"interior encodes");
  check((((unsigned(payload[6])&15)<<8)|payload[7])==code,"interior code matches");
  check(std::abs(cmd.torque_nm)<=5.,"input cap honored");
 }
 std::cout<<count<<" receipt/quantization checks, "<<errors<<" failures\n";
 return errors?1:0;
}
