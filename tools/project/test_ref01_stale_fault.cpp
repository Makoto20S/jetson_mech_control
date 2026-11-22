// Portable composition test: unchanged core, lease and fault precedence.
// No ROS, clocks, file I/O or physical device.
#include "ref01_impedance_core.hpp"
#include "mech_bringup/ak30_kinematic_lease.hpp"
#include <iostream>
namespace c=mech::mech_controllers::ref01;
int failures=0,checks=0;
void check(bool value,const char* message){++checks;if(!value){++failures;std::cerr<<message<<"\n";}}
c::Reason run(bool blocking_dump) {
  c::Session session;session.activate(10.);
  mech::mech_bringup::KinematicCommandLease lease;lease.reset(0.);
  c::Plan plan{true,true,10.,1.,10.002,{0.,c::pi/9.},{2.4,2.},{.18,.15},{.2,.2}};
  c::Feedback feedback{{{c::pi/4.,0.,10.005,1.},{c::pi/4.,0.,10.005,1.}}};
  auto model=[](c::Pair,c::Pair,c::ModelState& s){
    s={{0.,c::pi/9.},{0.,0.},{.05,.05},{1.,0.,0.,1.},0.,0};return true;
  };
  bool leased=false;double first_cancel=0.,fault_at=0.;
  for(int i=0;i<18;++i) {
    double now=10.006+i*.005;
    // A manager error callback's synchronous dump extends the next update.
    if(blocking_dump && i>=10)now+=.012;
    const auto o=session.update(feedback,plan,now,(blocking_dump&&i==10)? .017:.005,model);
    if(o.mode==c::Mode::Faulted) {
      fault_at=now;
      check(!o.refresh,"fault cannot generate a command");
      const auto again=session.update(feedback,plan,now+.005,.005,model);
      check(again.reason==o.reason,"first fault remains latched");
      break;
    }
    const bool next=lease.update(o.torque.motor_nm[0],static_cast<double>(o.generation),
                                feedback[0].source_time_s,true,static_cast<std::int64_t>(now*1e9));
    if(leased&&!next)first_cancel=now;
    leased=next;
  }
  check(first_cancel>10.04 && first_cancel<10.055,"40ms lease expires first");
  if(blocking_dump) {
    check(session.observation().reason==c::Reason::Period,"blocking cleanup masks expiry");
  } else {
    check(session.observation().reason==c::Reason::FeedbackExpired,"continuous updates observe source expiry");
    check(fault_at-feedback[0].source_time_s>=c::feedback_ttl_s,"expiry uses original source");
  }
  return session.observation().reason;
}
int main(){run(true);run(false);std::cout<<"checks="<<checks<<" failures="<<failures<<"\n";return failures?1:0;}
