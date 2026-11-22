#include "mech_bringup/ak30_kinematic_lease.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>
using mech::mech_bringup::KinematicCommandLease;
int checked=0;
void check(bool ok){++checked;if(!ok)throw std::runtime_error("lease check failed");}
int main(){
  try{
    KinematicCommandLease x;x.reset(0);
    check(!x.update(0,0,1,true,1000000000));
    check(x.update(.1,1,1,true,1000000000));
    check(x.update(.1,1,1,true,1039999999));
    check(!x.update(.1,2,1.04,true,1040000000)); // new target cannot rescue old expiry
    check(x.faulted());
    check(!x.update(.1,3,1.05,true,1050000000));
    x.reset(5);check(!x.update(.1,5,1.05,true,1050000000)); // no old-generation replay
    check(x.update(.1,6,1.05,true,1050000000));
    check(!x.update(.2,6,1.05,true,1051000000)); // mutation cannot refresh
    x.reset(0);check(!x.update(.21,1,1,true,1000000000));
    x.reset(0);check(!x.update(.1,1,.94,true,1000000001));
    x.reset(0);check(!x.update(.1,1,1,false,1000000000));
    x.reset(0);check(!x.update(.1,1.5,1,true,1000000000));
    x.reset(0);check(x.update(.1,1,1,true,1000000000));
    check(!x.update(.1,2,1,true,999999999));
    x.reset(0);check(x.update(.1,1,1,true,1000000000));
    check(!x.update(.1,1,1,false,1005000000));
    x.reset(0);check(x.update(.1,1,1,true,1000000000));
    check(x.update(.15,2,1.02,true,1020000000));
    check(x.update(.15,2,1.02,true,1059999999));
    check(!x.update(.15,2,1.02,true,1060000000));

    using mech::mech_bringup::KinematicBenchEnvelope;
    KinematicBenchEnvelope b;b.reset(80.);
    check(!b.update(false,false,0,0,0,0,0,1000000000,.005)); // unopened feedback is not a latch
    check(b.update(true,false,5.,80,0,0,40,1005000000,.005));
    check(std::abs(b.effort()-.0025)<1e-12); // 0.5 Nm/s, not a 5 Nm step
    check(b.update(true,false,-5.,80,0,0,40,1010000000,.100));
    check(std::abs(b.effort())<1e-12); // late tick cannot enlarge ramp
    check(b.update(true,false,.1,170.1,0,0,40,1015000000,.005));
    check(b.stopped() && b.effort()==0);
    check(!b.update(true,false,.1,80,0,0,40,1020000000,.005));
    b.reset(80);check(b.update(true,false,.1,80,0,0,40,1000000000,.005));
    check(b.update(true,false,.1,80,0,0,40,31000000000LL,.005));
    check(b.stopped() && b.effort()==0);
    for(int fault=0;fault<5;++fault) {
      b.reset(80);check(b.update(true,false,.1,80,0,0,40,1000000000,.005));
      check(b.update(fault!=0,fault==1,fault==2?10.01:.1,80,0,
                     fault==3?6.01:0.,fault==4?60.:40.,1005000000,.005));
      check(b.stopped() && b.effort()==0);
    }
    x.reset(0,5.);check(x.update(5,1,1,true,1000000000));
    x.reset(0,5.);check(!x.update(5.01,1,1,true,1000000000));
    x.reset(0,10.01);check(!x.update(.1,1,1,true,1000000000));
    b.reset(80.);check(b.update(true,false,.1,169.9,100.,5.99,59.,1000000000,.005));
    check(!b.stopped());
    for(double speed:{-327680.,-250.,120.,327670.}) {
      b.reset(80.);check(b.update(true,false,.1,80.,speed,0.,40.,1000000000,.005));
      check(!b.stopped());
    }
    b.reset(80.);check(b.update(true,false,.1,80.,0.,0.,40.,1000000000,.005));
    check(b.update(true,false,.1,80.,std::numeric_limits<double>::quiet_NaN(),0.,40.,1005000000,.005));
    check(b.stopped());

    // The production supported bench now selects Direct: no delayed reversal,
    // reduction or zero crossing. Limits and latched faults still apply.
    b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct);
    std::int64_t tick=1000000000;
    for(double target:{.4,-.4,.1,0.,5.,-5.,10.,-10.}) {
      check(b.update(true,false,target,80.,0.,0.,40.,tick,.005));
      check(b.effort()==target && !b.stopped());
      tick+=5000000;
    }
    for(int fault=0;fault<8;++fault) {
      b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct);
      check(b.update(true,false,.4,80.,0.,0.,40.,1000000000,.005));
      check(b.update(fault!=0,fault==1,fault==2?10.01:-.4,
                     fault==5?170.1:80.,0.,fault==3?6.01:0.,fault==4?60.:40.,
                     fault==6?31000000000LL:1005000000LL,
                     fault==7?0.:.005));
      check(b.stopped() && b.effort()==0.);
      check(!b.update(true,false,-.4,80.,0.,0.,40.,31005000000LL,.005));
    }
    for(double sign:{-1.,1.}) {
      b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct);
      check(b.update(true,false,sign*10.,80.+sign*90.,0.,sign*6.,40.,1000000000,.005));
      check(!b.stopped() && b.effort()==sign*10.);
      x.reset(0,10.);check(x.update(sign*10.,1,1,true,1000000000));
    }
    // Manual mode has no elapsed-time cutoff; every other gate still applies.
    KinematicBenchEnvelope::Limits limits;limits.duration_ns=0;
    limits.torque_nm=2.;limits.current_a=3.;limits.travel_deg=15.;
    check(limits.valid());
    b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct,limits);
    for(auto t:{1000000000LL,31000000000LL,3601000000000LL}) {
      check(b.update(true,false,2.,95.,100.,3.,40.,t,.005));
      check(!b.stopped() && b.effort()==2.);
    }
    for(int which=0;which<5;++which) {
      b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct,limits);
      check(b.update(true,false,1.,80.,0.,0.,40.,1000000000,.005));
      check(b.update(which!=3,which==4,which==0?2.01:1.,
                     which==1?95.01:80.,0.,which==2?3.01:0.,40.,31000000000LL,.005));
      check(b.stopped() && b.effort()==0.);
    }
    for(int which=0;which<4;++which) {
      auto invalid=limits;
      if(which==0)invalid.torque_nm=0.;
      if(which==1)invalid.current_a=6.01;
      if(which==2)invalid.travel_deg=std::numeric_limits<double>::quiet_NaN();
      if(which==3)invalid.duration_ns=-1;
      check(!invalid.valid());
    }
    // Transition travel is bounded in time, current/torque remain unchanged.
    KinematicBenchEnvelope::Limits transition;transition.duration_ns=0;transition.travel_deg=60.;transition.startup_s=4.;
    check(transition.valid());
    b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct,transition);
    check(b.update(true,false,1.,170.,0.,6.,40.,1000000000LL,.005));
    check(!b.stopped());
    check(b.update(true,false,1.,170.,0.,6.,40.,4999999999LL,.005));
    check(!b.stopped());
    check(b.update(true,false,1.,170.,0.,6.,40.,5000000000LL,.005));
    check(b.stopped() && b.effort()==0.);
    b.reset(80.,KinematicBenchEnvelope::EffortPolicy::Direct,transition);
    check(!b.update(true,false,1.,80.,0.,6.01,40.,1000000000LL,.005));
    check(b.stopped());
    transition.startup_s=61.01;check(!transition.valid());
    std::cout<<"bounded hold: "<<checked<<" checks passed\n";
  }catch(const std::exception& e){std::cerr<<e.what();return 1;}
}
