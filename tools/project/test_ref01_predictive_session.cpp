#include "ref01_predictive_session.hpp"
#include <iostream>
#include <limits>
namespace c=mech::mech_controllers::ref01;
double bound=0.;
struct S:ph_exo::control::PredictiveSession {S(){set_receive_time_bound(bound);}};
int checks=0,errors=0;
void check(bool value,const char* name){++checks;if(!value){++errors;std::cerr<<name<<'\n';}}
bool model(const c::Pair& a,const c::Pair& v,c::ModelState& out,std::array<double,4>& mass){
  out={a,v,{0.,0.},{1.,0.,0.,1.},0.,1};mass={.005,0.,0.,.005};return true;
}
c::Plan plan(double now=1.005){return {true,true,1.,1.,now,{0.,.2},{10.,10.},{1.,1.},{5.,5.}};}
c::Feedback feedback(double time=1.004,double seq=1.){
  return {{{c::pi/4.+.2,0.,time,seq},{c::pi/4.,0.,time,seq}}};
}
void start(S& s){s.activate(1.);auto o=s.update(feedback(),plan(),1.005,.005,model);check(o.refresh&&o.mode==c::Mode::Following,"valid startup");}
int main(int argc,char**){
  if(argc>1)bound=.007;
  {S s;check(!s.update(feedback(),plan(),1.005,.005,model).refresh,"inactive no command");}
  {S s;s.activate(1.);auto p=plan();p.valid=false;check(!s.update(feedback(),p,1.005,.005,model).refresh,"no plan no command");}
  {S s;s.activate(1.);check(!s.update(feedback(.999),plan(),1.005,.005,model).refresh,"preactivation cannot send");}
  {S s;start(s);const double source=s.observation().source_time_s;auto generation=s.observation().generation;
   for(int i=2;i<=12;++i){
     const double now=1.+i*.005;
     check(s.receipt(0,{now,s.observation().torque.motor_nm[0],static_cast<unsigned long long>(i-1)}),"receipt104");
     check(s.receipt(1,{now,s.observation().torque.motor_nm[1],static_cast<unsigned long long>(i-1)}),"receipt105");
     auto o=s.update(feedback(),plan(now),now,.005,model);
     check(o.refresh&&o.generation==++generation,"continuous prediction refresh");
     check(o.source_time_s==source,"prediction never renews original sample");
   }
   auto o=s.update(feedback(),plan(1.065),1.065,.005,model);
   check(!o.refresh&&o.mode==c::Mode::Faulted&&o.reason==c::Reason::FeedbackExpired,"feedback loss stops");
   check(!s.update(feedback(1.07,2.),plan(1.07),1.07,.005,model).refresh,"feedback recovery does not restart");
  }
  for(int mode=0;mode<6;++mode){S s;start(s);auto f=feedback();auto p=plan(1.01);
    if(mode==0)f[0].position+=.001;
    if(mode==1)f[0].sequence=0.;
    if(mode==2)f[0].source_time_s=1.011;
    if(mode==3)p.enabled=false;
    if(mode==4)p.reference[0]+=.001;
    if(mode==5)p.epoch=.9;
    const auto o=s.update(f,p,1.01,.005,model);
    check(!o.refresh&&o.mode==c::Mode::Faulted,"invalid input stops");
  }
  {S s;start(s);check(!s.update(feedback(),plan(1.015),1.015,.010,model).refresh,"10 ms boundary stops");}
  {S s;start(s);check(s.receipt(0,{1.01,0.,1}),"first receipt");
   check(!s.receipt(0,{1.015,0.,3}),"missing receipt sequence stops");
   check(s.observation().mode==c::Mode::Faulted,"receipt gap latches");
  }
  {S s;start(s);check(s.receipt(0,{1.02,0.,1}),"receipt queued for validation");
   check(!s.update(feedback(),plan(1.01),1.01,.005,model).refresh,"future receipt cannot predict");
  }
  {S s;s.activate(1.);auto p=plan();p.reference[1]+=.02;
   check(!s.update(feedback(),p,1.005,.005,model).refresh,"startup target mismatch stops");
  }
  {S s;start(s);s.deactivate();check(!s.observation().refresh&&s.observation().mode==c::Mode::Inactive,"deactivate revokes");
   s.activate(2.);auto p=plan(2.005);p.epoch=2.;
   check(s.update(feedback(2.004,1),p,2.005,.005,model).refresh,"explicit new epoch can restart");
  }
  // New axes can first appear at different instants. Waiting for the second
  // bracketing sample is valid startup behavior, not a fabricated common stamp.
  {S s;s.activate(1.);auto f=feedback();f[1].source_time_s=1.006;
   auto o=s.update(f,plan(1.007),1.007,.005,model);
   check(!o.refresh&&o.mode!=c::Mode::Faulted,"asynchronous startup waits for brackets");
   f[0].source_time_s=1.011;f[0].sequence=2.;
   check(s.update(f,plan(1.012),1.012,.005,model).refresh,"asynchronous startup begins after bracket");
  }

  {c::ModelState st{};st.A={1,0,0,1};c::Pair out{};
   check(ph_exo::experiment::select_wire({.1,-.2},st,{.01,0,0,.01},out,{.1,.2}),"cap selection");
   check(std::abs(out[0])<=.1&&std::abs(out[1])<=.2,"quantized value inside individual caps");
   check(!ph_exo::experiment::select_wire({0,0},st,{.01,0,0,.01},out,{.001,.001}),"unrepresentable cap rejected");
  }
  std::cout<<checks<<" checks, "<<errors<<" failures\n";return errors?1:0;
}
