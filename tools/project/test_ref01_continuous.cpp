
#include "ref01_continuous_engine.hpp"
#include <iostream>
#include <limits>
#include <cstdlib>
namespace c=mech::mech_controllers::ref01;
using Engine=ph_exo::control::AsynchronousPredictor;
int checks=0;
void check(bool v){++checks;if(!v){std::cerr<<"FAIL "<<checks<<"\n";std::exit(1);}}
struct Fixture {
 Engine e;c::Pair selected{};c::Torque torque{};
 c::Plan plan{true,true,1,1,1,{0,0},{10,10},{1,1},{5,5}};
 std::array<Engine::Alignment::Sample,2> samples{{{1.,c::pi/4.,0},{1.,c::pi/4.,0}}};
 Fixture(){check(e.reset(1.));}
 bool update(double now) {
  return e.update(now,samples,plan,[](const c::Pair&a,const c::Pair&v,c::ModelState&s,std::array<double,4>&m){
   s={a,v,{0,0},{1,0,0,1},0,1};m={.01,0,0,.01};return true;
  },selected,torque);
 }
};
int main() {
 {Fixture f;check(f.update(1.));f.samples[0].source=1.025;f.samples[1].source=1.032;
  check(f.e.receipt(0,{1.0023,.001,1}));check(f.e.receipt(1,{1.0037,-.001,1}));
  check(f.update(1.033));check(std::abs(f.e.aligned().source-1.025)<1e-12);}
 {Fixture f;check(f.update(1.));check(!f.update(1.061));check(f.selected==c::Pair{});
  f.samples[0].source=1.062;f.samples[1].source=1.062;
  check(!f.update(1.062));check(!f.e.receipt(0,{1.062,0,1}));}
 {Fixture f;check(f.update(1.));f.samples[0].position+=.001;check(!f.update(1.005));check(f.torque.motor_nm==c::Pair{});}
 {Fixture f;check(f.update(1.));f.samples[0].source=std::numeric_limits<double>::quiet_NaN();check(!f.update(1.005));}
 {Fixture f;check(f.update(1.));check(!f.update(std::numeric_limits<double>::quiet_NaN()));}
 {Fixture f;check(f.update(1.));check(f.e.receipt(0,{1.004,0,1}));check(!f.e.receipt(0,{1.006,0,3}));check(!f.update(1.007));}
 {Fixture f;check(f.update(1.));check(f.e.receipt(0,{1.02,0,1}));check(!f.update(1.005));}
 {Fixture f;check(f.update(1.));check(!f.e.receipt(0,{1.005,std::numeric_limits<double>::infinity(),1}));check(!f.update(1.005));}
 {Fixture f;f.plan.enabled=false;check(!f.update(1.));f.plan.enabled=true;check(!f.update(1.005));}
 {Fixture f;check(f.update(1.));f.samples[0].source=1.1;check(!f.update(1.005));}
 std::cout<<checks<<" continuous fault checks passed\n";
}
