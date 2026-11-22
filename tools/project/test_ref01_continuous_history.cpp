
#include "ref01_continuous_history.hpp"
#include <iostream>
#include <limits>
#include <cstdlib>
int checks=0;
void check(bool value){++checks;if(!value){std::cerr<<"failed check "<<checks<<"\n";std::exit(1);}}
struct Model {
 std::array<double,2> impulse{};double elapsed{};unsigned calls{};
 bool advance(const std::array<double,2>& effort,double dt) {
  check(dt>0&&dt<=.005000001);++calls;elapsed+=dt;
  for(unsigned i=0;i<2;++i)impulse[i]+=effort[i]*dt;
  return true;
 }
};
int main() {
 using History=ph_exo::control::EffortHistory;History h;double value{};
 check(!h.held(0,1.,value));h.reset(100000.);
 check(h.push(0,{100000.0023,1.2,4}));
 check(h.push(1,{100000.0037,-.8,7}));
 check(h.push(0,{100000.0079,-.4,5}));
 check(h.push(1,{100000.0111,.6,8}));
 check(h.integral(0,100000.,100000.02,value));
 const double expected0=1.2*(100000.0079-100000.0023)-.4*(100000.02-100000.0079);
 check(std::abs(value-expected0)<1e-12);
 check(h.integral(1,100000.,100000.02,value));
 const double expected1=-.8*(100000.0111-100000.0037)+.6*(100000.02-100000.0111);
 check(std::abs(value-expected1)<1e-12);
 Model m;check(h.replay(m,100000.,100000.02));
 check(std::abs(m.impulse[0]-expected0)<1e-12);
 check(std::abs(m.impulse[1]-expected1)<1e-12);
 check(std::abs(m.elapsed-(100000.02-100000.))<1e-12);
 check(h.push(0,{100000.0079,-.4,5}));
 check(!h.push(0,{100000.0079,-.5,5}));
 check(!h.push(0,{100000.01,.4,7}));
 check(!h.push(0,{100000.006,.4,6}));
 check(!h.push(2,{100000.02,0,1}));
 check(!h.push(0,{100000.02,std::numeric_limits<double>::quiet_NaN(),6}));
 check(!h.integral(0,99999.,100000.,value));
 check(!h.integral(0,100000.02,100000.01,value));
 check(!h.replay(m,100000.,100001.));
 for(unsigned i=0;i<80;++i)check(h.push(0,{100000.02+.005*i,.1,6+i}));
 check(!h.held(0,100000.,value));
 check(!h.integral(0,100000.,100000.5,value));
 h.reset(100001.);
 check(h.held(0,100001.,value)&&value==0.);
 check(!h.held(1,100000.99,value));
 h.reset(std::numeric_limits<double>::quiet_NaN());
 check(!h.push(0,{100001.1,0,1}));
 std::cout<<checks<<" history checks passed\n";
}
