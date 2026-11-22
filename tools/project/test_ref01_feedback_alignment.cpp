
#include "ref01_feedback_alignment.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using F=ph_exo::experiment::FeedbackAlignment;
int checks=0;
void check(bool v,const char* msg){++checks;if(!v)throw std::runtime_error(msg);}
int main(){
 try{
  F f;F::Pair p;
  check(!f.align(0,p),"missing channels");
  check(f.update(0,{0,10,2},0)&&f.update(1,{0,20,4},0),"seed");
  check(f.align(0,p)&&p.source==0&&p.position[0]==10&&p.position[1]==20,"synchronous exact");
  check(f.update(0,{4,18,6},4),"first axis sample");
  check(f.update(1,{2,24,8},4),"second axis phase sample");
  check(f.align(4,p)&&p.source==2,"common PAST source");
  check(p.position[0]==14&&p.velocity[0]==4&&p.position[1]==24,"linear interpolation");
  check(p.bracket[0]==4&&p.bracket[1]==0,"bracket evidence");
  check(p.age[0]==0&&p.age[1]==2,"separate original ages retained");
  check(f.update(0,{1,999,999},5)&&f.discarded()==1,"late old sample discarded");
  check(f.align(5,p)&&p.source==2&&p.position[0]==14,"discard does not refresh data");
  check(f.align(13,p),"55ms oldest source valid");
  check(!f.align(14,p),"60ms source expires");
  check(!f.update(0,{8,1,1},7),"future source invalid");
  check(!f.update(0,{4,99,6},7),"duplicate source mutation rejected");
  check(!f.update(2,{1,1,1},7),"bad axis invalid");
  check(!f.update(0,{5,std::numeric_limits<double>::quiet_NaN(),0},7),"nonfinite invalid");
  F missing;check(missing.update(0,{4,18,6},4)&&missing.update(1,{2,24,8},4),"seed without older bracket");
  check(!missing.align(4,p),"no extrapolation");
  F sync;
  for(int t=0;t<80;t+=4){
   check(sync.update(0,{t,double(t),double(2*t)},t)&&sync.update(1,{t,double(3*t),double(4*t)},t),"sync feed");
   check(sync.align(t,p)&&p.source==t&&p.position[0]==t&&p.position[1]==3*t,"sync identity");
  }
  std::cout<<"alignment contract: "<<checks<<" assertions passed\n";
 }catch(const std::exception& e){std::cerr<<e.what()<<"\n";return 1;}
}
