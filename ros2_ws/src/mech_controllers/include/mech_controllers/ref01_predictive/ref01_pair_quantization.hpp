#pragma once
#include "../ref01_impedance_core.hpp"
#include <limits>
namespace ph_exo::experiment {
namespace c=mech::mech_controllers::ref01;
// Select adjacent representable motor torques jointly. The vendor codec is
// unchanged; minimizing generalized acceleration error prevents simultaneous
// identical code jumps from doubling common-axis quantization ripple.
inline bool select_wire(const c::Pair& desired,const c::ModelState& state,
                    const std::array<double,4>& mass,c::Pair& result,const c::Pair& caps={5.,5.}) {
 result={};
 for(double cap:caps)if(!std::isfinite(cap)||cap<=0.||cap>10.)return false;
 for(double v:desired) if(!std::isfinite(v) || std::abs(v)>10.+1e-12) return false;
 for(double v:mass) if(!std::isfinite(v)) return false;
 for(double v:state.A) if(!std::isfinite(v)) return false;
 if(mass[0]<=0. || mass[3]<=0. ||
    std::abs(mass[1]-mass[2])>1e-10 ||
    mass[0]*mass[3]-mass[1]*mass[2]<=0.) return false;
 constexpr double step=30./4095.;
 const int lower0=static_cast<int>(std::floor((desired[0]+15.)/step));
 const int lower1=static_cast<int>(std::floor((desired[1]+15.)/step));
 const double det=mass[0]*mass[3]-mass[1]*mass[2];
 c::Pair best{};double best_cost=std::numeric_limits<double>::infinity();
 for(int b0=0;b0<2;++b0) for(int b1=0;b1<2;++b1) {
  c::Pair wire{-15.+(lower0+b0)*step,-15.+(lower1+b1)*step};
  if(std::abs(wire[0])>caps[0]+1e-12 || std::abs(wire[1])>caps[1]+1e-12) continue;
  const c::Pair error{wire[1]-desired[1],wire[0]-desired[0]};
  const auto& a=state.A;
  const c::Pair force{a[0]*error[0]+a[2]*error[1],a[1]*error[0]+a[3]*error[1]};
  const c::Pair accel{(mass[3]*force[0]-mass[1]*force[1])/det,
                      (mass[0]*force[1]-mass[2]*force[0])/det};
  const double cost=accel[0]*accel[0]+accel[1]*accel[1];
  if(cost<best_cost) {best_cost=cost;best=wire;}
 }
 result=best;
 return std::isfinite(best_cost);
}
}
