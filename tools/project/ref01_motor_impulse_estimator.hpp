
#pragma once
#include "ref01_impedance_core.hpp"
namespace ph_exo::experiment {
namespace c=mech::mech_controllers::ref01;
// Experimental reciprocal diagonal inertia fit using original per-motor
// velocity increments. No interpolated velocity or simulated plant parameters.
class MotorImpulseEstimator {
 public:
  void initialize(const c::ModelState& state,const std::array<double,4>& mass) {
    a_=state.A;nominal_=mass;estimate_={mass[0],mass[3]};
    inverse_={1./mass[0],1./mass[3]};
    covariance_={1e-6,0.,0.,1e-6};
    rhs_={1e-6*inverse_[0],1e-6*inverse_[1]};ready_=true;
  }
  template<class Commands>
  bool sample(unsigned motor,int source,double velocity,const Commands& commands) {
    if(!ready_) return false;
    if(source<=source_[motor]) return true;
    if(source_[motor]<0) {source_[motor]=source;velocity_[motor]=velocity;return true;}
    const int begin=source_[motor];const double dt=(source-begin)*.005;
    c::Pair impulse{};
    for(int k=begin;k<source;++k) {
      impulse[0]+=.005*commands[k%64][1];impulse[1]+=.005*commands[k%64][0];
    }
    c::Pair force{(a_[0]*impulse[0]+a_[2]*impulse[1])/dt,
                  (a_[1]*impulse[0]+a_[3]*impulse[1])/dt};
    const double acceleration=(velocity-velocity_[motor])/dt;
    if(have_[motor]) {
      const unsigned row=2*(1-motor);
      c::Pair x{a_[row]*(force[0]-force_[motor][0]),a_[row+1]*(force[1]-force_[motor][1])};
      double y=acceleration-acceleration_[motor];
      const double magnitude=x[0]*x[0]+x[1]*x[1];
      // Ignore unexcited intervals; velocity quantization is retained in y.
      if(magnitude>1e-6
#ifdef PH_ASYNC_STRONG_ONLY
         && magnitude>1e-4 && std::abs(y)>1.
#endif
         && y*(x[0]*inverse_[0]+x[1]*inverse_[1])>=0.) {
        for(unsigned i=0;i<2;++i) {
          rhs_[i]=.98*rhs_[i]+x[i]*y;
          for(unsigned j=0;j<2;++j) covariance_[2*i+j]=.98*covariance_[2*i+j]+x[i]*x[j];
        }
        const double det=covariance_[0]*covariance_[3]-covariance_[1]*covariance_[2];
        if(det>1e-20) {
          c::Pair next{(covariance_[3]*rhs_[0]-covariance_[1]*rhs_[1])/det,
                       (covariance_[0]*rhs_[1]-covariance_[2]*rhs_[0])/det};
          for(unsigned i=0;i<2;++i) {
            const double target=std::clamp(next[i],1./(30.*nominal_[3*i]),2./nominal_[3*i]);
#ifdef PH_ASYNC_MASS_RATIO
            // A constant physical inertia cannot jump thirty-fold on one
            // noisy acceleration interval. Bound parameter adaptation, not torque.
            const double ratio=std::clamp(target/inverse_[i],
              1./PH_ASYNC_MASS_RATIO,PH_ASYNC_MASS_RATIO);
            inverse_[i]*=ratio;
#else
            inverse_[i]=target;
#endif
            estimate_[i]=1./inverse_[i];
          }
          ++updates_;
        }
      }
    }
    source_[motor]=source;velocity_[motor]=velocity;force_[motor]=force;
    acceleration_[motor]=acceleration;have_[motor]=true;
    return true;
  }
  const c::Pair& mass() const { return estimate_; }
  bool ready() const {return ready_;}
 private:
  std::array<double,4> a_{},nominal_{},covariance_{};
  c::Pair estimate_{},inverse_{},rhs_{},velocity_{},acceleration_{};
  std::array<c::Pair,2> force_{};
  std::array<int,2> source_{{-1,-1}};
  std::array<bool,2> have_{};
  bool ready_{false};unsigned updates_{};
};
}
