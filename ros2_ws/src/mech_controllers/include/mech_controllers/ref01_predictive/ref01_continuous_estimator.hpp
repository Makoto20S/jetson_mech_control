
#pragma once
#include "../ref01_impedance_core.hpp"
namespace ph_exo::experiment {
namespace c=mech::mech_controllers::ref01;
// Experimental reciprocal diagonal inertia fit using original per-motor
// velocity increments. No interpolated velocity or simulated plant parameters.
class ContinuousEstimator {
 public:
  void initialize(const c::ModelState& state,const std::array<double,4>& mass) {
    *this=ContinuousEstimator{};
    a_=state.A;gravity_=state.gravity;nominal_=mass;estimate_={mass[0],mass[3]};
    inverse_={1./mass[0],1./mass[3]};
    covariance_={1e-6,0.,0.,1e-6};
    rhs_={1e-6*inverse_[0],1e-6*inverse_[1]};ready_=true;
  }
  template<class Commands>
  bool sample(unsigned motor,double source,double velocity,const Commands& commands) {
    if(!ready_||motor>1||!std::isfinite(source)||!std::isfinite(velocity))return false;
    if(source<=source_[motor]) return true;
    if(source_[motor]<0) {source_[motor]=source;velocity_[motor]=velocity;return true;}
    const double begin=source_[motor];const double dt=source-begin;
    if(dt<=0.||dt>=.060)return false;
    c::Pair impulse{};
    if(!commands(1,begin,source,impulse[0]) || !commands(0,begin,source,impulse[1]))return false;
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
            const double target=std::clamp(next[i],1./(30.*nominal_[3*i]),1./(.5*nominal_[3*i]));
            // A constant physical inertia cannot jump thirty-fold on one
            // noisy acceleration interval. Bound parameter adaptation, not torque.
            const double ratio=std::clamp(target/inverse_[i],
              1./1.5,1.5);
            inverse_[i]*=ratio;
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
  const c::Pair& external() const {return external_;}
  const c::Pair& mass() const { return estimate_; }
  bool ready() const {return ready_;}
 private:
  std::array<double,4> a_{},nominal_{},covariance_{};
  c::Pair gravity_{},external_{};
  c::Pair estimate_{},inverse_{},rhs_{},velocity_{},acceleration_{};
  std::array<c::Pair,2> force_{};
  std::array<double,2> source_{{-1,-1}};
  std::array<bool,2> have_{};
  bool ready_{false};unsigned updates_{};
};
}
