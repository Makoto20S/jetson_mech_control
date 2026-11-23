#pragma once
#include "ref01_continuous_engine.hpp"
namespace ph_exo::control {
// Experimental integration seam: existing lifecycle/plan/feedback validation,
// plus continuous prediction. It never owns a transport or enables a drive.
class PredictiveSession {
 public:
  using Receipt=EffortHistory::Sample;
  bool set_receive_time_bound(double value) noexcept{return engine_.set_receive_time_bound(value);}
  void activate(double epoch) noexcept {
    guard_.activate(epoch);engine_.reset(epoch);output_={};
    output_.mode=c::Mode::Waiting;output_.reason=c::Reason::None;
  }
  void deactivate() noexcept {guard_.deactivate();output_={};}
  const c::Observation& observation() const noexcept{return output_;}
  const char* predictor_reason() const noexcept{return engine_.reason();}
  bool receipt(unsigned axis,const Receipt& receipt) noexcept {
    if(output_.mode==c::Mode::Inactive || output_.mode==c::Mode::Faulted)return false;
    if(!engine_.receipt(axis,receipt)){reject(c::Reason::InvalidTorque);return false;}
    return true;
  }
  const c::Observation& reject(c::Reason reason) noexcept {
    if(output_.mode==c::Mode::Inactive || output_.mode==c::Mode::Faulted)return output_;
    guard_.reject(reason);
    if(output_.refresh && output_.generation>0)--output_.generation;
    output_.mode=c::Mode::Faulted;output_.reason=reason;output_.refresh=false;output_.torque={};
    return output_;
  }
  template<class Evaluate>
  const c::Observation& update(const c::Feedback& feedback,const c::Plan& plan,
      double now,double period,Evaluate&& evaluate) noexcept {
    output_.refresh=false;
    if(output_.mode==c::Mode::Inactive || output_.mode==c::Mode::Faulted)return output_;
    const auto& gate=guard_.update(feedback,plan,now,period,
      [&evaluate](const c::Pair& alpha,const c::Pair& velocity,c::ModelState& state) {
        std::array<double,4> mass{};return evaluate(alpha,velocity,state,mass);
      });
    if(gate.mode==c::Mode::Faulted)return reject(gate.reason);
    if(gate.mode!=c::Mode::Following)return output_;
    std::array<AsynchronousPredictor::Alignment::Sample,2> samples{};
    for(unsigned i=0;i<2;++i)samples[i]={feedback[i].source_time_s,feedback[i].position,feedback[i].velocity};
    c::Pair selected{};c::Torque torque{};
    if(!engine_.update(now,samples,plan,evaluate,selected,torque)) {
      if(engine_.waiting_for_initial_pair()){output_.mode=c::Mode::Waiting;return output_;}
      return reject(c::Reason::Model);
    }
    output_.mode=c::Mode::Following;output_.reason=c::Reason::None;
    output_.state=engine_.current().state();
    // Encoded-grid target. Hardware conversion must use a proven interior
    // representative; never add a quarter quantum beyond the requested cap.
    torque.motor_nm=selected;torque.alpha_nm={selected[1],selected[0]};
    output_.torque=torque;output_.source_time_s=engine_.aligned().source;
    output_.refresh=true;++output_.generation;
    return output_;
  }
 private:
  c::Session guard_{};
  AsynchronousPredictor engine_{};
  c::Observation output_{};
};
}
