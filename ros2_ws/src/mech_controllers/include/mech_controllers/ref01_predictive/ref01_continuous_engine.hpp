
#pragma once
#include "ref01_continuous_predictor.hpp"
#include "ref01_continuous_alignment.hpp"
#include "ref01_continuous_estimator.hpp"
#include "ref01_continuous_history.hpp"
#include "ref01_pair_quantization.hpp"
namespace ph_exo::control {
namespace c=mech::mech_controllers::ref01;
// Experiment adapter: exact monotonic timestamps and accepted-effort history.
// The caller owns plan/lifecycle and sensor sequence validation.
class AsynchronousPredictor {
 public:
  using Pair=c::Pair;
  using Alignment=ph_exo::experiment::ContinuousAlignment;
  using Predictor=c::ContinuousPredictor;
  bool set_receive_time_bound(double value) noexcept {
    if(anchor_time_>0. || !std::isfinite(value) || (value!=0. && value!=.007))return false;
    receive_time_bound_s_=value;return true;
  }
  bool waiting_for_initial_pair() const noexcept {return waiting_;}
  bool reset(double epoch) noexcept {
    history_=EffortHistory{};history_.reset(epoch);
    alignment_={};estimator_={};anchor_.reset();current_.reset();
    waiting_=false;anchor_time_=0.;failed_=false;reason_="ok";raw_samples_={};timed_samples_={};delay_={};anchor_measured_q_={};anchor_motor_position_={};
    return std::isfinite(epoch)&&epoch>0.;
  }
  bool receipt(unsigned axis,const EffortHistory::Sample& sample) noexcept {
    if(failed_)return false;
    if(!history_.push(axis,sample))return fail("receipt_invalid_or_missing");
    return true;
  }
  template<class Evaluate>
  bool update(double now,const std::array<Alignment::Sample,2>& received,
              const c::Plan& plan,Evaluate&& evaluate,Pair& selected,c::Torque& torque) noexcept {
    selected={};torque={};if(failed_)return false;
    if(!std::isfinite(now)||history_.newest_time()>now)return fail("invalid_time");
    for(const auto& packet:received)
      if(!std::isfinite(packet.source)||packet.source<=0.||packet.source>now ||
         !std::isfinite(packet.position)||!std::isfinite(packet.velocity))return fail("invalid_axis_packet");
    auto samples=received;
    if(receive_time_bound_s_>0.) {
    for(unsigned i=0;i<2;++i) {
      if(received[i].source==raw_samples_[i].source) {
        if(received[i].position!=raw_samples_[i].position || received[i].velocity!=raw_samples_[i].velocity)
          return fail("receive_mutation");
        samples[i]=timed_samples_[i];
      } else if(received[i].source>raw_samples_[i].source) {
        auto packet=received[i];
        if(anchor_time_>0.) {
          double best=std::numeric_limits<double>::infinity(),best_source=packet.source;
          for(double lag=0.;lag<=receive_time_bound_s_+1e-12;lag+=.0005) {
            const double candidate=packet.source-lag;
            if(candidate<anchor_time_ || candidate<=timed_samples_[i].source)continue;
            auto predicted=anchor_;
            if(!history_.replay(predicted,anchor_time_,candidate))continue;
            const auto& state=predicted.state();const unsigned row=2*(1-i);
            const double velocity=state.A[row]*state.dq[0]+state.A[row+1]*state.dq[1];
            const double position=anchor_motor_position_[i]+
                state.A[row]*(state.q[0]-anchor_measured_q_[0])+
                state.A[row+1]*(state.q[1]-anchor_measured_q_[1]);
            auto excess=[](double error,double quantum){return std::max(0.,std::abs(error)/quantum-.5);};
            const double v=excess(velocity-packet.velocity,10.*2.*c::pi/(60.*21.*8.));
            const double q=excess(position-packet.position,.1*c::pi/180.);
            const double penalty=(lag-delay_[i])/receive_time_bound_s_;
            const double cost=v*v+q*q+.01*penalty*penalty;
            if(cost<best){best=cost;best_source=candidate;}
          }
          delay_[i]=packet.source-best_source;packet.source=best_source;
        }
        samples[i]=packet;timed_samples_[i]=packet;raw_samples_[i]=received[i];
      } else return fail("receive_rollback");
    }
    }
    for(unsigned i=0;i<2;++i)
      if(!alignment_.update(i,samples[i],now))return fail("invalid_axis_packet");
    if(!alignment_.align(now,aligned_)) {
      const bool fresh=now-samples[0].source<.060-1e-12 && now-samples[1].source<.060-1e-12;
      if(anchor_time_==0. && fresh){waiting_=true;reason_="waiting_for_initial_pair";return false;}
      return fail("alignment_missing_or_expired");
    }
    waiting_=false;reason_="ok";
    auto impulse=[this](unsigned axis,double begin,double end,double& value) {
      return history_.integral(axis,begin,end,value);
    };
    if(estimator_.ready())for(unsigned i=0;i<2;++i)
      if(!estimator_.sample(i,samples[i].source,samples[i].velocity,impulse))
        return fail("identification_history_missing");
    const double source=aligned_.source;
    if(source<anchor_time_||source>now||now-source>=.060-1e-12)
      return fail("source_invalid_or_expired");
    if(source>anchor_time_) {
      auto corrected=anchor_;
      if(anchor_time_>0.) {
        if(!history_.replay(corrected,anchor_time_,source))return fail("correction_history_missing");
        const auto& st=corrected.state();auto mass=corrected.estimated_mass();
        mass[0]=estimator_.mass()[0];mass[3]=estimator_.mass()[1];
        const double det=mass[0]*mass[3]-mass[1]*mass[2];
        if(!std::isfinite(det)||det<=0.)return fail("invalid_inertia");
        for(unsigned i=0;i<2;++i) {
          const double left=aligned_.left[i].source,right=aligned_.right[i].source;
          if(right==left)continue;
          Pair motor_impulse{};const double weight=(source-left)/(right-left);
          for(unsigned j=0;j<2;++j) {
            double partial{},whole{};
            if(!impulse(j,left,source,partial)||!impulse(j,left,right,whole))
              return fail("alignment_history_missing");
            motor_impulse[1-j]=partial-weight*whole;
          }
          Pair force{st.A[0]*motor_impulse[0]+st.A[2]*motor_impulse[1],
                     st.A[1]*motor_impulse[0]+st.A[3]*motor_impulse[1]};
          Pair dv{(mass[3]*force[0]-mass[1]*force[1])/det,
                  (mass[0]*force[1]-mass[2]*force[0])/det};
          const unsigned row=2*(1-i);
          aligned_.velocity[i]+=st.A[row]*dv[0]+st.A[row+1]*dv[1];
        }
      }
      Pair alpha{aligned_.position[1]-c::pi/4.,aligned_.position[0]-c::pi/4.};
      Pair velocity{aligned_.velocity[1],aligned_.velocity[0]};
      c::ModelState measured{};std::array<double,4> mass{};
      if(!evaluate(alpha,velocity,measured,mass))return fail("model_domain");
      if(!estimator_.ready()) {
        estimator_.initialize(measured,mass);
        for(unsigned i=0;i<2;++i)
          if(!estimator_.sample(i,samples[i].source,samples[i].velocity,impulse))
            return fail("initial_identification");
      }
      corrected.set_identified_mass(estimator_.mass());
      if(!corrected.observe(measured,mass,source))return fail("correction_rejected");
      anchor_=corrected;anchor_time_=source;anchor_measured_q_=measured.q;anchor_motor_position_=aligned_.position;
    }
    current_=anchor_;
    if(!history_.replay(current_,anchor_time_,now))return fail("replay_domain");
    auto preview=current_;Pair held{};
    for(unsigned i=0;i<2;++i)if(!history_.held(i,now,held[i]))return fail("receipt_missing");
    if(!preview.advance(held,.005))return fail("send_horizon_domain");
    if(!preview.command(plan,torque))return fail("command_rejected");
    auto wire_demand=torque.motor_nm;
    if(!ph_exo::experiment::select_wire(wire_demand,preview.state(),preview.estimated_mass(),selected,{plan.caps[1],plan.caps[0]}))
      return fail("wire_selection_rejected");
    return true;
  }
  const Predictor& current() const noexcept{return current_;}
  const Alignment::Pair& aligned() const noexcept{return aligned_;}
  const char* reason() const noexcept{return reason_;}
 private:
  bool fail(const char* reason) noexcept {failed_=true;reason_=reason;return false;}
  EffortHistory history_{};
  Alignment alignment_{};
  ph_exo::experiment::ContinuousEstimator estimator_{};
  Predictor anchor_{},current_{};
  Alignment::Pair aligned_{};
  std::array<Alignment::Sample,2> raw_samples_{},timed_samples_{};
  double receive_time_bound_s_{};bool waiting_{};
  Pair delay_{},anchor_measured_q_{},anchor_motor_position_{};
  double anchor_time_{};
  bool failed_{};
  const char* reason_{"ok"};
};
}
