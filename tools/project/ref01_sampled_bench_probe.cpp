#include "ref01_model.hpp"
#include "ref01_bench_predictor.hpp"
#include "mech_bringup/ak30_kinematic_lease.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
namespace core=mech::mech_controllers::ref01;
namespace wire=mech::mech_protocol_cubemars;
int main(int argc,char** argv) {
 if(argc!=4)return 2;
 try {
  ph_exo::Ref01Model model(argv[1]);
  core::Plan plan{true,true,1.,1.,1.,{std::stod(argv[2]),std::stod(argv[3])},{10.,10.},{1.,1.},{5.,5.}};
  core::Session session; session.activate(1.);
  core::BenchPredictor predictor;core::Pair applied{};bool advancing=false;
  std::array<mech::mech_bringup::KinematicCommandLease,2> leases;
  std::array<mech::mech_bringup::KinematicBenchEnvelope,2> envelopes;
  double generation=0.; bool envelopes_initialized=false;
  std::cout<<std::setprecision(17);
  std::string line;unsigned tick=0;double tick_time=1.;std::array<double,4> mass{};
  while(std::getline(std::cin,line)){
   std::istringstream in(line);int p[2],v[2];double sample_time,sequence,plan_source;int enable;
   if(!(in>>p[0]>>p[1]>>v[0]>>v[1]>>sample_time>>sequence>>plan_source>>enable))return 3;
   tick_time=1.+(++tick)*.005;plan.issued_at=plan_source;plan.enabled=enable;
   core::Feedback feedback{};
   for(int i=0;i<2;++i){
    if(p[i]<-32768||p[i]>32767||v[i]<-32768||v[i]>32767)return 4;
    const auto a=static_cast<std::uint16_t>(p[i]),b=static_cast<std::uint16_t>(v[i]);
    wire::ForceControlPayload bytes{{static_cast<std::uint8_t>(a>>8),static_cast<std::uint8_t>(a),
     static_cast<std::uint8_t>(b>>8),static_cast<std::uint8_t>(b),0,0,40,0}};
    wire::ForceControlFeedback f;wire::decode_feedback(bytes,f);
    feedback[i]={f.position_deg*core::pi/180.,f.electrical_speed_erpm*2*core::pi/(60*21*8),sample_time,sequence};
   }
   if(!envelopes_initialized) {
    for(int i=0;i<2;++i) {
     leases[i].reset(0.,5.);
     envelopes[i].reset(feedback[i].position*180./core::pi,
       mech::mech_bringup::KinematicBenchEnvelope::EffortPolicy::Direct);
    }
    envelopes_initialized=true;
   }
   auto observation=session.update(feedback,plan,tick_time,.005,
    [&](const core::Pair& alpha,const core::Pair& velocity,core::ModelState& s){
     ph_exo::ModelState n;if(!model.evaluate(alpha,velocity,n))return false;
     s={n.q,n.dq,n.gravity,n.A,n.residual,n.iterations};mass=n.mass;return true;});
   core::Torque torque;
   if(observation.mode==core::Mode::Following){
    if(advancing && !predictor.advance(applied,.005))session.reject(core::Reason::Model);
    // This verifier is synchronous in model time: it does not hide delivery delay.
    if(observation.refresh && (std::abs(tick_time-sample_time)>1e-9 ||
       !predictor.observe(observation.state,mass,sample_time)))session.reject(core::Reason::Model);
    if(session.observation().mode==core::Mode::Following && !predictor.command(plan,torque))
     session.reject(core::Reason::InvalidTorque);
   }
   const bool send=session.observation().mode==core::Mode::Following;
   for(double x:predictor.state().q)std::cout<<x<<' ';
   for(double x:predictor.state().dq)std::cout<<x<<' ';
   if(send)++generation;
   unsigned sent_mask=0;
   for(int i=0;i<2;++i){
    const auto ns=static_cast<std::int64_t>(std::llround(tick_time*1e9));
    const bool leased=leases[i].update(send?torque.motor_nm[i]:0.,generation,
      feedback[i].source_time_s,true,ns);
    const bool write=envelopes[i].update(leased,!send||leases[i].faulted(),leases[i].effort(),
      feedback[i].position*180./core::pi,feedback[i].velocity*60.*21.*8./(2*core::pi),0.,40.,ns,.005);
    if(write)sent_mask|=1U<<i;
    wire::ForceControlCommand cmd;cmd.torque_nm=write?envelopes[i].effort():0.;
    wire::ForceControlPayload bytes;if(!wire::encode_force_control(cmd,wire::ake60_8_ranges(),bytes))return 7;
    auto code=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
    applied[i]=wire::dequantize(code,-15.,15.,12);
    std::cout<<cmd.torque_nm<<' '<<applied[i]<<' '<<code<<' ';
   }
   advancing=send;
   std::cout<<static_cast<int>(session.observation().mode)<<' '<<core::reason_name(session.observation().reason)<<' '
    <<predictor.source()<<' '<<predictor.external()[0]<<' '<<predictor.external()[1]<<' '<<sent_mask<<'\n'<<std::flush;
  }
 }catch(const std::exception& e){std::cerr<<e.what();return 8;}
}
