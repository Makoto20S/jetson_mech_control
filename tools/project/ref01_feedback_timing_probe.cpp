#include "ref01_model.hpp"
#include "ref01_adaptive_predictor.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <limits>
#include "ref01_pair_quantization.hpp"
#include "ref01_feedback_alignment.hpp"
namespace c=mech::mech_controllers::ref01;
namespace w=mech::mech_protocol_cubemars;
// Numerical experiment only; histories use model time, no device access.
int main(int argc,char** argv){
 if(argc!=6 || std::string(argv[2])!="predict")return 2;
 try{
  ph_exo::Ref01Model model(argv[1]);bool predictive=std::string(argv[2])=="predict";
  c::Plan plan{true,true,1,1,1,{std::stod(argv[3]),std::stod(argv[4])},{10,10},{std::stod(argv[5]),std::stod(argv[5])},{5,5}};
  c::AdaptiveBenchPredictor predictor;std::array<c::AdaptiveBenchPredictor,64> history;
  std::array<c::Pair,64> sent{};std::array<int,64> stamps;stamps.fill(-1);
  ph_exo::experiment::FeedbackAlignment aligner;
  ph_exo::experiment::FeedbackAlignment::Pair aligned;
  c::ModelState measured{};int last=-1;bool fault=false;std::string reason="ok",line;
  std::cout<<std::setprecision(17);
  std::cerr<<"# t,source,M00,M01,M10,M11,external0,external1,measured_dq0,measured_dq1,strong0,strong1,weak0,weak1\n";
  for(int tick=0;std::getline(std::cin,line);++tick){
   std::istringstream input(line);int source_axis[2],p[2],v[2];
   if(!(input>>source_axis[0]>>p[0]>>v[0]>>source_axis[1]>>p[1]>>v[1])) return 3;
   for(unsigned i=0;i<2 && !fault;++i){
     if(p[i]<-32768||p[i]>32767||v[i]<-32768||v[i]>32767)return 4;
     auto a=static_cast<std::uint16_t>(p[i]),b=static_cast<std::uint16_t>(v[i]);
     w::ForceControlPayload bytes{{static_cast<std::uint8_t>(a>>8),static_cast<std::uint8_t>(a),
       static_cast<std::uint8_t>(b>>8),static_cast<std::uint8_t>(b),0,0,40,0}};
     w::ForceControlFeedback f;w::decode_feedback(bytes,f);
     if(!aligner.update(i,{source_axis[i],f.position_deg,f.electrical_speed_erpm},tick)){
       fault=true;reason="invalid_axis_packet";
     }
   }
   if(!fault && !aligner.align(tick,aligned)){fault=true;reason="alignment_missing_or_expired";}
   const int source=aligned.source;
   if(!fault && (source<last||source>tick||tick-source>=12)){
     fault=true;reason="source_invalid_or_expired";
   }
   if(!fault && predictive && tick>0 && !predictor.advance(sent[(tick-1)%64],.005)){fault=true;reason="prediction_domain";}
   history[tick%64]=predictor;stamps[tick%64]=tick;
   if(!fault && source>last){
    c::Pair alpha{},velocity{};
    for(int i=0;i<2;++i){
     alpha[1-i]=aligned.position[i]*c::pi/180.-c::pi/4.;
     velocity[1-i]=aligned.velocity[i]*2*c::pi/(60*21*8);
    }
    ph_exo::ModelState native{};
    if(!model.evaluate(alpha,velocity,native)){fault=true;reason="model_domain";}
    else {
     measured={native.q,native.dq,native.gravity,native.A,native.residual,native.iterations};
     if(predictive){
      if(stamps[source%64]!=source){fault=true;reason="history_missing";}
      else {
       auto corrected=history[source%64];
       if(!corrected.observe(measured,native.mass,1.+source*.005)){fault=true;reason="correction_rejected";}
       else {
        history[source%64]=corrected;
        for(int j=source;!fault && j<tick;++j){
         if(!corrected.advance(sent[j%64],.005)){fault=true;reason="replay_domain";break;}
         history[(j+1)%64]=corrected;
        }
        predictor=corrected;
       }
      }
     }
    }
    last=source;
   }
   if(tick%4==0){
    std::cerr<<std::setprecision(17)<<tick*.005<<','<<source*.005;
    for(double v:predictor.estimated_mass()) std::cerr<<','<<v;
    for(double v:predictor.external()) std::cerr<<','<<v;
    for(double v:measured.dq) std::cerr<<','<<v;
    for(auto n:predictor.strong_updates()) std::cerr<<','<<n;
    for(auto n:predictor.weak_updates()) std::cerr<<','<<n;
    std::cerr<<'\n';
   }
   c::Torque torque{};
   if(!fault && !(predictive?predictor.command(plan,torque):c::fixed_pose_torque(measured,plan,torque))){fault=true;reason="command_rejected";}
   c::Pair selected{};
   if(!fault && !ph_exo::experiment::select_wire(torque.motor_nm,predictor.state(),
                                                predictor.estimated_mass(),selected)) {
     fault=true;reason="wire_selection_rejected";
   }
   for(int i=0;i<2;++i){
    w::ForceControlCommand command;command.torque_nm=fault?0.:torque.motor_nm[i];
    auto encoded_command=command;
    // Interior of selected bin avoids floating point roundoff at a code boundary.
    if(!fault) encoded_command.torque_nm=selected[i]+.25*30./4095.;
    w::ForceControlPayload bytes;if(!w::encode_force_control(encoded_command,w::ake60_8_ranges(),bytes))return 5;
    auto code=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
    sent[tick%64][i]=w::dequantize(code,-15,15,12);
    if(!fault && std::abs(sent[tick%64][i]-selected[i])>1e-10) return 7;
    std::cout<<command.torque_nm<<' '<<sent[tick%64][i]<<' ';
   }
   const auto& state=predictive?predictor.state():measured;
   std::cout<<state.q[0]<<' '<<state.q[1]<<' '<<state.dq[0]<<' '<<state.dq[1]<<' '<<fault<<' '<<reason<<' '<<source<<' '<<aligned.age[0]<<' '<<aligned.age[1]<<' '<<aligned.bracket[0]<<' '<<aligned.bracket[1]<<'\n'<<std::flush;
  }
 }catch(const std::exception& ex){std::cerr<<ex.what();return 6;}
}
