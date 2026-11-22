#include "ref01_model.hpp"
#include "ref01_bench_predictor.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
namespace c=mech::mech_controllers::ref01;
namespace w=mech::mech_protocol_cubemars;
// Numerical experiment only; histories use model time, no device access.
int main(int argc,char** argv){
 if(argc!=6)return 2;
 try{
  ph_exo::Ref01Model model(argv[1]);bool predictive=std::string(argv[2])=="predict";
  c::Plan plan{true,true,1,1,1,{std::stod(argv[3]),std::stod(argv[4])},{10,10},{std::stod(argv[5]),std::stod(argv[5])},{5,5}};
  c::BenchPredictor predictor;std::array<c::BenchPredictor,64> history;
  std::array<c::Pair,64> sent{};std::array<int,64> stamps;stamps.fill(-1);
  c::ModelState measured{};int last=-1;bool fault=false;std::string reason="ok",line;
  std::cout<<std::setprecision(17);
  for(int tick=0;std::getline(std::cin,line);++tick){
   std::istringstream input(line);int source,p[2],v[2];
   if(!(input>>source>>p[0]>>p[1]>>v[0]>>v[1]))return 3;
   if(!fault && (source<last||source>tick||tick-source>=12)){fault=true;reason="source_invalid_or_expired";}
   if(!fault && predictive && tick>0 && !predictor.advance(sent[(tick-1)%64],.005)){fault=true;reason="prediction_domain";}
   history[tick%64]=predictor;stamps[tick%64]=tick;
   if(!fault && source>last){
    c::Pair alpha{},velocity{};
    for(int i=0;i<2;++i){
     if(p[i]<-32768||p[i]>32767||v[i]<-32768||v[i]>32767)return 4;
     auto a=static_cast<std::uint16_t>(p[i]),b=static_cast<std::uint16_t>(v[i]);
     w::ForceControlPayload bytes{{static_cast<std::uint8_t>(a>>8),static_cast<std::uint8_t>(a),static_cast<std::uint8_t>(b>>8),static_cast<std::uint8_t>(b),0,0,40,0}};
     w::ForceControlFeedback f;w::decode_feedback(bytes,f);
     alpha[1-i]=f.position_deg*c::pi/180.-c::pi/4.;
     velocity[1-i]=f.electrical_speed_erpm*2*c::pi/(60*21*8);
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
   c::Torque torque{};
   if(!fault && !(predictive?predictor.command(plan,torque):c::fixed_pose_torque(measured,plan,torque))){fault=true;reason="command_rejected";}
   for(int i=0;i<2;++i){
    w::ForceControlCommand command;command.torque_nm=fault?0.:torque.motor_nm[i];
    w::ForceControlPayload bytes;if(!w::encode_force_control(command,w::ake60_8_ranges(),bytes))return 5;
    auto code=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
    sent[tick%64][i]=w::dequantize(code,-15,15,12);
    std::cout<<command.torque_nm<<' '<<sent[tick%64][i]<<' ';
   }
   const auto& state=predictive?predictor.state():measured;
   std::cout<<state.q[0]<<' '<<state.q[1]<<' '<<state.dq[0]<<' '<<state.dq[1]<<' '<<fault<<' '<<reason<<'\n'<<std::flush;
  }
 }catch(const std::exception& ex){std::cerr<<ex.what();return 6;}
}
