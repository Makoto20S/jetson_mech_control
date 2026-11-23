#include "ref01_model.hpp"
#include "ref01_impedance_core.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
namespace core=mech::mech_controllers::ref01;
namespace wire=mech::mech_protocol_cubemars;
int main(int argc,char** argv) {
  if (argc!=2 && argc!=4) return 2;
  try {
    ph_exo::Ref01Model model(argv[1]);
    core::Plan plan{true,true,1.,1.,1.,{0.,core::pi/9.},{2.4,2.},{.18,.15},{.2,.2}};
    if(argc==4) {
      plan.damping={std::stod(argv[2]),std::stod(argv[3])};
      if(plan.damping[0]<0 || plan.damping[0]>.18 || plan.damping[1]<0 || plan.damping[1]>.15) return 9;
    }
    std::cout<<std::setprecision(17);
    std::string line;
    while(std::getline(std::cin,line)) {
      std::istringstream in(line); int p[2],v[2];
      if (!(in>>p[0]>>p[1]>>v[0]>>v[1])) return 3;
      std::array<double,2> alpha{},velocity{};
      for (int i=0;i<2;++i) {
        if(p[i]<-32768||p[i]>32767||v[i]<-32768||v[i]>32767) return 4;
        const auto a=static_cast<std::uint16_t>(p[i]), b=static_cast<std::uint16_t>(v[i]);
        wire::ForceControlPayload bytes{{static_cast<std::uint8_t>(a>>8),static_cast<std::uint8_t>(a),
          static_cast<std::uint8_t>(b>>8),static_cast<std::uint8_t>(b),0,0,40,0}};
        wire::ForceControlFeedback feedback;
        wire::decode_feedback(bytes,feedback);
        alpha[1-i]=feedback.position_deg*core::pi/180.-core::pi/4.;
        velocity[1-i]=feedback.electrical_speed_erpm*2.*core::pi/(60.*21.*8.);
      }
      ph_exo::ModelState n; core::Torque torque;
      if(!model.evaluate(alpha,velocity,n)) return 5;
      core::ModelState state{n.q,n.dq,n.gravity,n.A,n.residual,n.iterations};
      if(!core::fixed_pose_torque(state,plan,torque)) return 6;
      for(double value:n.q) std::cout<<value<<' ';
      for(double value:n.dq) std::cout<<value<<' ';
      for(int i=0;i<2;++i) {
        wire::ForceControlCommand cmd;cmd.torque_nm=torque.alpha_nm[1-i];
        wire::ForceControlPayload bytes;
        if(!wire::encode_force_control(cmd,wire::ake60_8_ranges(),bytes)) return 7;
        std::cout<<torque.alpha_nm[1-i]<<' ';
        const auto code=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
        std::cout<<wire::dequantize(code,-15.,15.,12)<<' '<<code<<' ';
      }
      std::cout<<'\n'<<std::flush;
    }
  }catch(const std::exception& e){std::cerr<<e.what();return 8;}
}
