// Portable production Session + model + CubeMars codec. No device or ROS I/O.
#include "ref01_model.hpp"
#include "ref01_impedance_core.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
namespace core=mech::mech_controllers::ref01;
namespace wire=mech::mech_protocol_cubemars;
#ifdef PH_EXO_OFFLINE_GAIN_SWEEP
constexpr bool offline_sweep = true;
#else
constexpr bool offline_sweep = false;
#endif
int main(int argc,char** argv) {
  if(argc!=3 && argc!=5 && argc!=6 && argc!=8 && argc!=9)return 2;
  try {
    const double scale=std::stod(argv[2]);
    if(!std::isfinite(scale)||scale<=0.)return 3;
    ph_exo::Ref01Model model(argv[1]);
    const core::Pair stiffness=argc>=5?core::Pair{std::stod(argv[3]),std::stod(argv[4])}:core::Pair{2.4,2.};
    for(double k:stiffness)if(!std::isfinite(k)||k<=0.||k>(offline_sweep?60.:30.))return 9;
    const double cap=argc>=6?std::stod(argv[5]):.2;
    if(!std::isfinite(cap)||cap<=0.||cap>10.)return 10;
    const core::Pair damping=argc>=8?core::Pair{std::stod(argv[6]),std::stod(argv[7])}:core::Pair{.18,.15};
    for(double b:damping)if(!std::isfinite(b)||b<0.||b>(offline_sweep?2.:1.))return 11;
    core::Session session;
    if (argc == 9) {
      const double ramp = std::stod(argv[8]);
      if (ramp != 0. && ramp != .1 && ramp != .2) return 12;
      session = core::Session(ramp);
    }
    session.activate(1.);
    core::Plan plan;
    double last_seq=0.,last_source=0.;
    std::cout<<std::setprecision(17);
    std::string line;
    unsigned tick=0;
    while(std::getline(std::cin,line)) {
      std::istringstream in(line);
      double t,seq,source;int p[2],v[2],send;core::Pair q,dq;
      if(!(in>>t>>p[0]>>p[1]>>v[0]>>v[1]>>send>>seq>>source>>q[0]>>q[1]>>dq[0]>>dq[1]))return 4;
      const double now=1.005+t;
      if(send) {
        std::array<double,12> frame{1.,seq,source,1.,q[0],q[1],stiffness[0],stiffness[1],damping[0],damping[1],cap,cap};
        core::Plan next;
        if(core::valid_plan<offline_sweep>(frame,now,1.,last_seq,last_source,next,cap,true,true,dq)) {
          plan=next;last_seq=seq;last_source=source;
        }
      }
      core::Feedback fb{};
      for(unsigned i=0;i<2;++i) {
        if(p[i]<-32768||p[i]>32767||v[i]<-32768||v[i]>32767)return 5;
        const auto a=static_cast<std::uint16_t>(p[i]),b=static_cast<std::uint16_t>(v[i]);
        wire::ForceControlPayload bytes{{static_cast<std::uint8_t>(a>>8),static_cast<std::uint8_t>(a),
          static_cast<std::uint8_t>(b>>8),static_cast<std::uint8_t>(b),0,0,40,0}};
        wire::ForceControlFeedback decoded;
        wire::decode_feedback(bytes,decoded);
        fb[i]={decoded.position_deg*core::pi/180.,decoded.electrical_speed_erpm*scale,
               now-.001,static_cast<double>(tick+1)};
      }
      const auto& o=session.update(fb,plan,now,.005,
        [&model](const core::Pair& a,const core::Pair& v,core::ModelState& result) noexcept {
          ph_exo::ModelState n;
          if(!model.evaluate(a,v,n))return false;
          result={n.q,n.dq,n.gravity,n.A,n.residual,n.iterations};return true;
        });
      std::cout<<static_cast<int>(o.mode)<<' '<<static_cast<int>(o.reason)<<' '
               <<o.refresh<<' '<<o.generation;
      for(double x:o.state.q)std::cout<<' '<<x;
      for(double x:o.state.dq)std::cout<<' '<<x;
      for(unsigned i=0;i<2;++i) {
        double actual=0.;unsigned code=0;
        if(o.refresh) {
          wire::ForceControlCommand cmd;cmd.torque_nm=o.torque.motor_nm[i];
          wire::ForceControlPayload bytes;
          if(!wire::encode_force_control(cmd,wire::ake60_8_ranges(),bytes))return 7;
          code=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
          actual=wire::dequantize(code,-15.,15.,12);
        }
        std::cout<<' '<<o.torque.motor_nm[i]<<' '<<actual<<' '<<code;
      }
      std::cout<<' '<<o.torque.scale<<'\n'<<std::flush;++tick;
    }
  }catch(const std::exception& e){std::cerr<<e.what();return 8;}
}
