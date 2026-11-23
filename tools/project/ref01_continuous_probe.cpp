
#include "ref01_model.hpp"
#include "ref01_continuous_engine.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
namespace c=mech::mech_controllers::ref01;
namespace w=mech::mech_protocol_cubemars;
int main(int argc,char** argv) {
 if(argc!=6)return 2;
 try {
  ph_exo::Ref01Model model(argv[1]);ph_exo::control::AsynchronousPredictor control;
#ifdef PH_ASYNC_RECEIVE_TIME_BOUND
  if(!control.set_receive_time_bound(PH_ASYNC_RECEIVE_TIME_BOUND))return 8;
#endif
  if(!control.reset(1.))return 3;
  c::Plan plan{true,true,1,1,1,{std::stod(argv[3]),std::stod(argv[4])},{10,10},{std::stod(argv[5]),std::stod(argv[5])},{5,5}};
  c::Pair pending{};std::string line;bool fault=false;
  std::cout<<std::setprecision(17);
  for(int tick=0;std::getline(std::cin,line);++tick) {
    std::istringstream in(line);int source[2],p[2],v[2];
    if(!(in>>source[0]>>p[0]>>v[0]>>source[1]>>p[1]>>v[1]))return 4;
    const double now=1.+.005*tick;
    std::array<ph_exo::control::AsynchronousPredictor::Alignment::Sample,2> samples;
    for(unsigned i=0;i<2;++i) {
      samples[i]={1.+.005*source[i],p[i]*.1*c::pi/180.,v[i]*10.*2*c::pi/(60.*21.*8.)};
      if(tick>0 && !control.receipt(i,{now,pending[i],static_cast<std::uint64_t>(tick)}))fault=true;
    }
    c::Pair selected{};c::Torque torque{};
    if(!fault&&!control.update(now,samples,plan,
      [&model](const c::Pair& alpha,const c::Pair& velocity,c::ModelState& state,std::array<double,4>& mass) noexcept {
        ph_exo::ModelState native{};
        if(!model.evaluate(alpha,velocity,native))return false;
        state={native.q,native.dq,native.gravity,native.A,native.residual,native.iterations};mass=native.mass;return true;
      },selected,torque))fault=true;
    for(unsigned i=0;i<2;++i) {
      const double applied=pending[i];
      w::ForceControlCommand command;command.torque_nm=fault?0.:selected[i]+.25*30./4095.;
      w::ForceControlPayload bytes;
      if(!w::encode_force_control(command,w::ake60_8_ranges(),bytes))return 5;
      auto code=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
      pending[i]=w::dequantize(code,-15,15,12);
      if(!fault&&std::abs(pending[i]-selected[i])>1e-10)return 6;
      std::cout<<(fault?0.:torque.motor_nm[i])<<' '<<applied<<' ';
    }
    const auto& state=control.current().state();const auto& aligned=control.aligned();
    std::cout<<state.q[0]<<' '<<state.q[1]<<' '<<state.dq[0]<<' '<<state.dq[1]<<' '<<fault<<' '<<control.reason()<<' '
      <<std::llround((aligned.source-1.)/.005)<<' '<<std::llround(aligned.age[0]/.005)<<' '<<std::llround(aligned.age[1]/.005)<<' '
      <<std::llround(aligned.bracket[0]/.005)<<' '<<std::llround(aligned.bracket[1]/.005)<<'\n'<<std::flush;
  }
 }catch(const std::exception& e){std::cerr<<e.what();return 7;}
}
