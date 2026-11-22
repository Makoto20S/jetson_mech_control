// Offline measured-state replay. No ROS, ports, predictor or device commands.
#include "ref01_model.hpp"
#include "ref01_impedance_core.hpp"
#include "mech_protocol_cubemars/ak30_force_wire.hpp"
#include <iostream>
#include <iomanip>
#include <string>
namespace c=mech::mech_controllers::ref01;
namespace w=mech::mech_protocol_cubemars;
int main(int argc,char** argv) {
 if(argc!=4 && argc!=5 && argc!=6 && argc!=7)return 2;
 try {
  const double damping=argc>=5?std::stod(argv[4]):1.;
  if(!std::isfinite(damping)||damping<0||damping>1.)return 2;
  const double cap=argc>=6?std::stod(argv[5]):5.;
  if(!std::isfinite(cap)||cap<=0.||cap>10.)return 2;
  const double stiffness=argc==7?std::stod(argv[6]):10.;
  if(!std::isfinite(stiffness)||stiffness<=0||stiffness>30.)return 2;
  ph_exo::Ref01Model model(argv[1]);c::Session session;session.activate(.99);
  c::Plan plan{true,true,.99,1.,1.,{std::stod(argv[2]),std::stod(argv[3])},{stiffness,stiffness},{damping,damping},{cap,cap}};
  c::Feedback feedback{};c::Pair held{},previous{};bool warm=false;double now,prior=0.;
  std::cout<<std::setprecision(17);
  while(std::cin>>now>>feedback[0].position>>feedback[0].velocity>>feedback[0].source_time_s>>feedback[0].sequence
                    >>feedback[1].position>>feedback[1].velocity>>feedback[1].source_time_s>>feedback[1].sequence) {
   plan.issued_at=now;plan.sequence+=1.;
   const auto& o=session.update(feedback,plan,now,prior>0.?now-prior:.005,
    [&](const c::Pair&a,const c::Pair&v,c::ModelState& out) {
     for(unsigned i=0;i<2;++i)if(warm&&std::abs(a[i]-previous[i])>.15)return false;
     ph_exo::ModelState st{};if(!model.evaluate(a,v,st))return false;
     out={st.q,st.dq,st.gravity,st.A,st.residual,st.iterations};previous=a;warm=true;return true;
    });
   prior=now;
   if(o.mode==c::Mode::Faulted)held={};
   if(o.refresh)for(unsigned i=0;i<2;++i){
    w::ForceControlCommand command;command.torque_nm=o.torque.motor_nm[i];
    w::ForceControlPayload bytes;if(!w::encode_force_control(command,w::ake60_8_ranges(),bytes))return 3;
    const auto value=(static_cast<unsigned>(bytes[6]&15U)<<8U)|bytes[7];
    held[i]=w::dequantize(value,-15,15,12);
   }
   const auto& st=o.state;
   std::cout<<now<<' '<<static_cast<int>(o.mode)<<' '<<o.refresh<<' '<<c::reason_name(o.reason)
     <<' '<<held[0]<<' '<<held[1]<<' '<<st.q[0]<<' '<<st.q[1]<<' '<<st.dq[0]<<' '<<st.dq[1];
   for(unsigned i=0;i<2;++i)std::cout<<' '<<plan.stiffness[i]*(plan.reference[i]-st.q[i]);
   for(unsigned i=0;i<2;++i)std::cout<<' '<<-plan.damping[i]*st.dq[i];
   for(double g:st.gravity)std::cout<<' '<<g;
   std::cout<<' '<<o.torque.scale;
   for(double a:st.A)std::cout<<' '<<a;
   for(double t:o.torque.motor_nm)std::cout<<' '<<t;
   std::cout<<std::endl;
  }
 }catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 4;}
}
