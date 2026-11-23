
#include "ref01_adaptive_predictor.hpp"
#include <cmath>
#include "ref01_pair_quantization.hpp"
#include <iostream>
#include <limits>
#include <stdexcept>

namespace c = mech::mech_controllers::ref01;
using Predictor = c::AdaptiveBenchPredictor;
using Matrix = std::array<double,4>;
int checks = 0;
void check(bool ok, const char* message) {
  ++checks;
  if (!ok) throw std::runtime_error(message);
}
c::ModelState observation() {
  return {{0.,.3},{0.,0.},{0.,.1},{1.,1.,1.,-1.},0.,0};
}
Matrix nominal() { return {.006,0.,0.,.004}; }
c::Plan plan() {
  return {true,true,1.,1.,1.,{0.,.3},{10.,10.},{1.,1.},{5.,5.}};
}
void source_contract() {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (double source : {1.,.99,1.02,1.08,nan}) {
    Predictor p;
    check(p.observe(observation(),nominal(),1.),"seed");
    // 1.02 is newer but lacks the delivered-torque impulse for its interval.
    check(!p.observe(observation(),nominal(),source),"bad or unsupported source rejected");
    c::Torque t;
    check(!p.command(plan(),t) && t.motor_nm==c::Pair{},"fault blocks targets");
    check(!p.observe(observation(),nominal(),2.),"no automatic recovery");
    p.reset();
    check(!p.failed() && !p.initialized() && p.external()==c::Pair{},"reset lifecycle");
    check(p.strong_updates()==std::array<unsigned,2>{} &&
          p.weak_updates()==std::array<unsigned,2>{},"reset identification evidence");
  }
  for(double dt : {0.,-.005,.010,nan}) {
    Predictor p;
    check(p.observe(observation(),nominal(),1.),"seed invalid dt");
    check(!p.advance({0.,0.},dt),"bad integration period rejected");
  }
  Predictor p;
  check(!p.observe(observation(),Matrix{.006,.1,.1,.004},1.),"non-SPD inertia rejected");
  p.reset();
  check(p.observe(observation(),nominal(),1.),"seed finite test");
  check(!p.advance({nan,0.},.005),"nonfinite delivered target rejected");
}
void replay_equivalence() {
  Predictor direct;
  check(direct.observe(observation(),nominal(),1.),"seed replay test");
  for(int i=0;i<4;++i) check(direct.advance({.05,.05},.005),"source interval");
  auto source_snapshot=direct;
  auto measured=observation();measured.dq={.03,0.};
  check(direct.observe(measured,nominal(),1.02),"direct correction");
  const c::Pair sent[]{ {-.02,.08},{-.03,.07},{-.04,.06},{-.05,.05} };
  for(const auto& t:sent) check(direct.advance(t,.005),"direct delivered history");
  // Delayed arrival restores the source snapshot, not the current predictor.
  auto replay=source_snapshot;
  check(replay.observe(measured,nominal(),1.02),"delayed source correction");
  for(const auto& t:sent) check(replay.advance(t,.005),"replayed delivered history");
  for(unsigned i=0;i<2;++i) {
    check(std::abs(direct.state().q[i]-replay.state().q[i])<1e-12,"replay pose matches");
    check(std::abs(direct.state().dq[i]-replay.state().dq[i])<1e-12,"replay speed matches");
  }
  check(replay.source()==1.02,"replay does not refresh source");
  c::Torque a,b;
  check(direct.command(plan(),a) && replay.command(plan(),b),"replay commands valid");
  check(a.motor_nm==b.motor_nm,"same applied history yields same command");
}
void unknown_inertia_and_static_stiffness(double ratio) {
  Predictor p;
  auto state=observation();
  const c::Pair actual_mass{.006*ratio,.004*ratio};
  double max_late_speed=0.;
  for(int tick=0;tick<2400;++tick) {
    if(tick%4==0)
      check(p.observe(state,nominal(),1.+tick*.005),"unknown inertia observation");
    c::Torque t;
    check(p.command(plan(),t),"unknown inertia command");
    check(p.advance(t.motor_nm,.005),"predict actual sent target");
    const double external=tick>=80 ? .3 : 0.;
    const c::Pair force{t.alpha_nm[0]+t.alpha_nm[1]-state.gravity[0]+external,
                        t.alpha_nm[0]-t.alpha_nm[1]-state.gravity[1]};
    for(unsigned i=0;i<2;++i) {
      const double acc=force[i]/actual_mass[i];
      state.q[i]+=.005*state.dq[i]+.5*.005*.005*acc;
      state.dq[i]+=.005*acc;
      if(tick>=1600) max_late_speed=std::max(max_late_speed,std::abs(state.dq[i]));
    }
  }
  check(std::abs(state.q[0]-.03)<.003,"K10 yields 0.03 rad under 0.3 Nm");
  check(std::abs(state.q[1]-.3)<.003,"other axis remains at reference");
  check(max_late_speed<.01,"constant-load equilibrium stable in linear plant");
  if(ratio>1.)
    check(p.estimated_mass()[0]>.012,"learnt higher inertia without truth input");
}
void joint_quantizer_contract() {
  auto state=observation();
  state.A={1.,1.44,-1.,1.44};
  c::Pair wire{};
  check(ph_exo::experiment::select_wire({-.044,-.044},state,{.005,0.,0.,.003},wire),
        "valid joint quantization");
  check(wire[0]!=wire[1],"avoid simultaneous common-axis code jump");
  for(double v:wire)
    check(std::abs((v+15.)/(30./4095.)-std::round((v+15.)/(30./4095.)))<1e-9,
          "exact MIT code lattice");
  for(const c::Pair t : {c::Pair{5.,-5.},c::Pair{0.,0.},c::Pair{4.999,-4.999}}) {
    check(ph_exo::experiment::select_wire(t,state,{.005,0.,0.,.003},wire),"boundary selection");
    check(std::abs(wire[0])<=5.+1e-12 && std::abs(wire[1])<=5.+1e-12,"physical wire cap");
  }
  check(!ph_exo::experiment::select_wire({6.,0.},state,nominal(),wire),"invalid demand rejected");
  check(!ph_exo::experiment::select_wire({0.,0.},state,{0.,0.,0.,0.},wire),"invalid mass rejected");
}
int main() {
  try {
    source_contract();replay_equivalence();joint_quantizer_contract();
    unknown_inertia_and_static_stiffness(1.);
    unknown_inertia_and_static_stiffness(8.);
    std::cout<<"adaptive contract: "<<checks<<" checks passed\n";
  } catch(const std::exception& e) {
    std::cerr<<"adaptive contract failed after "<<checks<<" checks: "<<e.what()<<"\n";
    return 1;
  }
}
