// No assert(): checks remain active under -DNDEBUG. No ROS/MuJoCo/devices.
#include "ref01_impedance_core.hpp"
#include "../../ros2_ws/src/mech_bringup/include/mech_bringup/ak30_kinematic_lease.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace core = mech::mech_controllers::ref01;
int failures = 0;
int checks = 0;
void require(bool condition, const char* description) {
  ++checks;
  if (!condition) { ++failures; std::cerr << "FAIL: " << description << '\n'; }
}
bool near(double a, double b) { return std::abs(a-b) < 1e-12; }

core::Plan plan(double epoch = 10.) {
  return {true, true, epoch, 1., epoch+.002,
          {0., core::pi/9.}, {2.4, 2.}, {.18, .15}, {.2, .2}};
}
core::Feedback feedback(double source = 10.005, double sequence = 1.) {
  return {{{core::pi/4.+.2, .4, source, sequence},
           {core::pi/4.+.1, .3, source, sequence}}};
}
struct Evaluate {
  int calls{0};
  bool invalid{false};
  bool operator()(core::Pair alpha, core::Pair velocity, core::ModelState& state) {
    ++calls;
    require(near(alpha[0], .1) && near(alpha[1], .2), "motor105/104 -> alpha1/2 mapping");
    require(near(velocity[0], .3) && near(velocity[1], .4), "motor velocity mapping");
    state = {{0., core::pi/9.}, {0., 0.}, {.1, .2}, {1., 0., 0., 1.}, 0., 0};
    if (invalid) state.q[0] = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
};

void fresh_pair_and_duplicate() {
  core::Session session;
  Evaluate evaluate;
  session.activate(10.);
  auto f = feedback();
  auto p = plan();
  auto o = session.update(f, p, 10.006, .005, evaluate);
  require(o.mode == core::Mode::Following && o.refresh && o.generation == 1, "first complete pair follows");
  require(near(o.torque.motor_nm[0], .2) && near(o.torque.motor_nm[1], .1), "paired motor effort ordering");
  o = session.update(f, p, 10.011, .005, evaluate);
  require(!o.refresh && o.generation == 1 && evaluate.calls == 1, "duplicate pair does not recompute or renew");
  f[0].sequence = 2.; f[0].source_time_s = 10.012;
  o = session.update(f, p, 10.016, .005, evaluate);
  require(!o.refresh && o.generation == 1 && near(o.source_time_s, 10.005), "one fresh axis cannot renew pair lease");
  f[1].sequence = 2.; f[1].source_time_s = 10.017;
  o = session.update(f, p, 10.021, .005, evaluate);
  require(o.refresh && o.generation == 2 && near(o.source_time_s, 10.012), "new complete pair uses older axis source");
}

void expiry_and_latch() {
  core::Session session;
  Evaluate evaluate;
  session.activate(10.);
  auto f = feedback();
  auto p = plan();
  session.update(f, p, 10.006, .005, evaluate);
  for (int i = 1; i <= 12; ++i) session.update(f, p, 10.006+i*.005, .005, evaluate);
  auto o = session.observation();
  require(o.mode == core::Mode::Faulted && o.reason == core::Reason::FeedbackExpired,
          "original source expires despite repeated receptions");
  require(!o.refresh && o.torque.alpha_nm == core::Pair{}, "expired state supplies no new effort");
  o = session.update(feedback(10.070, 2.), p, 10.071, .005, evaluate);
  require(o.mode == core::Mode::Faulted && evaluate.calls == 1, "fresh feedback cannot clear latched fault");
  session.deactivate(); session.activate(10.080);
  o = session.update(feedback(10.085, 1.), plan(10.080), 10.086, .005, evaluate);
  require(o.mode == core::Mode::Following && o.generation == 1, "explicit lifecycle reset permits new session");
}

void changed_input_and_deadlines() {
  for (int scenario = 0; scenario < 5; ++scenario) {
    core::Session session;
    Evaluate evaluate;
    session.activate(10.);
    auto f = feedback(); auto p = plan();
    session.update(f, p, 10.006, .005, evaluate);
    double now = 10.011, dt = .005;
    core::Reason expected = core::Reason::None;
    if (scenario == 0) { f[0].position += .001; expected = core::Reason::FeedbackMutation; }
    if (scenario == 1) { p.reference[0] += .001; expected = core::Reason::PlanChanged; }
    if (scenario == 2) { now = 10.017; expected = core::Reason::Period; }
    if (scenario == 3) { dt = .010; expected = core::Reason::Period; }
    if (scenario == 4) { p.enabled = false; expected = core::Reason::Disabled; }
    const auto o = session.update(f, p, now, dt, evaluate);
    require(o.mode == core::Mode::Faulted && o.reason == expected, "input mutation/plan change/late period/disable latches");
    require(!o.refresh, "rejected update cannot renew command generation");
  }
  core::Session session;
  Evaluate evaluate;
  session.activate(10.);
  auto p = plan();
  for (int i = 0; i <= 21; ++i)
    session.update(feedback(10.005+i*.005, i+1), p, 10.006+i*.005, .005, evaluate);
  require(session.observation().reason == core::Reason::PlanExpired, "fresh sensors cannot extend old plan source");
}

void activation_and_bad_model() {
  core::Session session;
  Evaluate evaluate;
  session.activate(10.);
  auto o = session.update(feedback(10., 1.), plan(), 10.001, .005, evaluate);
  require(o.mode == core::Mode::Waiting && evaluate.calls == 0, "pre-activation feedback is not consumed");
  evaluate.invalid = true;
  o = session.update(feedback(10.005, 2.), plan(), 10.006, .005, evaluate);
  require(o.mode == core::Mode::Faulted && o.reason == core::Reason::Model && !o.refresh,
          "NaN model result is rejected even when evaluator claims success");
  session.activate(11.);
  evaluate.invalid = false;
  session.update(feedback(11.005, 1.), plan(11.), 11.006, .005, evaluate);
  o = session.reject(core::Reason::ComputeDeadline);
  require(o.mode == core::Mode::Faulted && !o.refresh, "post-compute deadline rejection prevents write");
}

void numeric_allocation_and_validation() {
  auto p = plan();
  core::ModelState state{{0., core::pi/9.}, {0., 0.}, {.8, -.4}, {1., 0., 0., 1.}, 0., 0};
  core::Torque result;
  require(core::fixed_pose_torque(state, p, result), "finite allocation succeeds");
  require(near(result.scale, .25) && near(result.alpha_nm[0], .2) && near(result.alpha_nm[1], -.1),
          "common factor preserves generalized force direction");
  p.caps = {.1, .2};
  require(core::fixed_pose_torque(state, p, result) && near(result.scale, .125)
          && near(result.alpha_nm[1], -.05), "asymmetric caps retain common factor");
  state.A = {1., 1., 1., 1.};
  require(!core::fixed_pose_torque(state, p, result), "singular allocation is rejected");
  state.A = {1., 0., 0., 1.};
  state.gravity[0] = std::numeric_limits<double>::quiet_NaN();
  require(!core::fixed_pose_torque(state, p, result), "NaN gravity is rejected");
  state.gravity[0] = .1; p.caps[0] = 0.;
  require(!core::fixed_pose_torque(state, p, result), "zero cap is rejected");

  std::array<double, 12> frame{1., 1., 10.002, 1., 0., core::pi/9., 2.4, 2., .18, .15, .2, .2};
  core::Plan parsed;
  require(core::valid_plan(frame, 10.003, 10., 0., 0., parsed), "versioned plan accepted");
  require(!core::valid_plan(frame, 10.003, 10., 1., 10.002, parsed), "duplicate plan sequence rejected");
  require(!core::valid_plan(frame, 10.001, 10., 0., 0., parsed), "future plan source rejected");
  frame[1] = 1.5;
  require(!core::valid_plan(frame, 10.003, 10., 0., 0., parsed), "fractional sequence rejected");
  frame[1] = 2.; frame[7] = std::numeric_limits<double>::quiet_NaN();
  require(!core::valid_plan(frame, 10.003, 10., 0., 0., parsed), "NaN plan rejected");
}


void dynamic_reference() {
  std::array<double,12> frame{1.,1.,10.002,1.,0.,core::pi/9.,2.4,2.,.18,.15,.2,.2};
  core::Plan parsed;
  require(core::valid_plan(frame,10.003,10.,0.,0.,parsed,.2,false,true,{.1,-.2}),
          "dynamic q/dq sample accepted");
  require(parsed.dynamic && parsed.reference_velocity==core::Pair{.1,-.2},"q/dq atomic parse");
  require(!core::valid_plan(frame,10.003,10.,0.,0.,parsed,.2,false,false,{.1,0.}),
          "fixed mode cannot silently take velocity target");
  require(!core::valid_plan(frame,10.003,10.,0.,0.,parsed,.2,false,true,
          {std::numeric_limits<double>::quiet_NaN(),0.}),"invalid dq rejected");
  auto p=plan(); p.dynamic=true; p.reference_velocity={.1,-.2};
  core::ModelState state{{0.,core::pi/9.},{.03,-.04},{0.,0.},{1.,0.,0.,1.},0.,0};
  core::Torque torque;
  require(core::fixed_pose_torque(state,p,torque) &&
          near(torque.feedback_q_nm[0],.18*(.1-.03)) &&
          near(torque.feedback_q_nm[1],.15*(-.2+.04)),"relative velocity damping signs");
  for(int change=0;change<6;++change) {
    core::Session session;Evaluate model;session.activate(10.);
    p=plan();p.dynamic=true;
    auto o=session.update(feedback(),p,10.006,.005,model);
    require(o.mode==core::Mode::Following,"dynamic starts at current quiet pose");
    p.sequence=2.;p.issued_at=10.010;
    if(change==0) {p.reference[0]+=.02;p.reference_velocity[1]=-.03;}
    if(change==1) p.stiffness[0]=2.;
    if(change==2) p.damping[1]=.1;
    if(change==3) p.caps[0]=.1;
    if(change==4) p.dynamic=false;
    if(change==5) p.enabled=false;
    o=session.update(feedback(10.010,2.),p,10.011,.005,model);
    require(change==0 ? (o.mode==core::Mode::Following && o.refresh &&
                         near(o.torque.feedback_q_nm[0],2.4*.02)) :
            (o.mode==core::Mode::Faulted &&
             o.reason==(change==5?core::Reason::Disabled:core::Reason::PlanChanged)),
            "dynamic updates targets only; gains/caps/mode immutable");
  }
  core::Session session;Evaluate model;session.activate(10.);
  p=plan();p.dynamic=true;
  session.update(feedback(),p,10.006,.005,model);
  for(int i=1;i<=21;++i)
    session.update(feedback(10.005+i*.005,1.+i),p,10.006+i*.005,.005,model);
  require(session.observation().reason==core::Reason::PlanExpired &&
          !session.observation().refresh,"fresh feedback cannot renew missing dynamic reference");
  p.issued_at=10.115;p.sequence=50.;
  auto o=session.update(feedback(10.115,50.),p,10.116,.005,model);
  require(o.mode==core::Mode::Faulted,"reference resumption does not auto resume fault");
  session.deactivate();session.activate(11.);p=plan(11.);p.dynamic=true;
  o=session.update(feedback(11.005,1.),p,11.006,.005,model);
  require(o.mode==core::Mode::Following,"explicit lifecycle reset permits dynamic again");
  session.deactivate();session.activate(12.);p=plan(12.);p.dynamic=true;p.reference_velocity[0]=.051;
  o=session.update(feedback(12.005,1.),p,12.006,.005,model);
  require(o.reason==core::Reason::InitialReference,"first dynamic target must start quietly");
}

void saturation_stays_in_cap() {
  bool within=true, accepted=true, proportional=true;
  for(double cap : {.1,.2,2.,10.}) for(int sign : {-1,1}) for(int n=1;n<=1000;++n) {
    auto p=plan();p.reference={0.,0.};p.caps={cap,cap*.7};
    core::ModelState s{};s.A={1.,0.,0.,1.};
    s.gravity={sign*(cap+n*.013),-sign*(cap+n*.017)};
    core::Torque t;
    within = core::fixed_pose_torque(s,p,t) && within;
    for(unsigned i=0;i<2;++i) {
      within = within && std::abs(t.alpha_nm[i])<=p.caps[i];
      proportional = proportional && std::abs(t.alpha_nm[i]-s.gravity[i]*t.scale)<1e-12;
      mech::mech_bringup::KinematicCommandLease lease;lease.reset(0.,p.caps[i]);
      accepted = lease.update(t.alpha_nm[i],1.,10.,true,10001000000LL) && accepted;
      mech::mech_bringup::KinematicCommandLease strict;strict.reset(0.,p.caps[i]);
      const double over=std::nextafter(p.caps[i],std::numeric_limits<double>::infinity());
      accepted = !strict.update(over,1.,10.,true,10001000000LL) && strict.faulted() && accepted;
    }
  }
  require(within,"saturation stays exactly inside caps under roundoff");
  require(accepted,"saturated commands accepted while genuine above-cap inputs still rejected");
  require(proportional,"common scaling preserved up to floating roundoff");
}

int main() {
  saturation_stays_in_cap();
  {
    core::Plan parsed;
    std::array<double,12> f{{1,1,1.005,1,0,.3,30,30,1,1,10,10}};
    require(core::valid_plan(f,1.01,1.,0.,0.,parsed,10.,true),"bench accepts K30 B1 cap10");
    require(!core::valid_plan(f,1.01,1.,0.,0.,parsed,10.,false),"simulation gain limits preserved");
    for(unsigned i=6;i<12;++i) {
      auto bad=f;bad[i]+=.001;
      require(!core::valid_plan(bad,1.01,1.,0.,0.,parsed,10.,true),"upper K B cap boundaries reject");
    }
  }
  dynamic_reference();
  fresh_pair_and_duplicate();
  expiry_and_latch();
  changed_input_and_deadlines();
  activation_and_bad_model();
  numeric_allocation_and_validation();
  core::Plan plan;std::array<double,12> frame{{1,1,1.005,1,0,.3,2.4,2.,.18,.15,5,5}};
  require(!core::valid_plan(frame,1.01,1.,0.,0.,plan),"simulation retains 0.2 Nm cap");
  require(core::valid_plan(frame,1.01,1.,0.,0.,plan,5.),"explicit bench accepts 5 Nm cap");
  frame[10]=5.01;require(!core::valid_plan(frame,1.01,1.,0.,0.,plan,5.),"bench rejects above 5 Nm");
  frame={{1,1,1.005,1,0,core::pi/9.,10,10,1,1,5,5}};
  require(!core::valid_plan(frame,1.01,1.,0.,0.,plan,5.),"cap alone does not opt into bench gains");
  require(core::valid_plan(frame,1.01,1.,0.,0.,plan,5.,true),"bench K10 B1 plan accepted");
  require(plan.stiffness==core::Pair{10,10} && plan.damping==core::Pair{1,1},"bench gain field ordering");
  frame[10]=10.;frame[11]=10.;
  require(core::valid_plan(frame,1.01,1.,0.,0.,plan,10.,true),"10 Nm bench plan accepted");
  frame[10]=10.01;
  require(!core::valid_plan(frame,1.01,1.,0.,0.,plan,10.,true),"above 10 Nm rejected");
  frame[10]=10.;
  require(!core::valid_plan(frame,1.01,1.,0.,0.,plan,10.01,true),"unsupported cap rejected");
  core::Session bench_session;Evaluate bench_model;bench_session.activate(1.);
  auto observed=bench_session.update(feedback(1.005,1.),plan,1.01,.005,bench_model);
  require(observed.mode==core::Mode::Following && observed.refresh,"accepted bench plan enters Following");
  for(unsigned index=6;index<=9;++index){
    auto bad=frame;bad[index]=(index<=7?30.:1.)+.01;
    require(!core::valid_plan(bad,1.01,1.,0.,0.,plan,5.,true),"bench gain above authorized value rejected");
  }
  std::cout << "checks=" << checks << " failures=" << failures << '\n';
  return failures == 0 ? 0 : 1;
}
