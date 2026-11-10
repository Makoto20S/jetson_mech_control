// Standalone test of the same pair guard used by the ROS controller on Orin.
#include "mech_bringup/mit_pair_guard.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace mech::mech_bringup;

void require(bool ok, const char * why) {
  if (!ok) { std::cerr << why << '\n'; std::exit(1); }
}

int main() {
  constexpr std::int64_t start = 1000000000, ms = 1000000;
  const MitPairLimits limits{.2, 5., 100., .5, 60., 100*ms, 60*ms, 30*ms};
  require(limits.valid() && !MitPairLimits{}.valid(), "limits");
  const std::array<MitPairAxis, 2> good{{{104, start/1000, 1, 0, -10, -.1, 40},
                                        {105, start/1000, 1, 0, 20, .2, 42}}};
  const MitPairTarget target{start+ms, {.8, -.1}};
  MitPairGuard guard;
  auto activate = [&] { guard.activate(limits, start); };
  auto begin = [&] {
    activate();
    auto first = guard.update(target, good, start+2*ms, .002);
    require(first.refresh && std::abs(first.effort[0]-.01)<1e-10 &&
            std::abs(first.effort[1]+.01)<1e-10, "pair sign/order and slew");
  };
  activate();
  require(!guard.update({start,{1.,1.}}, good, start+ms, .002).refresh, "pre-activation replay");
  require(!guard.update({start+5*ms,{1.,1.}}, good, start+ms, .002).refresh, "future command");
  begin();
  const auto stopped = guard.request_stop();
  require(stopped.refresh && stopped.effort == std::array<double,2>{} &&
          guard.mode()==PairMode::OperatorStop, "supervisor stop zeros both immediately");
  require(!guard.request_stop().refresh && !guard.update(target,good,start+3*ms,.002).refresh,
          "supervisor stop latches");
  activate();
  require(!guard.request_stop().refresh && !guard.update(target,good,start+2*ms,.002).refresh,
          "stop before first target cannot arm");
  begin();
  for (int tick=2; tick<=25; ++tick) {
    const auto w = guard.update(target, good, start+2*tick*ms, .002);
    require(w.refresh && w.effort[0]<=.2000000001 && w.effort[1]>=-.1000000001, "limit");
    if (tick==25) require(std::abs(w.effort[0]-.2)<1e-10 && std::abs(w.effort[1]+.1)<1e-10, "endpoints");
  }
  auto stop = guard.update({start+101*ms, {1.,1.}}, good, start+101*ms, .002);
  require(stop.refresh && stop.effort == std::array<double,2>{} && guard.mode()==PairMode::TargetExpired,
          "late command must not bridge lease gap");
  require(!guard.update(target, good, start+102*ms, .002).refresh, "latched expiry");
  for (int i=0; i<2; ++i) {
    for (int fault=0; fault<10; ++fault) {
      begin();
      auto axes=good;
      auto & a=axes[i];
      switch (fault) {
        case 0: a.id=999; break;
        case 1: a.valid=0; break;
        case 2: a.fault=1; break;
        case 3: a.received_us-=61000; break;
        case 4: a.received_us+=10000; break;
        case 5: a.erpm=-110; break;
        case 6: a.iq=-.51; break;
        case 7: a.temperature=61; break;
        case 8: a.erpm=std::numeric_limits<double>::quiet_NaN(); break;
        case 9: a.received_us-=31000; break;
      }
      auto w=guard.update(target, axes, start+4*ms, .002);
      require(w.refresh && w.effort==std::array<double,2>{}, "both axes zero on trip");
      require(guard.mode()==(fault==9 ? PairMode::PairSkew : PairMode::FeedbackInvalid), "fault attribution");
      require(!guard.update({start+5*ms,{.1,.1}},good,start+6*ms,.002).refresh, "no automatic restart");
    }
  }
  std::cout << "MIT pair guard: limits, signed pair, slew, replay, lease, 20 axis fault cases passed\n";
}
