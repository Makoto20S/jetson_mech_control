// No devices: pure schedule validation plus linked in-memory serial endpoints.
#define main legacy_semantic_test_entry
#include "test_duplex.cpp"
#undef main
#include <fstream>

namespace {
ReplaySchedule fixture() {
  ReplaySchedule s;s.profile="fixed_forward__paired";
  for(unsigned i=0;i<2500;++i) {
    s.events.push_back({std::uint64_t(i)*2000000+100000,0,0});
    s.events.push_back({std::uint64_t(i)*2000000+120000,0,1});
    if(i%10==0) {
      s.events.push_back({std::uint64_t(i)*2000000+1000000,1,0});
      s.events.push_back({std::uint64_t(i)*2000000+1020000,1,1});
    }
  }
  s.validate();return s;
}
template<class F> void must_reject(F f) {
  bool failed=false;try{f();}catch(const std::exception&){failed=true;}CHECK(failed);
}
}
int main() {
  try {
    auto s=fixture();CHECK(s.events.size()==5500);
    CHECK(!replay_deadline_allowed(1000,999));
    CHECK(replay_deadline_allowed(1000,1000));
    CHECK(replay_deadline_allowed(1000,251000));
    CHECK(!replay_deadline_allowed(1000,251001));
    {auto x=s;x.events[0].port=2;must_reject([&]{x.validate();});}
    {auto x=s;x.events.pop_back();must_reject([&]{x.validate();});}
    {auto x=s;x.events[1].lane=0;must_reject([&]{x.validate();});}
    {auto x=s;x.events[1].offset_ns=0;must_reject([&]{x.validate();});}
    {auto x=s;x.events.back().offset_ns=kReplayDurationNs;must_reject([&]{x.validate();});}
    {auto x=s;x.profile="unreviewed";must_reject([&]{x.validate();});}
    Config c;c.replay=s;c.seconds=5;c.hz=500;c.feedback_hz=50;c.validate();
    {auto x=c;x.feedback_phase_ms=5;must_reject([&]{x.validate();});}
    {auto x=c;x.seconds=6;must_reject([&]{x.validate();});}
    {Config x; x.mixed_receive=true;must_reject([&]{x.validate();});}
    {auto x=c;x.mixed_receive=true;x.validate();CHECK(plan_json(x).find("frame_family_0_command")!=std::string::npos);}
    {Config x=c;x.feedback_sequence=true;must_reject([&]{x.validate();});}
    {must_reject([]{sequenced_feedback(0,250);});must_reject([]{sequenced_feedback(2,0);});}
    for(unsigned lane=0;lane<2;++lane) for(unsigned n=0;n<250;++n) {
      auto f=sequenced_feedback(lane,n);CHECK(f.id.value==0x2968+lane);
      CHECK(f.payload[2]==0x40+lane && f.payload[3]==n+1);
      const auto base=frame(true,lane);for(unsigned j=0;j<8;++j) if(j!=2 && j!=3)CHECK(f.payload[j]==base.payload[j]);
    }
#ifdef __linux__
    for(bool duplicate:{false,true}) {
      auto x=c;x.mixed_receive=true;x.feedback_sequence=true;FakePort a,b;link(a,b);
      auto prior=a.on_write;bool injected=false;const auto first=dual_board::encode(sequenced_feedback(1,0));
      a.on_write=[&](const Bytes& bytes){prior(bytes);if(duplicate && !injected && bytes==first){b.incoming.push_back(bytes);injected=true;}};
      Capture ca(8*1024*1024),cb(8*1024*1024);const auto r=run(a,b,x,ca,cb,[]{return false;});
      CHECK(r.complete());CHECK(r.content_match==!duplicate);CHECK(r.feedback_sequence_match==!duplicate);
      CHECK(a.writes.size()==5501 && b.writes.size()==1);closed(a,b);
    }

    {auto x=c;x.mixed_receive=true;FakePort a,b;link(a,b);Capture ca(8*1024*1024),cb(8*1024*1024);
     const auto r=run(a,b,x,ca,cb,[]{return false;});
     CHECK(r.complete());CHECK(r.content_match);CHECK(a.writes.size()==5501);CHECK(b.writes.size()==1);
     CHECK(r.content_b_feedback.received_per_id[0]==250 && r.content_b_feedback.received_per_id[1]==250);
     CHECK(r.b.tx_frames==0 && r.a.tx_frames==5500);closed(a,b);}

    // Happy path uses actual Linux monotonic scheduling, but no PTY/device.
    {FakePort a,b;link(a,b);Capture ca(8*1024*1024),cb(8*1024*1024);
     const auto result=run(a,b,c,ca,cb,[]{return false;});
     if(!result.complete()) std::cerr << result.error << '\n';
     CHECK(result.complete());CHECK(result.content_match);
     CHECK(result.sent_a[0]==2500 && result.sent_a[1]==2500);
     CHECK(result.sent_b[0]==250 && result.sent_b[1]==250);closed(a,b);}
    // Force a delay after first CAN write: second event must not be sent.
    {FakePort a,b;link(a,b);auto prior=a.on_write;
     a.on_write=[&](const Bytes& bytes){prior(bytes);if(bytes!=init)std::this_thread::sleep_for(std::chrono::milliseconds(2));};
     Capture ca(8*1024*1024),cb(8*1024*1024);
     const auto result=run(a,b,c,ca,cb,[]{return false;});
     CHECK(!result.complete());CHECK(result.error=="replay_deadline_missed_no_catchup");
     CHECK(result.sent_a[0]==1 && result.sent_a[1]==0 && result.sent_b[0]==0);closed(a,b);}
#endif
    std::cout << "replay checks=" << checks << " passed; Linux-only runtime checks excluded on Windows\n";
    return 0;
  }catch(const std::exception& e){std::cerr << e.what()<<'\n';return 1;}
}
