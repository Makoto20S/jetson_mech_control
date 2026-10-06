// Offline standalone tests: no ROS, gtest, physical serial port or CAN access.
#include "observer.hpp"
#include <algorithm>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <thread>
#include "mech_bringup/pass_through_init.hpp"

namespace {
using namespace observer;
using namespace mech::mech_control_core;
unsigned checks=0;
void check(bool yes,const char* what,int line){++checks;if(!yes)throw std::runtime_error(std::to_string(line)+": "+what);}
#define CHECK(x) check((x),#x,__LINE__)
template<class F>void rejects(F action){bool caught=false;try{action();}catch(const std::exception&){caught=true;}CHECK(caught);}
Bytes literal(const std::string& s){Bytes b;for(std::size_t i=0;i<s.size();i+=2)b.push_back(static_cast<std::uint8_t>(std::stoul(s.substr(i,2),nullptr,16)));return b;}
// Independent frozen bytes: command target scaling is 10000/deg and 100/10;
// feedback records below are synthetic protocol-format fixtures, NOT measurements.
const std::array<Bytes,4> four{{
 literal("f7120e000bf8e2680600000c08000cd528000a000a"),
 literal("f7120e000b0405690600000c08000ddec8000a000a"),
 literal("f7120e000b2085682900000c080349000000002800"),
 literal("f7120e000b7fcc692900000c08038d000000002900")}};
const Bytes sequence0=literal("f7120e000b55b40101ff1f0c0812345678000000a4");
const Bytes sequence1=literal("f7120e000b63360201ff1f0c0812345678000000b5");
const Bytes observed_prefix=literal("f7121100ff0eb0682900682900000c0803440000fffa2c00");
std::uint64_t u64(const Bytes& b,std::size_t offset){std::uint64_t v=0;for(unsigned i=0;i<8;++i)v|=std::uint64_t(b.at(offset+i))<<(8*i);return v;}
unsigned u32(const Bytes& b,std::size_t offset){unsigned v=0;for(unsigned i=0;i<4;++i)v|=unsigned(b.at(offset+i))<<(8*i);return v;}
Config observing(){Config c;c.seconds=1;return c;}
Config emitting(){auto c=observing();c.role="fixture";c.hz=10;c.frames_per_id=1;c.drain_ms=200;c.nonce=0x78563412;return c;}
class FakePort final:public CdcSerialPort {
 public:
  bool opened{true};unsigned closes{0},reads{0};std::size_t fragment{1008},forced_size{0};
  std::deque<Bytes> incoming;std::vector<Bytes> writes;
  std::deque<TransportResult> outcomes;TransportResult read_result{TransportResult::Ok};
  std::function<void(const Bytes&)> on_write;
  bool is_open()const noexcept override{return opened;}
  bool open()noexcept override{opened=true;return true;}
  void close()noexcept override{opened=false;++closes;}
  TransportResult write_all(const std::uint8_t* p,std::size_t n)noexcept override{
    writes.emplace_back(p,p+n);auto result=TransportResult::Ok;
    if(!outcomes.empty()){result=outcomes.front();outcomes.pop_front();}
    if(result==TransportResult::Ok&&on_write)on_write(writes.back());
    return result;
  }
  TransportResult read_some(std::uint8_t* p,std::size_t cap,std::size_t& n)noexcept override{
    ++reads;n=0;
    if(forced_size){n=forced_size;std::fill_n(p,std::min(cap,n),0);return read_result;}
    if(read_result!=TransportResult::Ok)return read_result;
    if(incoming.empty())return TransportResult::WouldBlock;
    auto& b=incoming.front();n=std::min({cap,fragment,b.size()});std::copy_n(b.begin(),n,p);
    b.erase(b.begin(),b.begin()+n);if(b.empty())incoming.pop_front();return TransportResult::Ok;
  }
};
void closed(const FakePort& p){CHECK(!p.opened);CHECK(p.closes==1);}
void golden_fixture(){
  auto c=emitting();c.lanes=2;
  auto packets=fixture_packets(c,0);CHECK(packets.size()==2);CHECK(packets[0]==sequence0);CHECK(packets[1]==sequence1);
  c.profile="four-id";c.frames_per_id=4;CHECK(c.id_count()==4);CHECK(c.count()==4);
  for(unsigned s=0;s<4;++s){auto out=fixture_packets(c,s);CHECK(out.size()==4);for(unsigned lane=0;lane<4;++lane){CHECK(out[lane]==four[lane]);CHECK(fixture_frame(c,lane,s).direction==FrameDirection::Tx);}}
  CHECK(fixture_frame(c,0,0).id.value==0x668);CHECK(fixture_frame(c,1,0).id.value==0x669);
  CHECK(fixture_frame(c,2,0).id.value==0x2968);CHECK(fixture_frame(c,3,0).id.value==0x2969);
  CHECK(plan_json(c).find("\"simulated_feedback\":true")!=std::string::npos);
  rejects([&]{fixture_frame(c,4,0);});rejects([&]{fixture_frame(c,0,4);});
}
void capture_schema(){
  Capture full(45);const auto& init=mech::mech_bringup::kPassThroughInitFrame;
  full.append(1,0,13,13,init.data(),13,100,200);const auto& bytes=full.bytes();
  CHECK(bytes.size()==45);CHECK(bytes[0]==1&&bytes[1]==0&&bytes[2]==0&&bytes[3]==0);
  CHECK(u32(bytes,4)==29);CHECK(u64(bytes,8)==200);CHECK(u64(bytes,16)==100);
  CHECK(u32(bytes,24)==13&&u32(bytes,28)==13);CHECK(std::equal(bytes.begin()+32,bytes.end(),init.begin()));
  rejects([&]{full.reserve_record(0);});
  Capture count(77,true);count.append(1,0,13,13,init.data(),13,100,200);
  CHECK(count.bytes().size()==45&&u32(count.bytes(),4)==29);CHECK(count.count_only());
  count.append(2,0,13,13,init.data(),13,201,202);
  CHECK(count.bytes().size()==77&&u32(count.bytes(),49)==16);
  Capture invalid(4096);rejects([&]{invalid.append(2,0,10,10,nullptr,10,100,200);});
  rejects([&]{invalid.append(2,0,1,1,init.data(),1,200,100);});
  rejects([&]{invalid.reserve_record(1009);});
}
void observe_all_ids(){
  for(bool count_only:{false,true}){
    auto c=observing();c.seconds=3;c.count_only=count_only;FakePort p;p.fragment=1;Capture capture(2*1024*1024,count_only);unsigned ready=0;
    Bytes joined;for(const auto& packet:four)joined.insert(joined.end(),packet.begin(),packet.end());
    joined.insert(joined.end(),observed_prefix.begin(),observed_prefix.end());
    std::array<std::uint8_t,kMaxCanPayloadBytes> payload{};payload[0]=0xde;payload[1]=0xad;payload[2]=0xbe;
    auto unknown=*RawCanFrame::create(1,*CanId::create(0x123,CanFrameFormat::Standard),CanFrameType::Classic,FrameDirection::Tx,3,payload,*MonotonicTime::from_nanoseconds(1));
    auto other=dual_board::encode(unknown);joined.insert(joined.end(),other.begin(),other.end());
    auto r=run(p,c,capture,[]{return false;},[&]{++ready;p.incoming.push_back(joined);});
    if(!r.complete())throw std::runtime_error("observe fragmented: "+r.error);
    CHECK(r.complete());CHECK(r.raw_complete()==!count_only);CHECK(ready==1);CHECK(p.writes.size()==1);
    CHECK(p.writes.front()==Bytes(mech::mech_bringup::kPassThroughInitFrame.begin(),mech::mech_bringup::kPassThroughInitFrame.end()));
    CHECK(r.tx_frames==0&&r.write_calls==1);CHECK(r.rx_frames==6);CHECK(r.rx_bytes==joined.size());
    CHECK(r.start_ns<=r.ready_ns&&r.ready_ns<=r.first_frame_ns&&r.first_frame_ns<=r.last_frame_ns&&r.last_frame_ns<=r.end_ns);
    CHECK(r.ids.size()==5);CHECK(std::any_of(r.ids.begin(),r.ids.end(),[](const IdCount& id){return id.id==0x123&&!id.extended&&id.frames==1;}));
    CHECK(r.read_calls==p.reads);closed(p);
    std::size_t offset=0;std::uint64_t last=0;unsigned writes=0;
    while(offset<capture.bytes().size()){
      const auto& b=capture.bytes();auto size=u32(b,offset+4);auto end=u64(b,offset+8);auto begin=u64(b,offset+16);
      CHECK(begin<=end&&begin>=last);CHECK(size>=16);CHECK(b[offset+1]==0);last=end;
      if(b[offset]==1)++writes;
      if(count_only&&b[offset]==2)CHECK(size==16);
      offset+=16+size;
    }
    CHECK(offset==capture.bytes().size()&&writes==1);
  }
}
void failure_exits(){
  auto c=observing();
  for(auto result:{TransportResult::Fault,TransportResult::WouldBlock,TransportResult::Disconnected}){
    FakePort p;Capture cap(8192);p.outcomes.push_back(result);unsigned ready=0;
    auto r=run(p,c,cap,[]{return false;},[&]{++ready;});CHECK(!r.complete()&&!r.error.empty());CHECK(ready==0&&p.writes.size()==1);closed(p);
  }
  {
    FakePort p;Capture cap(8192);auto r=run(p,c,cap,[]{return true;});CHECK(!r.complete());CHECK(p.writes.empty());closed(p);
  }
  {
    FakePort p;Capture cap(8192);auto r=run(p,c,cap,[]{return false;},[]{throw std::runtime_error("ready_failed");});CHECK(r.error=="ready_failed");CHECK(p.writes.size()==1&&p.reads==0);closed(p);
  }
  {
    FakePort p;Capture cap(8192);p.read_result=TransportResult::Fault;auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete()&&!r.error.empty());CHECK(p.writes.size()==1);closed(p);
  }
  for(unsigned altered:{0U,4U,5U,13U}){
    FakePort p;Capture cap(8192);auto packet=four[0];packet[altered]^=1;p.incoming.push_back(packet);
    auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete()&&!r.error.empty());CHECK(p.writes.size()==1);closed(p);
  }
  {
    FakePort p;Capture cap(2*1024*1024);p.incoming.push_back(Bytes(four[0].begin(),four[0].begin()+10));
    auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete());CHECK(r.rx_frames==0&&p.writes.size()==1);closed(p);
  }
  for(std::size_t limit:{0U,44U,45U,1039U}){
    FakePort p;Capture cap(limit);auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete()&&!r.error.empty());CHECK(cap.bytes().size()<=limit&&p.writes.size()<=1);closed(p);
  }
  {
    FakePort p;Capture cap(8192);p.forced_size=1009;auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete());CHECK(p.writes.size()==1);closed(p);
  }
  {
    FakePort p;Capture cap(8192);p.read_result=TransportResult::WouldBlock;p.forced_size=1;auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete());closed(p);
  }
  {
    FakePort p;Capture cap(8192);c.seconds=0;auto r=run(p,c,cap,[]{return false;});CHECK(!r.complete());closed(p);
  }
}
void fixture_run(){
  auto c=emitting();c.lanes=2;FakePort p;Capture cap(2*1024*1024);unsigned ready=0;
  auto r=run(p,c,cap,[]{return false;},[&]{++ready;});CHECK(r.complete());CHECK(r.tx_frames==2&&ready==1);CHECK(p.writes.size()==3);CHECK(p.writes[1]==sequence0&&p.writes[2]==sequence1);closed(p);
  c.profile="four-id";c.frames_per_id=4;FakePort q;Capture raw(2*1024*1024);
  auto rr=run(q,c,raw,[]{return false;});CHECK(rr.complete());CHECK(rr.tx_frames==16);CHECK(q.writes.size()==17);
  for(unsigned i=1;i<q.writes.size();++i)CHECK(q.writes[i]==four[(i-1)%4]);
  closed(q);
  FakePort fault;Capture bad(8192);fault.outcomes={TransportResult::Ok,TransportResult::Fault};
  auto error=run(fault,c,bad,[]{return false;});CHECK(!error.complete());CHECK(fault.writes.size()==2&&error.tx_frames==0);closed(fault);
}
}
int main(){try{golden_fixture();capture_schema();observe_all_ids();failure_exits();fixture_run();std::cout<<"PASS "<<checks<<" observer checks\n";return 0;}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
