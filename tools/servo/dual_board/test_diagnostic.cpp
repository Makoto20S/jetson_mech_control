// Standalone offline tests: no ROS, gtest, real port or CAN access.
#include "diagnostic.hpp"
#include <algorithm>
#include <deque>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include "mech_bringup/pass_through_init.hpp"

namespace {
using namespace dual_board;
unsigned checks = 0;
void check(bool value, const char* expression, int line) {
  ++checks;
  if (!value) throw std::runtime_error(std::string("line ")+std::to_string(line)+": "+expression);
}
#define CHECK(x) check((x), #x, __LINE__)
template<class F> void throws(F f, const std::string& contains) {
  bool caught = false;
  try { f(); } catch (const std::exception& e) {
    caught = true; CHECK(std::string(e.what()).find(contains) != std::string::npos);
  }
  CHECK(caught);
}
Bytes literal(const std::string& s) {
  Bytes b;
  for (std::size_t i=0; i<s.size(); i+=2) b.push_back(static_cast<std::uint8_t>(std::stoul(s.substr(i,2),nullptr,16)));
  return b;
}
// Independent recorded production literals, copied from test_usb_cdc_pty_isolation.
const Bytes tx104 = literal("f7120e000b9160680600000c08000cc1a0000a000a");
const Bytes tx105 = literal("f7120e000bde7e690600000c08000af0c8000a000a");
const Bytes rx104 = literal("f7120e000b93f6682900000c0803440000fffa2c00");
const Bytes rx105 = literal("f7120e000b326269290000040802cd0000ffd22c00");
// Frozen sequence/batch vectors: independently calculated CRCs, never populated
// from the production encoder or diagnostic envelope under test.
const Bytes sequence0 = literal("f7120e000b55b40101ff1f0c0812345678000000a4");
const Bytes sequence1 = literal("f7120e000b63360201ff1f0c0812345678000000b5");
const Bytes sequence_batch = literal("f7121c00767b470101ff1f0c0812345678000000a40201ff1f0c0812345678000000b5");
const Bytes prefixed104 = literal("f7121100ff0eb0682900682900000c0803440000fffa2c00");
RawCanFrame from_literal(std::uint32_t id, const Bytes& p) {
  std::array<std::uint8_t,kMaxCanPayloadBytes> data{};
  std::copy_n(p.begin()+13,8,data.begin());
  return *RawCanFrame::create(1,*CanId::create(id,CanFrameFormat::Extended),CanFrameType::Classic,
      FrameDirection::Tx,8,data,*MonotonicTime::from_nanoseconds(1));
}
Config config(unsigned lanes=1) {
  Config c; c.hz=3; c.seconds=1; c.lanes=lanes; c.drain_ms=200; c.nonce=0x78563412;
  return c;
}
RawCanFrame reply(const Config& c, unsigned lane, unsigned sequence) {
  auto f=frame(c,lane,sequence); f.direction=FrameDirection::Rx; return f;
}
std::vector<RawCanFrame> parse(const Bytes& bytes, std::size_t chunk) {
  Parser p; std::vector<RawCanFrame> output;
  for (std::size_t i=0;i<bytes.size();i+=chunk)
    p.feed(bytes.data()+i,std::min(chunk,bytes.size()-i),[&](const RawCanFrame& f){output.push_back(f);});
  p.finish(); return output;
}
// Only used to produce deliberately corrupt but CRC-valid envelope shapes.
std::uint8_t independent_header_crc(const Bytes& bytes) {
  unsigned crc=255;
  for(unsigned i=1;i<4;++i) { crc^=bytes[i]; for(unsigned j=0;j<8;++j) crc=(crc&1)?((crc>>1)^140):(crc>>1); }
  return static_cast<std::uint8_t>(crc);
}
void golden_and_packing() {
  CHECK(encode(from_literal(0x668,tx104))==tx104);
  CHECK(encode(from_literal(0x669,tx105))==tx105);
  for(const auto* p:{&rx104,&rx105,&prefixed104}) {
    auto out=parse(*p,1); CHECK(out.size()==1);
    const auto& want=p==&rx105?rx105:rx104;
    CHECK(out[0].id.value==(p==&rx105?0x2969U:0x2968U));
    CHECK(out[0].payload_size==8); CHECK(out[0].direction==FrameDirection::Rx);
    CHECK(std::equal(out[0].payload.begin(),out[0].payload.begin()+8,want.begin()+13));
  }
  auto c=config(2);
  CHECK(encode(frame(c,0,0))==sequence0); CHECK(encode(frame(c,1,0))==sequence1);
  for(const std::string packing:{"separate","joined","batch"}) {
    c.packing=packing; auto group=packets(c,0);
    CHECK(group.size()==(packing=="separate"?2U:1U));
    if(packing=="separate") { CHECK(group[0]==sequence0); CHECK(group[1]==sequence1); }
    if(packing=="joined") {
      auto want=sequence0; want.insert(want.end(),sequence1.begin(),sequence1.end());
      CHECK(group[0]==want); CHECK(group[0].size()==42);
    }
    if(packing=="batch") { CHECK(group[0]==sequence_batch); CHECK(group[0].size()==35); }
    Bytes wire; for(const auto& b:group) wire.insert(wire.end(),b.begin(),b.end());
    for(std::size_t split=1;split<=wire.size();++split) {
      auto out=parse(wire,split); CHECK(out.size()==2);
      for(unsigned lane=0;lane<2;++lane) {
        CHECK(out[lane].id.value==0x1fff0101U+lane);
        CHECK(out[lane].payload==frame(c,lane,0).payload);
      }
    }
  }
  c.profile="mode6";
  CHECK(frame(c,0,0).id.value==0x668); CHECK(frame(c,1,0).id.value==0x669);
  CHECK(frame(c,0,0).payload==frame(c,0,2).payload);
  CHECK(frame(c,0,0).payload!=frame(c,1,0).payload);
  c.profile="sequence"; c.hz=500; c.seconds=120;
  auto last=frame(c,1,59999); CHECK(last.payload[4]==0x5f); CHECK(last.payload[5]==0xea); CHECK(last.payload[6]==0);
  throws([&]{frame(c,2,0);},"index"); throws([&]{frame(c,0,60000);},"index");
}
void parser_errors_and_bounds() {
  for(std::size_t cut=1;cut<rx104.size();++cut) {
    Parser p; unsigned delivered=0;
    p.feed(rx104.data(),cut,[&](const RawCanFrame&){++delivered;});
    CHECK(delivered==0); throws([&]{p.finish();},"truncated");
  }
  Parser empty; empty.feed(nullptr,0,[](const RawCanFrame&){}); empty.finish();
  for(unsigned index:{0U,1U,4U,5U,13U}) {
    auto bad=rx104; bad[index]^=1;
    throws([&]{parse(bad,bad.size());},index<2?"unexpected_envelope":(index==4?"header_crc":"payload_crc_or_frame_shape"));
  }
  for(unsigned length:{0U,5U,513U,65535U}) {
    auto bad=rx104; bad[2]=length&255; bad[3]=length>>8; bad[4]=independent_header_crc(bad);
    throws([&]{parse(bad,bad.size());},"payload_length");
  }
  throws([&]{envelope({});},"limit"); throws([&]{envelope(Bytes(513,0));},"limit");
  // 64 zero-payload standard records are the codec batch boundary.
  Bytes records;
  for(unsigned i=0;i<64;++i) records.insert(records.end(),{static_cast<std::uint8_t>(i),0,0,0,8,0});
  auto many=parse(envelope(records),1); CHECK(many.size()==64);
  records.insert(records.end(),{64,0,0,0,8,0});
  throws([&]{parse(envelope(records),7);},"payload_crc_or_frame_shape");
  // Parser reaches its full 519-byte bound and rejects malformed record shape,
  // rather than writing out of bounds. A valid minimum record is also accepted.
  auto maximum=envelope(Bytes(512,0)); CHECK(maximum.size()==519);
  throws([&]{parse(maximum,1);},"payload_crc_or_frame_shape");
  CHECK(parse(envelope(Bytes{1,0,0,0,8,0}),1).size()==1);
  auto wrong_prefix=prefixed104; wrong_prefix[7]^=1;
  Bytes payload(wrong_prefix.begin()+7,wrong_prefix.end());
  throws([&]{parse(envelope(payload),1);},"payload_crc_or_frame_shape");
  // A corrupted packet followed by a golden packet must fail, never resync.
  auto bad=rx104; bad[5]^=1; bad.insert(bad.end(),rx105.begin(),rx105.end());
  throws([&]{parse(bad,1);},"payload_crc_or_frame_shape");
  // The production codec currently recognizes a prefix only at packet start.
  // A speculative multi-record layout with a prefix before EACH record must
  // reject, rather than silently modifying the production receive rules.
  Bytes per_record(prefixed104.begin()+7,prefixed104.end());
  per_record.insert(per_record.end(),{0x69,0x29,0});
  per_record.insert(per_record.end(),rx105.begin()+7,rx105.end());
  throws([&]{parse(envelope(per_record),1);},"payload_crc_or_frame_shape");
  Parser poisoned; bad=rx104; bad[5]^=1;
  throws([&]{poisoned.feed(bad.data(),bad.size(),[](const RawCanFrame&){});},"payload_crc_or_frame_shape");
  throws([&]{poisoned.feed(rx104.data(),rx104.size(),[](const RawCanFrame&){});},"parser_failed");
  throws([&]{poisoned.finish();},"parser_failed");
  Parser null_input; throws([&]{null_input.feed(nullptr,1,[](const RawCanFrame&){});},"parser_null_input");
}
void matching() {
  for(unsigned count:{1U,2U}) {
    auto c=config(count); Matcher m(c);
    for(unsigned s=0;s<c.count();++s) for(unsigned lane=0;lane<count;++lane) m.sent(lane);
    // A faster second ID may arbitrate ahead by several sequence positions;
    // the sequence ordering contract is per ID.
    if(count==2) for(unsigned s=0;s<c.count();++s) CHECK(m.receive(reply(c,1,s)).empty());
    for(unsigned s=0;s<c.count();++s) CHECK(m.receive(reply(c,0,s)).empty());
    CHECK(m.complete()); CHECK(m.missing(0)==0);
    CHECK(m.receive(reply(c,0,0))=="duplicate"); CHECK(!m.complete());
  }
  auto c=config(2); Matcher order(c);
  for(unsigned s=0;s<3;++s) {order.sent(0);order.sent(1);}
  CHECK(order.receive(reply(c,0,2)).empty());
  CHECK(order.receive(reply(c,0,1))=="out_of_order");
  CHECK(order.lanes[0].out_of_order==1); CHECK(order.missing(0)==1); CHECK(!order.complete());
  Matcher missing(c); missing.sent(0);missing.sent(1);
  CHECK(missing.receive(reply(c,0,0)).empty()); CHECK(missing.missing(1)==1); CHECK(!missing.complete());
  auto changed=reply(c,0,0); changed.payload=reply(c,1,0).payload;
  CHECK(missing.receive(changed)=="data_or_frame_mismatch");
  changed=reply(c,0,0); changed.id.value=0x1fff0103;
  CHECK(missing.receive(changed)=="unexpected_id"); CHECK(missing.unexpected_id==1);
  changed=reply(c,0,0); changed.payload[0]^=1;
  CHECK(missing.receive(changed)=="data_or_frame_mismatch"); // old nonce
  changed=reply(c,0,2); CHECK(missing.receive(changed)=="unsent_or_invalid_sequence");
  for(unsigned field=0;field<9;++field) {
    Matcher m(c);m.sent(0); auto f=reply(c,0,0);
    switch(field) { case 0:f.direction=FrameDirection::Tx;break; case 1:f.remote_request=true;break;
      case 2:f.error_frame=true;break;case 3:f.bitrate_switch=true;break;case 4:f.payload_size=7;break;
      case 5:f.type=CanFrameType::FlexibleDataRate;break;case 6:f.id.format=CanFrameFormat::Standard;break;
      case 7:f.payload[7]^=1;break;default:f.logical_bus=2; }
    CHECK(m.receive(f)=="data_or_frame_mismatch"); CHECK(m.lanes[0].mismatch==1);
  }
  Matcher unsent(c); CHECK(unsent.receive(reply(c,0,0))=="unsent_or_invalid_sequence");
  throws([&]{missing.sent(2);},"send_count");
  for(unsigned i=0;i<3;++i)unsent.sent(0);
  throws([&]{unsent.sent(0);},"send_count");
  throws([&]{missing.missing(2);},"lane_index");
  for(const std::string profile:{"constant","mode6"}) {
    c.profile=profile; Matcher fixed(c);
    for(unsigned s=0;s<3;++s) for(unsigned lane=0;lane<2;++lane) fixed.sent(lane);
    // Same count and identical bytes cannot distinguish duplicate+loss or order.
    for(unsigned s=0;s<3;++s) for(unsigned lane=0;lane<2;++lane) CHECK(fixed.receive(reply(c,lane,2-s)).empty());
    CHECK(fixed.complete()); CHECK(fixed.lanes[0].out_of_order==0);
    CHECK(fixed.receive(reply(c,0,0))=="excess_fixed_payload"); CHECK(!fixed.complete());
    Matcher fixed_missing(c); fixed_missing.sent(0); CHECK(fixed_missing.missing(0)==1);
  }
}
void capture_and_config() {
  Capture c(19); const Bytes data{0xab,0xcd,0xef};
  c.append(1,2,3,data.data(),3,0x0102030405060708ULL);
  CHECK(c.bytes()==literal("01020300030000000807060504030201abcdef"));
  throws([&]{c.append(1,0,0,nullptr,0,0);},"capture_capacity"); CHECK(c.bytes().size()==19);
  Capture zero(0); throws([&]{zero.reserve_record(0);},"capture_capacity");
  Capture big(4096); throws([&]{big.reserve_record(1025);},"capture_capacity");
  throws([&]{Capture excessive(64U*1024U*1024U+1);},"capture_limit");
  throws([&]{big.append(1,0,0,nullptr,1,0);},"capture_null_input"); CHECK(big.bytes().empty());
  for(unsigned mutation=0;mutation<10;++mutation) {
    auto v=config();
    switch(mutation) {case 0:v.hz=0;break;case 1:v.hz=501;break;case 2:v.seconds=0;break;
      case 3:v.seconds=121;break;case 4:v.lanes=3;break;case 5:v.drain_ms=199;break;
      case 6:v.drain_ms=5001;break;case 7:v.log_mib=65;break;case 8:v.profile="bad";break;
      default:v.packing="batch";}
    throws([&]{v.validate();},"");
  }
}

class FakePort final : public CdcSerialPort {
 public:
  bool opened{true}; unsigned closes{0}, reads{0};
  std::vector<Bytes> writes; std::deque<Bytes> incoming;
  std::deque<TransportResult> write_results;
  TransportResult read_result{TransportResult::Ok};
  std::size_t forced_read_size{0};
  std::function<void(const Bytes&)> on_write;
  std::size_t fragment{1024};
  bool is_open() const noexcept override {return opened;}
  bool open() noexcept override {opened=true;return true;}
  void close() noexcept override {++closes;opened=false;}
  TransportResult write_all(const std::uint8_t* p,std::size_t n) noexcept override {
    writes.emplace_back(p,p+n);
    auto result=TransportResult::Ok;
    if(!write_results.empty()) {result=write_results.front();write_results.pop_front();}
    if(result==TransportResult::Ok && on_write) on_write(writes.back());
    return result;
  }
  TransportResult read_some(std::uint8_t* p,std::size_t cap,std::size_t& n) noexcept override {
    ++reads;n=0;
    if(forced_read_size){n=forced_read_size;std::fill_n(p,std::min(n,cap),0);return read_result;}
    if(read_result!=TransportResult::Ok)return read_result;
    if(incoming.empty())return TransportResult::WouldBlock;
    auto& front=incoming.front();n=std::min({cap,fragment,front.size()});
    std::copy_n(front.begin(),n,p);front.erase(front.begin(),front.begin()+n);
    if(front.empty())incoming.pop_front();
    return TransportResult::Ok;
  }
};
void closed(const FakePort& tx,const FakePort& rx) {CHECK(!tx.opened);CHECK(!rx.opened);CHECK(tx.closes==1);CHECK(rx.closes==1);}
void run_faults() {
  auto c=config(2);c.hz=1;
  for(const auto result:{TransportResult::Fault,TransportResult::WouldBlock,TransportResult::Disconnected}) {
    FakePort tx,rx;Capture capture(8192);tx.write_results.push_back(result);
    auto r=run(tx,rx,c,capture,[]{return false;});
    CHECK(r.error=="write_incomplete_no_retry");CHECK(!r.complete());
    CHECK(rx.writes.size()==1);CHECK(tx.writes.size()==1);closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);rx.write_results.push_back(TransportResult::Fault);
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="write_incomplete_no_retry");
    CHECK(tx.writes.empty());CHECK(rx.writes.size()==1);closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);
    auto r=run(tx,rx,c,capture,[]{return true;});CHECK(r.error=="interrupted");
    CHECK(tx.writes.empty());CHECK(rx.writes.empty());closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);rx.read_result=TransportResult::Fault;
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="read_fault");
    CHECK(tx.writes.size()==1);CHECK(rx.writes.size()==1);closed(tx,rx);
  }
  for(const auto outcome:{TransportResult::Ok,TransportResult::WouldBlock}) {
    FakePort tx,rx;Capture capture(8192);rx.read_result=outcome;rx.forced_read_size=1025;
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="read_size");
    CHECK(tx.writes.size()==1);CHECK(rx.writes.size()==1);closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);rx.read_result=TransportResult::WouldBlock;rx.forced_read_size=1;
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="read_would_block_with_bytes");
    CHECK(tx.writes.size()==1);closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);
    auto r=run(tx,rx,c,capture,[]()->bool{throw std::runtime_error("stop\"\\\n");});
    CHECK(r.error=="stop\"\\\n");CHECK(tx.writes.empty());CHECK(rx.writes.empty());
    CHECK(result_json(r,capture.bytes().size()).find("stop\\\"\\\\\\u000a")!=std::string::npos);closed(tx,rx);
  }
  for(std::size_t limit:{0U,28U,57U,1039U}) {
    FakePort tx,rx;Capture capture(limit);
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="capture_capacity");
    CHECK(capture.bytes().size()<=limit);CHECK(tx.writes.size()<=1);CHECK(rx.writes.size()<=1);closed(tx,rx);
  }
  for(bool sender:{false,true}) {
    FakePort tx,rx;Capture capture(8192);
    (sender?tx:rx).incoming.push_back(rx104);
    auto r=run(tx,rx,c,capture,[]{return false;});
    CHECK(r.error==(sender?"unexpected_sender_rx":"unexpected_id"));
    CHECK(tx.writes.size()==1);CHECK(rx.writes.size()==1);closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);rx.incoming.push_back(Bytes(rx104.begin(),rx104.begin()+10));
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="truncated_packet");
    CHECK(tx.writes.size()==1);closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);tx.opened=false;
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="ports_not_open");closed(tx,rx);
  }
  {
    FakePort tx,rx;Capture capture(8192);c.hz=0;
    throws([&]{run(tx,rx,c,capture,[]{return false;});},"");closed(tx,rx);
  }
}
void run_success_and_partial_evidence() {
  for(const std::string packing:{"separate","joined","batch"}) {
    auto c=config(2);c.hz=1;c.packing=packing;
    FakePort tx,rx;Capture capture(1024*1024);rx.fragment=1;
    tx.on_write=[&](const Bytes& p){if(p.size()!=13)rx.incoming.push_back(p);};
    auto r=run(tx,rx,c,capture,[]{return false;});
    CHECK(r.complete());CHECK(r.finished);CHECK(r.matcher.lanes[0].matched==1);CHECK(r.matcher.lanes[1].matched==1);
    CHECK(tx.writes.size()==(packing=="separate"?3U:2U));CHECK(rx.writes.size()==1);closed(tx,rx);
  }
  {
    auto c=config(2);c.hz=1;FakePort tx,rx;Capture capture(8192);
    tx.write_results={TransportResult::Ok,TransportResult::Fault};
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="write_incomplete_no_retry");
    CHECK(tx.writes.size()==2);CHECK(r.matcher.lanes[0].sent==0);CHECK(r.matcher.lanes[1].sent==0);
    CHECK(capture.bytes().size()>0);closed(tx,rx);
  }
  {
    auto c=config(2);c.hz=1;FakePort tx,rx;Capture capture(8192);
    // Deliberately corrupt a valid envelope on the first lane: stop before
    // any later sequence; both lane writes occurred before the bounded pump.
    tx.on_write=[&](const Bytes& p){if(p.size()!=13){auto bad=p;bad[5]^=1;rx.incoming.push_back(bad);}};
    auto r=run(tx,rx,c,capture,[]{return false;});CHECK(r.error=="payload_crc_or_frame_shape");
    CHECK(tx.writes.size()==3);closed(tx,rx);
  }
  {
    auto c=config();c.hz=1;FakePort tx,rx;Capture capture(8192);
    auto r=run(tx,rx,c,capture,[&]{return tx.writes.size()>1;});CHECK(r.error=="interrupted");
    CHECK(tx.writes.size()==2);CHECK(r.matcher.lanes[0].sent==1);closed(tx,rx);
  }
  {
    auto c=config(2);c.hz=1;FakePort tx,rx;Capture capture(8192);
    auto r=run(tx,rx,c,capture,[]{return false;});
    CHECK(r.finished);CHECK(r.error=="missing_frames");CHECK(!r.complete());
    for(unsigned lane=0;lane<2;++lane) {CHECK(r.matcher.lanes[lane].sent==1);CHECK(r.matcher.missing(lane)==1);}
    CHECK(tx.writes.size()==3);closed(tx,rx);
  }
  {
    auto c=config();c.hz=10;FakePort tx,rx;Capture capture(8192);
    tx.on_write=[&](const Bytes& p){if(p.size()!=13)std::this_thread::sleep_for(std::chrono::milliseconds(150));};
    auto r=run(tx,rx,c,capture,[]{return false;});
    CHECK(r.error=="scheduler_overrun_no_catchup");CHECK(!r.complete());
    CHECK(tx.writes.size()==2);CHECK(r.matcher.lanes[0].sent==1);closed(tx,rx);
  }
}
}
int main() {
  try {
    golden_and_packing();parser_errors_and_bounds();matching();capture_and_config();
    run_faults();run_success_and_partial_evidence();
    std::cout<<"PASS: "<<checks<<" offline diagnostic checks\n";return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
