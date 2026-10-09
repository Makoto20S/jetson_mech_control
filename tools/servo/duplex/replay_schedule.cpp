#include "replay_schedule.hpp"
#include <array>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace duplex {
namespace {
void require(bool ok, const char* why) { if (!ok) throw std::runtime_error(why); }
std::uint64_t number(const std::string& s) {
  require(!s.empty(), "replay_empty_number");
  std::uint64_t n=0;
  for (const unsigned char c:s) {
    require(c>='0' && c<='9', "replay_unsigned_decimal_required");
    require(n <= (std::numeric_limits<std::uint64_t>::max()-(c-'0'))/10,
            "replay_number_overflow");
    n=n*10+(c-'0');
  }
  return n;
}
}
void ReplaySchedule::validate() const {
  if (events.empty()) { require(profile.empty(), "replay_missing_events"); return; }
  require(profile=="fixed_forward__paired" || profile=="observed_order__paired" ||
          profile=="fixed_forward__observed_separation" ||
          profile=="observed_order__observed_separation", "replay_unknown_profile");
  require(events.size()==5500, "replay_event_count");
  std::array<std::array<unsigned,2>,2> counts{};
  std::array<std::array<std::uint64_t,2>,2> previous{};
  std::uint64_t last=0;
  for (const auto& e:events) {
    require(e.port<2 && e.lane<2 && e.offset_ns<kReplayDurationNs && e.offset_ns>=last,
            "replay_invalid_event_or_order");
    const auto n=counts[e.port][e.lane]++;
    require(!n || e.offset_ns-previous[e.port][e.lane] >= (e.port ? 15000000ULL:1500000ULL),
            "replay_per_id_rate_limit");
    previous[e.port][e.lane]=e.offset_ns;last=e.offset_ns;
  }
  require(counts[0][0]==2500 && counts[0][1]==2500 && counts[1][0]==250 && counts[1][1]==250,
          "replay_per_id_counts");
}
ReplaySchedule load_replay_schedule(const std::string& path) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  require(in.is_open(), "replay_file_open_failed");
  const auto length=in.tellg();
  require(length>0 && length<=200000, "replay_file_size");
  in.seekg(0);
  std::string bytes(static_cast<std::size_t>(length),'\0');
  require(static_cast<bool>(in.read(bytes.data(),length)), "replay_file_read_failed");
  std::istringstream input(bytes);
  ReplaySchedule s;
  std::string magic,duration,count,t,p,l;
  require(static_cast<bool>(input>>magic>>s.profile>>duration>>count), "replay_header_missing");
  require(magic=="SERVO_REPLAY_V1" && number(duration)==kReplayDurationNs && number(count)==5500,
          "replay_header_invalid");
  s.events.reserve(5500);
  for (unsigned i=0;i<5500;++i) {
    require(static_cast<bool>(input>>t>>p>>l), "replay_truncated");
    const auto port=number(p),lane=number(l);
    require(port<2 && lane<2, "replay_port_lane");
    s.events.push_back({number(t),static_cast<unsigned>(port),static_cast<unsigned>(lane)});
  }
  require(!(input>>t), "replay_trailing_data");
  s.validate();return s;
}
std::string replay_json(const ReplaySchedule& s) {
  s.validate();
  std::ostringstream out;
  out << "{\"profile\":\"" << s.profile << "\",\"duration_ns\":" << kReplayDurationNs
      << ",\"max_lateness_ns\":" << kReplayMaxLateNs << ",\"read_guard_ns\":" << kReplayReadGuardNs
      << ",\"timestamp_basis\":\"host_schedule_not_CAN_wire_time\",\"events\":[";
  bool first=true;
  for(const auto& e:s.events) {
    if(!first) out << ',';
    first=false;out << '[' << e.offset_ns << ',' << e.port << ',' << e.lane << ']';
  }
  out << "]}";return out.str();
}
bool replay_deadline_allowed(std::uint64_t deadline, std::uint64_t now) noexcept {
  return now>=deadline && now-deadline<=kReplayMaxLateNs;
}
}  // namespace duplex
