#include "mech_bringup/command_trace.hpp"

namespace mech::mech_bringup {
CommandTrace::Record* CommandTrace::append(const char* stage) noexcept {
  const auto index = total_++;
  if (index >= records_.size()) {
    records_.publish(total_, false);
    return nullptr;
  }
  auto& r = records_[index];
  for (std::size_t i = 0; i + 1 < r.stage.size() && stage[i]; ++i) r.stage[i] = stage[i];
  r.ns = now(); r.cycle = cycle;
  r.joint = joint; r.generation = generation; r.io = io;
  return &r;
}
}  // namespace mech::mech_bringup
