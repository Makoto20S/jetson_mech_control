#include "mech_bringup/command_trace.hpp"

namespace mech::mech_bringup {
CommandTrace::Record* CommandTrace::append(const char* stage) noexcept {
  const auto index = total_++;
  if (index >= records_.size()) return nullptr;
  auto& r = records_.at(index);
  r.stage = stage; r.ns = now(); r.cycle = cycle;
  r.joint = joint; r.generation = generation; r.io = io;
  return &r;
}
}  // namespace mech::mech_bringup
