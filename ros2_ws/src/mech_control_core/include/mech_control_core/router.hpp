#pragma once

#include <cstdint>
#include <array>
#include <optional>
#include <vector>

#include "mech_control_core/frame.hpp"

namespace mech::mech_control_core {

struct FrameFilter final {
  CanFrameFormat format{CanFrameFormat::Standard};
  std::uint32_t value{0U};
  std::uint32_t mask{0U};
  std::optional<CanFrameType> frame_type;

  [[nodiscard]] bool is_valid() const noexcept {
    const auto maximum = format == CanFrameFormat::Standard
                             ? kMaxStandardCanId
                             : kMaxExtendedCanId;
    return value <= maximum && mask <= maximum;
  }

  [[nodiscard]] bool matches(const RawCanFrame& frame) const noexcept {
    return is_valid() && frame.id.format == format &&
           (!frame_type.has_value() || frame.type == *frame_type) &&
           ((frame.id.value & mask) == (value & mask));
  }

  [[nodiscard]] bool overlaps(const FrameFilter& other) const noexcept {
    if (!is_valid() || !other.is_valid() || format != other.format) {
      return false;
    }
    if (frame_type.has_value() && other.frame_type.has_value() &&
        frame_type != other.frame_type) {
      return false;
    }
    return ((value ^ other.value) & (mask & other.mask)) == 0U;
  }
};

struct FrameRoute final {
  std::uint16_t route_id{0U};
  FrameFilter filter;
  std::uint16_t fanout_group{0U};
};

enum class RouterError : std::uint8_t {
  InvalidRoute,
  DuplicateRoute,
  OverlappingFilter,
  CapacityExceeded,
};

class FrameRouter final {
 public:
  explicit FrameRouter(std::size_t capacity = 32U) : capacity_(capacity) {
    routes_.reserve(capacity_);
  }

  [[nodiscard]] std::optional<RouterError> add_route(
      const FrameRoute& route) {
    if (route.route_id == 0U || !route.filter.is_valid()) {
      return RouterError::InvalidRoute;
    }
    if (routes_.size() >= capacity_) {
      return RouterError::CapacityExceeded;
    }
    for (const auto& existing : routes_) {
      if (existing.route_id == route.route_id) {
        return RouterError::DuplicateRoute;
      }
      if (existing.filter.overlaps(route.filter) &&
          (existing.fanout_group == 0U ||
           existing.fanout_group != route.fanout_group)) {
        return RouterError::OverlappingFilter;
      }
    }
    routes_.push_back(route);
    return std::nullopt;
  }

  [[nodiscard]] std::vector<std::uint16_t> route(
      const RawCanFrame& frame) const {
    std::vector<std::uint16_t> destinations;
    for (const auto& route : routes_) {
      if (route.filter.matches(frame)) {
        destinations.push_back(route.route_id);
      }
    }
    return destinations;
  }

  template <std::size_t Capacity>
  [[nodiscard]] bool route_into(const RawCanFrame& frame,
                                std::array<std::uint16_t, Capacity>& output,
                                std::size_t& count) const noexcept {
    count = 0U;
    for (const auto& route : routes_) {
      if (!route.filter.matches(frame)) {
        continue;
      }
      if (count >= Capacity) {
        return false;
      }
      output[count++] = route.route_id;
    }
    return true;
  }

  [[nodiscard]] bool contains(std::uint16_t route_id) const noexcept {
    for (const auto& route : routes_) {
      if (route.route_id == route_id) {
        return true;
      }
    }
    return false;
  }

  [[nodiscard]] std::size_t size() const noexcept { return routes_.size(); }

 private:
  std::size_t capacity_;
  std::vector<FrameRoute> routes_;
};

}  // namespace mech::mech_control_core
