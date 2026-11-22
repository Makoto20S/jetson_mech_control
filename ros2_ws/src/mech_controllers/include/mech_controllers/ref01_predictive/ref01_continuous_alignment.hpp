
#pragma once
#include <array>
#include <cmath>
#include <algorithm>

namespace ph_exo::experiment {
// Continuous-time experimental alignment. The caller declares the time basis.
// Interpolate each decoded motor channel to min(latest source ticks), using only
// already-arrived bracketing packets. No extrapolation or timestamp relabeling.
class ContinuousAlignment {
 public:
  struct Sample { double source{-1}; double position{}, velocity{}; };
  struct Pair {
    double source{-1};
    std::array<double,2> position{},velocity{};
    std::array<double,2> age{},bracket{};
    std::array<Sample,2> left{},right{};
  };
  bool update(unsigned axis,Sample packet,double now) {
    if(axis>1 || !std::isfinite(now) || !std::isfinite(packet.source) || packet.source<0 || packet.source>now ||
       !std::isfinite(packet.position) || !std::isfinite(packet.velocity)) return false;
    auto& n=count_[axis];
    if(n && packet.source<samples_[axis][n-1].source) {++discarded_;return true;}
    if(n && packet.source==samples_[axis][n-1].source) {
      return packet.position==samples_[axis][n-1].position &&
             packet.velocity==samples_[axis][n-1].velocity;
    }
    if(n==capacity) {
      for(unsigned i=1;i<n;++i) samples_[axis][i-1]=samples_[axis][i];
      --n;
    }
    samples_[axis][n++]=packet;
    return true;
  }
  bool align(double now,Pair& result) const {
    if(!std::isfinite(now) || !count_[0] || !count_[1]) return false;
    result.source=std::min(samples_[0][count_[0]-1].source,samples_[1][count_[1]-1].source);
    for(unsigned axis=0;axis<2;++axis) {
      result.age[axis]=now-samples_[axis][count_[axis]-1].source;
      if(result.age[axis]<0 || result.age[axis]>=.060-1e-12) return false; // Original 60 ms boundary.
      bool found=false;
      for(unsigned i=0;i<count_[axis];++i) {
        const auto& right=samples_[axis][i];
        if(right.source==result.source) {
          result.position[axis]=right.position;result.velocity[axis]=right.velocity;
          result.bracket[axis]=0;result.left[axis]=right;result.right[axis]=right;found=true;break;
        }
        if(right.source>result.source && i>0) {
          const auto& left=samples_[axis][i-1];
          const double w=double(result.source-left.source)/(right.source-left.source);
          if(w<0. || w>1.) return false;
          result.position[axis]=left.position+w*(right.position-left.position);
          result.velocity[axis]=left.velocity+w*(right.velocity-left.velocity);
          result.bracket[axis]=right.source-left.source;
          result.left[axis]=left;result.right[axis]=right;
          found=true;break;
        }
      }
      if(!found) return false;
    }
    return true;
  }
  unsigned discarded() const { return discarded_; }
 private:
  static constexpr unsigned capacity=16;
  std::array<std::array<Sample,capacity>,2> samples_{};
  std::array<unsigned,2> count_{};
  unsigned discarded_{};
};
}
