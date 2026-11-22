
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace ph_exo::control {
// Source times are monotonic seconds, never rounded onto the control grid.
// This is host acceptance history; it is not a drive execution acknowledgement.
class EffortHistory {
 public:
  struct Sample {double time{},effort{};std::uint64_t sequence{};};
  void reset(double epoch) noexcept {
    count_={};valid_=std::isfinite(epoch)&&epoch>0.;
    if(valid_) for(unsigned i=0;i<2;++i) {
      samples_[i][0]={epoch,0.,0};count_[i]=1;
    }
  }
  bool push(unsigned axis,const Sample& value) noexcept {
    if(!valid_||axis>1||!std::isfinite(value.time)||!std::isfinite(value.effort)||
       value.sequence==0||std::abs(value.effort)>10.+1e-9) return false;
    auto& n=count_[axis];const auto& last=samples_[axis][n-1];
    if(value.sequence==last.sequence)
      return value.time==last.time && value.effort==last.effort;
    if(value.sequence<=last.sequence||value.time<last.time) return false;
    if(last.sequence && value.sequence!=last.sequence+1) return false;
    if(n==capacity) {for(unsigned i=1;i<n;++i)samples_[axis][i-1]=samples_[axis][i];--n;}
    samples_[axis][n++]=value;return true;
  }
  bool held(unsigned axis,double time,double& result) const noexcept {
    if(!valid_||axis>1||!std::isfinite(time)||time<samples_[axis][0].time) return false;
    result=samples_[axis][0].effort;
    for(unsigned i=1;i<count_[axis] && samples_[axis][i].time<=time;++i)
      result=samples_[axis][i].effort;
    return true;
  }
  bool integral(unsigned axis,double begin,double end,double& result) const noexcept {
    result=0.;
    double value{};
    if(!std::isfinite(end)||end<begin||!held(axis,begin,value)) return false;
    double cursor=begin;
    for(unsigned i=1;i<count_[axis];++i) {
      const auto& s=samples_[axis][i];
      if(s.time<=begin)continue;
      if(s.time>=end)break;
      result+=(s.time-cursor)*value;cursor=s.time;value=s.effort;
    }
    result+=(end-cursor)*value;
    return std::isfinite(result);
  }
  double newest_time() const noexcept {
    if(!valid_)return -1.;
    return std::max(samples_[0][count_[0]-1].time,samples_[1][count_[1]-1].time);
  }
  double next_change(double after,double until) const noexcept {
    double next=until;
    for(unsigned axis=0;axis<2;++axis)
      for(unsigned i=0;i<count_[axis];++i) {
        const double t=samples_[axis][i].time;
        if(t>after && t<next)next=t;
      }
    return next;
  }
  template<class Predictor>
  bool replay(Predictor& predictor,double begin,double end) const noexcept {
    if(!std::isfinite(begin)||!std::isfinite(end)||end<begin||end-begin>.32) return false;
    double cursor=begin;
    for(unsigned iteration=0;cursor<end && iteration<192;++iteration) {
      const double next=std::min(next_change(cursor,end),cursor+.005);
      std::array<double,2> effort{};
      if(next<=cursor||!held(0,cursor,effort[0])||!held(1,cursor,effort[1])||
         !predictor.advance(effort,next-cursor))return false;
      cursor=next;
    }
    return cursor==end;
  }
 private:
  static constexpr unsigned capacity=64;
  std::array<std::array<Sample,capacity>,2> samples_{};
  std::array<unsigned,2> count_{};
  bool valid_{};
};
}
