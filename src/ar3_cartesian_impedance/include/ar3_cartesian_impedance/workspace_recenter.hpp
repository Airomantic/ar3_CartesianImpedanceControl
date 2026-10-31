#pragma once
#include "ar3_cartesian_impedance/command.hpp"
#include <cstdint>

namespace ar3_cartesian_impedance {
struct WorkspaceCenter { std::uint64_t session{}; Six pose{}; };

// RT-owned, allocation-free gate. Only one recenter is allowed per session;
// a stationary, zero-wrench hold must be observed for 300 ms before accepting.
class WorkspaceRecenterGate {
 public:
  void reset() { *this = WorkspaceRecenterGate{}; }
  bool observe(const Six &measured, const Command &target, const Six &output, std::int64_t now) {
    if(done_) return false;
    if(std::any_of(target.wrench.begin(), target.wrench.end(), [](double x) { return x != 0; }) ||
       !near(measured, target.pose, .02, .08) || !near(output, target.pose, .001, .01)) {
      since_ = 0; return false;
    }
    if(!since_ || now < since_ || !near(measured, measured_anchor_, .001, .01) ||
       !near(target.pose, target_anchor_, 1e-6, 1e-6)) {
      since_ = now; measured_anchor_ = measured; target_anchor_ = target.pose; return false;
    }
    if(now - since_ < 300000000) return false;
    done_ = true; return true;
  }
 private:
  static bool near(const Six &a, const Six &b, double distance, double angle) {
    double squared = 0;
    for(int i=0; i<6; ++i) if(!std::isfinite(a[i]) || !std::isfinite(b[i])) return false;
    for(int i=0; i<3; ++i) squared += (a[i]-b[i])*(a[i]-b[i]);
    return squared <= distance*distance && orientation(a).angularDistance(orientation(b)) <= angle;
  }
  bool done_{};
  std::int64_t since_{};
  Six measured_anchor_{}, target_anchor_{};
};
}  // namespace ar3_cartesian_impedance
