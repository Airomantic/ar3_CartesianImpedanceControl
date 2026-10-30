#pragma once
#include "ar3_cartesian_impedance/command.hpp"
#include <iomanip>
#include <sstream>

namespace ar3_cartesian_impedance {

// Keep the SDK's six raw channels intact: their physical component mapping
// has not yet been validated. Never silently subtract a startup offset.
struct WrenchViolation {
  std::size_t channel{6};
  Six raw{}, limits{};
  explicit operator bool() const noexcept { return channel < 6; }
};
inline WrenchViolation check_wrench_limits(const Six &raw, const Six &limits) noexcept {
  for(std::size_t i = 0; i < raw.size(); ++i)
    if(!std::isfinite(raw[i]) || !std::isfinite(limits[i]) || limits[i] < 0 || std::abs(raw[i]) > limits[i])
      return {i, raw, limits};
  return {};
}

// Diagnostics are formatted only outside the 1 ms callback.
inline std::string describe_wrench_violation(const WrenchViolation &violation) {
  if(!violation) return {};
  std::ostringstream out;
  out << std::fixed << std::setprecision(3)
      << "tau_ext_base raw channel " << violation.channel + 1
      << "=" << violation.raw[violation.channel]
      << " exceeds abs limit=" << violation.limits[violation.channel] << "; raw=[";
  for(std::size_t i = 0; i < 6; ++i) out << (i ? "," : "") << violation.raw[i];
  out << "]; limits=[";
  for(std::size_t i = 0; i < 6; ++i) out << (i ? "," : "") << violation.limits[i];
  out << "]; component units/origin unverified; no bias removed";
  return out.str();
}
inline std::string describe_wrench_violation(const WrenchViolation &violation, const Six &measured_pose) {
  if(!violation) return {};
  std::ostringstream out;
  out << describe_wrench_violation(violation) << std::fixed << std::setprecision(5) << "; measured TCP/base=[";
  for(std::size_t i = 0; i < 6; ++i) out << (i ? "," : "") << measured_pose[i];
  out << "] (xyz[m],rpy[rad])";
  return out.str();
}
}  // namespace ar3_cartesian_impedance
