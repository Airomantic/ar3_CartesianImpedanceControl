#pragma once
#include <array>
#include <cmath>
#include <limits>

namespace ar3_cartesian_impedance {

// SDK tau_m is joint torque [Nm], in SDK joint order 1..7. It is
// independent of the six-component tau_ext_base Cartesian feedback.
struct JointTorqueSample {
  std::array<double, 7> values{};
  bool valid{};
};

// Read the existing RT packet, never a new hardware connection or RPC.
// A fresh NaN buffer also rejects an incomplete SDK write. The exact array7
// type lets getStateData reject a packet whose field shape/type differs.
// Optional recording telemetry must not change the motion control path.
template<class Read> JointTorqueSample read_joint_torque(Read read) noexcept {
  JointTorqueSample result;
  result.values.fill(std::numeric_limits<double>::quiet_NaN());
  try {
    if(read(result.values) != 0) return result;
  } catch(...) {
    return result;
  }
  for(double value : result.values) if(!std::isfinite(value)) return result;
  result.valid = true;
  return result;
}
}  // namespace ar3_cartesian_impedance
