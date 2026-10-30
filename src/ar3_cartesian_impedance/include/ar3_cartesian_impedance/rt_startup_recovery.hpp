#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ar3_cartesian_impedance {

enum class StartupRecoveryDecision { observe, exit_rt_mode, already_idle };

// This checks stationary evidence, not control ownership. It is used only
// after explicit opt-in for a deployment whose exclusive control is confirmed.
// Integer state values match the public xCore SDK enums (asserted by the node).
// These are diagnostic limits, not SDK precision or a safety-rated motion test.
// On this controller a stationary 2s trace had <=2.4 microrad position range,
// but instantaneous velocity jitter in 0.001047198 rad/s steps up to 0.004189.
// Bound the whole position envelope tightly and check velocity over time;
// raising only the instantaneous velocity threshold would mask slow motion.
class RtStartupRecoveryCheck {
 public:
  using Joints = std::array<double, 7>;
  static constexpr double max_position_range_rad = 0.00002;
  static constexpr double max_instant_velocity_rad_s = 0.01;
  static constexpr double max_mean_velocity_rad_s = 0.001;
  static constexpr double max_mean_abs_velocity_rad_s = 0.002;
  static constexpr unsigned minimum_samples = 20;
  static constexpr std::int64_t minimum_window_ns = 1000000000;
  static constexpr std::int64_t max_sample_gap_ns = 250000000;
  RtStartupRecoveryCheck(std::int64_t start_ns, double timeout_s)
      : start_ns_(start_ns), timeout_s_(timeout_s) {
    if(start_ns < 0 || !std::isfinite(timeout_s) || timeout_s <= 0 || timeout_s > 3.0)
      throw std::invalid_argument("invalid startup recovery observation deadline");
  }

  StartupRecoveryDecision observe(int operation, int power, const Joints &position,
      const Joints &velocity, std::int64_t now_ns) {
    check_deadline(now_ns);
    if(power != 0 && power != 1)
      throw std::runtime_error("RT startup recovery refused: power state=" + std::to_string(power) + " (estop/gstop/unknown)");
    if(operation == 0) return StartupRecoveryDecision::already_idle;
    if(operation != 2)
      throw std::runtime_error("RT startup recovery refused: operationState changed to " + std::to_string(operation));
    if(samples_ && now_ns <= last_sample_ns_)
      throw std::runtime_error("RT startup recovery refused: joint sample timestamps must strictly increase");
    if(samples_ && now_ns - last_sample_ns_ > max_sample_gap_ns)
      throw std::runtime_error("RT startup recovery refused: joint observation gap exceeds 0.25s");
    for(std::size_t axis = 0; axis < position.size(); ++axis) {
      if(!std::isfinite(position[axis]) || !std::isfinite(velocity[axis]))
        throw std::runtime_error("RT startup recovery refused: nonfinite joint sample, axis=" + std::to_string(axis + 1));
      if(std::abs(velocity[axis]) > max_instant_velocity_rad_s)
        throw std::runtime_error("RT startup recovery refused: motion detected, axis=" + std::to_string(axis + 1) +
          " instantaneous velocity=" + std::to_string(velocity[axis]) + " rad/s exceeds 0.01");
      if(samples_ && std::max(position[axis], max_position_[axis]) - std::min(position[axis], min_position_[axis]) > max_position_range_rad)
        throw std::runtime_error("RT startup recovery refused: joint motion envelope exceeded, axis=" + std::to_string(axis + 1) +
          " range=" + std::to_string(std::max(position[axis], max_position_[axis]) - std::min(position[axis], min_position_[axis])) +
          " rad exceeds 0.000020");
    }
    if(!samples_) {
      min_position_ = max_position_ = position;
      first_sample_ns_ = now_ns;
    } else {
      const double dt_s = (now_ns - last_sample_ns_) * 1e-9;
      for(std::size_t axis = 0; axis < position.size(); ++axis) {
        min_position_[axis] = std::min(position[axis], min_position_[axis]);
        max_position_[axis] = std::max(position[axis], max_position_[axis]);
        // Trapezoidal time weighting prevents a burst of near-identical host
        // timestamps from diluting sustained velocity between sparse samples.
        velocity_integral_[axis] += 0.5 * (last_velocity_[axis] + velocity[axis]) * dt_s;
        abs_velocity_integral_[axis] += 0.5 * (std::abs(last_velocity_[axis]) + std::abs(velocity[axis])) * dt_s;
      }
    }
    last_velocity_ = velocity;
    last_sample_ns_ = now_ns;
    ++samples_;
    if(samples_ < minimum_samples || now_ns - first_sample_ns_ < minimum_window_ns)
      return StartupRecoveryDecision::observe;
    const double observed_s = (now_ns - first_sample_ns_) * 1e-9;
    for(std::size_t axis = 0; axis < position.size(); ++axis) {
      if(std::abs(velocity_integral_[axis]) / observed_s > max_mean_velocity_rad_s)
        throw std::runtime_error("RT startup recovery refused: sustained joint velocity, axis=" + std::to_string(axis + 1) +
          " time-weighted mean=" + std::to_string(velocity_integral_[axis] / observed_s) + " rad/s exceeds +/-0.001");
      if(abs_velocity_integral_[axis] / observed_s > max_mean_abs_velocity_rad_s)
        throw std::runtime_error("RT startup recovery refused: excessive joint velocity variation, axis=" + std::to_string(axis + 1) +
          " time-weighted absolute mean=" + std::to_string(abs_velocity_integral_[axis] / observed_s) + " rad/s exceeds 0.002");
    }
    return StartupRecoveryDecision::exit_rt_mode;
  }

  void check_deadline(std::int64_t now_ns) const {
    if(now_ns < start_ns_ || (now_ns - start_ns_) * 1e-9 > timeout_s_)
      throw std::runtime_error("RT startup recovery observation/idle confirmation timed out");
  }

 private:
  std::int64_t start_ns_{}, first_sample_ns_{}, last_sample_ns_{};
  double timeout_s_{};
  unsigned samples_{};
  Joints min_position_{}, max_position_{}, last_velocity_{}, velocity_integral_{}, abs_velocity_integral_{};
};

}  // namespace ar3_cartesian_impedance
