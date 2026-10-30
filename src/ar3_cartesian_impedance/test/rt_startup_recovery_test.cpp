#include "ar3_cartesian_impedance/rt_startup_recovery.hpp"
#include <iostream>
#include <limits>

using namespace ar3_cartesian_impedance;
namespace {
void require(bool ok, const char *message) { if(!ok) throw std::runtime_error(message); }
template<class F> void rejects(F action, const char *reason = nullptr) {
  try { action(); } catch(const std::exception &error) {
    if(reason) require(std::string(error.what()).find(reason) != std::string::npos,
      "startup recovery rejected for an unexpected reason");
    return;
  }
  throw std::runtime_error("unsafe startup recovery observation accepted");
}
constexpr std::int64_t second = 1000000000;
constexpr std::int64_t step = second / 20;
using Joints = RtStartupRecoveryCheck::Joints;

void stationary_rci_requires_time_and_sample_count() {
  Joints position{0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7}, velocity{};
  RtStartupRecoveryCheck check(0, 3.0);
  for(int index = 0; index < 20; ++index)
    require(check.observe(2, 0, position, velocity, index * step) == StartupRecoveryDecision::observe,
      "recovery allowed before one second of stationary observations");
  require(check.observe(2, 0, position, velocity, second) == StartupRecoveryDecision::exit_rt_mode,
    "stationary rci main state could not request a single Idle transition");
  RtStartupRecoveryCheck too_few(0, 3.0);
  for(int index = 0; index < 19; ++index)
    require(too_few.observe(2, 1, position, velocity, index * 60000000) == StartupRecoveryDecision::observe,
      "fewer than twenty readings were mistaken for a sufficiently observed stationary interval");
}

void stationary_encoder_jitter_can_pass_without_loosening_position_limits() {
  // Reproduce the observed scale: <=2.4 microrad joint position range,
  // velocity quantization of 0.001047198 rad/s and spikes of four quanta.
  // This is regression input, not a claim about the SDK's specified precision.
  constexpr double quantum = 0.001047198;
  constexpr std::array<int, 10> noise{-4, 1, 0, 1, 0, 1, 0, 1, 0, 0};
  RtStartupRecoveryCheck check(0, 3.0); Joints position{}, velocity{};
  for(int index = 0; index <= 20; ++index) {
    for(std::size_t axis = 0; axis < 7; ++axis) {
      position[axis] = 0.1 * axis + 0.0000006 * ((index + axis) % 5);
      velocity[axis] = quantum * noise[(index + axis) % noise.size()];
    }
    require(check.observe(2, 0, position, velocity, index * step) ==
      (index < 20 ? StartupRecoveryDecision::observe : StartupRecoveryDecision::exit_rt_mode),
      "observed stationary position/velocity jitter was mistaken for sustained motion");
  }
}

void persistent_velocity_and_large_alternating_velocity_are_rejected() {
  for(std::size_t axis = 0; axis < 7; ++axis) {
    for(double sign : {-1.0, 1.0}) {
      RtStartupRecoveryCheck check(0, 3.0); Joints position{}, velocity{};
      velocity[axis] = sign * 0.001047198;
      // The position read may be stale: a stable q alone is insufficient.
      for(int index = 0; index < 20; ++index)
        check.observe(2, 0, position, velocity, index * step);
      rejects([&] { check.observe(2, 0, position, velocity, second); }, "sustained joint velocity");
    }
    RtStartupRecoveryCheck alternating(0, 3.0); Joints position{}, velocity{};
    for(int index = 0; index < 20; ++index) {
      velocity[axis] = (index % 2 ? -1 : 1) * 0.002094396;
      alternating.observe(2, 0, position, velocity, index * step);
    }
    velocity[axis] = 0.002094396;
    rejects([&] { alternating.observe(2, 0, position, velocity, second); }, "excessive joint velocity variation");
  }
}

void motion_envelope_spike_and_invalid_samples_reject_immediately() {
  for(std::size_t axis = 0; axis < 7; ++axis) {
    for(double sign : {-1.0, 1.0}) {
      RtStartupRecoveryCheck speed(0, 3.0); Joints position{}, velocity{};
      velocity[axis] = sign * 0.010001;
      rejects([&] { speed.observe(2, 0, position, velocity, 0); }, "instantaneous velocity");
      RtStartupRecoveryCheck drift(0, 3.0);
      velocity.fill(0); drift.observe(2, 0, position, velocity, 0);
      position[axis] = sign * 0.000021;
      rejects([&] { drift.observe(2, 0, position, velocity, step); }, "motion envelope");
    }
    for(bool bad_position : {false, true}) {
      for(double invalid_value : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        RtStartupRecoveryCheck invalid(0, 3.0); Joints position{}, velocity{};
        (bad_position ? position : velocity)[axis] = invalid_value;
        rejects([&] { invalid.observe(2, 0, position, velocity, 0); }, "nonfinite joint sample");
      }
    }
    // An out-and-back path must fail even if its endpoint or each offset
    // from the initial q is small. Check the full peak-to-peak envelope.
    RtStartupRecoveryCheck out_and_back(0, 3.0); Joints position{}, velocity{};
    out_and_back.observe(2, 0, position, velocity, 0);
    position[axis] = -0.000015;
    out_and_back.observe(2, 0, position, velocity, step);
    position[axis] = 0.000015;
    rejects([&] { out_and_back.observe(2, 0, position, velocity, 2 * step); }, "motion envelope");
  }
  // Compare the full window, not neighboring samples: drift is not allowed
  // to become a moving baseline even if the velocity read happens to be zero.
  RtStartupRecoveryCheck cumulative(0, 3.0); Joints position{}, velocity{};
  for(int index = 0; index < 10; ++index) {
    position[6] = index * 0.000002;
    cumulative.observe(2, 0, position, velocity, index * step);
  }
  position[6] = 0.000022;
  rejects([&] { cumulative.observe(2, 0, position, velocity, 10 * step); }, "motion envelope");
}

void timestamps_and_sample_density_cannot_dilute_motion() {
  const Joints position{}, velocity{};
  for(std::int64_t invalid_ns : {second / 2, second / 2 - 1}) {
    RtStartupRecoveryCheck check(0, 3.0);
    check.observe(2, 0, position, velocity, second / 2);
    rejects([&] { check.observe(2, 0, position, velocity, invalid_ns); }, "timestamps must strictly increase");
  }
  RtStartupRecoveryCheck gap(0, 3.0);
  gap.observe(2, 0, position, velocity, 0);
  rejects([&] { gap.observe(2, 0, position, velocity, 250000001); }, "observation gap");

  RtStartupRecoveryCheck burst(0, 3.0); Joints moving{};
  moving[0] = 0.0015;
  for(int index = 0; index <= 4; ++index)
    burst.observe(2, 0, position, moving, index * second / 5);
  // Many zero readings near the end would dilute an arithmetic sample mean.
  // They cannot erase the measured 0.8 seconds of positive velocity.
  for(int index = 0; index < 20; ++index)
    burst.observe(2, 0, position, velocity, 800000001 + index);
  rejects([&] { burst.observe(2, 0, position, velocity, second); }, "sustained joint velocity");
}

void mode_power_and_deadline_fail_closed() {
  const Joints position{}, velocity{};
  for(int operation : {-1, 1, 3, 4, 9, 10}) {
    RtStartupRecoveryCheck check(0, 3.0);
    rejects([&] { check.observe(operation, 0, position, velocity, 0); }, "operationState changed");
  }
  for(int power : {-1, 2, 3, 42}) {
    RtStartupRecoveryCheck check(0, 3.0);
    rejects([&] { check.observe(2, power, position, velocity, 0); }, "power state");
    rejects([&] { check.observe(0, power, position, velocity, 0); }, "power state");
  }
  RtStartupRecoveryCheck idle(0, 3.0);
  require(idle.observe(0, 1, position, velocity, 0) == StartupRecoveryDecision::already_idle,
    "already-idle controller unnecessarily requested another mode transition");
  RtStartupRecoveryCheck timeout(0, 3.0);
  rejects([&] { timeout.observe(2, 0, position, velocity, 3 * second + 1); }, "timed out");
  rejects([&] { timeout.check_deadline(-1); }, "timed out");
  RtStartupRecoveryCheck short_timeout(0, 0.5);
  short_timeout.observe(2, 0, position, velocity, 0);
  rejects([&] { short_timeout.observe(2, 0, position, velocity, second); }, "timed out");
  rejects([&] { RtStartupRecoveryCheck invalid(0, 3.1); }, "invalid startup recovery observation deadline");
}
}  // namespace

int main() {
  stationary_rci_requires_time_and_sample_count();
  stationary_encoder_jitter_can_pass_without_loosening_position_limits();
  persistent_velocity_and_large_alternating_velocity_are_rejected();
  motion_envelope_spike_and_invalid_samples_reject_immediately();
  timestamps_and_sample_density_cannot_dilute_motion();
  mode_power_and_deadline_fail_closed();
  std::cout << "RT startup recovery position envelope, measured jitter, sustained velocity, sample timing and fail-closed regressions passed\n";
}
