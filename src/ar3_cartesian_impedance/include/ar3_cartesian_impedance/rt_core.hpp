#pragma once
#include "ar3_cartesian_impedance/frames.hpp"
#include "ar3_cartesian_impedance/joint_torque.hpp"
#include <atomic>
#include <cstdint>
#include <type_traits>

namespace ar3_cartesian_impedance {

// Exactly one producer and one consumer. Ownership of three fixed buffers is
// exchanged atomically, so neither thread waits or reads a concurrently written
// object. The consumer gets the newest publication, never a queue of old goals.
template<class T> class LatestMailbox {
 public:
  static_assert(std::is_trivially_copyable_v<T>);
  static_assert(std::atomic<unsigned>::is_always_lock_free);
  void publish(const T &value) noexcept {
    buffers_[back_] = value;
    back_ = middle_.exchange(back_ | dirty, std::memory_order_acq_rel) & mask;
  }
  bool read(T &value) noexcept {
    if(!(middle_.load(std::memory_order_acquire) & dirty)) return false;
    front_ = middle_.exchange(front_, std::memory_order_acq_rel) & mask;
    value = buffers_[front_];
    return true;
  }
 private:
  static constexpr unsigned dirty = 4, mask = 3;
  T buffers_[3]{};
  unsigned back_{0}, front_{1};
  std::atomic<unsigned> middle_{2};
};

using Matrix16 = std::array<double, 16>;
template<class Pull> unsigned drain_rt_packets(Pull pull, unsigned budget) {
  unsigned packets = 0;
  while(packets < budget && pull(packets == 0)) ++packets;
  if(packets == budget) throw std::runtime_error("RT state backlog exceeded bounded drain");
  return packets;
}
inline Matrix16 pose_matrix(const Six &pose) {
  const auto matrix = pose_transform(pose).matrix();
  Matrix16 result{};
  for(int row = 0; row < 4; ++row)
    for(int col = 0; col < 4; ++col) result[row * 4 + col] = matrix(row, col);
  return result;
}
inline Six matrix_pose(const Matrix16 &matrix) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  for(int row = 0; row < 4; ++row)
    for(int col = 0; col < 4; ++col) {
      const double value = matrix[row * 4 + col];
      if(!std::isfinite(value)) throw std::invalid_argument("nonfinite RT TCP matrix");
      transform.matrix()(row, col) = value;
    }
  if(!transform.linear().transpose().isApprox(transform.linear().inverse(), 1e-6) ||
     std::abs(transform.linear().determinant() - 1) > 1e-6 ||
     std::abs(matrix[12]) + std::abs(matrix[13]) + std::abs(matrix[14]) + std::abs(matrix[15] - 1) > 1e-6)
    throw std::invalid_argument("invalid RT TCP rigid transform");
  return transform_pose(transform);
}

inline Six stiffness_in_world(const Six &base_stiffness, const Six &world_base) {
  const auto rotation = orientation(world_base).toRotationMatrix();
  Six result{};
  for(int offset : {0, 3}) {
    const Eigen::Vector3d k(base_stiffness[offset], base_stiffness[offset + 1], base_stiffness[offset + 2]);
    const Eigen::Matrix3d world = rotation * k.asDiagonal() * rotation.transpose();
    const Eigen::Matrix3d diagonal = world.diagonal().asDiagonal();
    if((world - diagonal).cwiseAbs().maxCoeff() > 1e-8 * std::max(1.0, k.cwiseAbs().maxCoeff()))
      throw std::invalid_argument("base stiffness produces off-diagonal world stiffness unsupported by RT SDK");
    for(int axis = 0; axis < 3; ++axis) result[offset + axis] = world(axis, axis);
  }
  return result;
}

inline bool same_session_parameters(const Command &a, const Command &b) noexcept {
  return a.stiffness == b.stiffness && a.wrench == b.wrench && a.damping == b.damping && a.mass == b.mass;
}
inline bool obsolete_session_status(std::uint64_t status_session, std::uint64_t requested_session, bool newly_armed) noexcept {
  return newly_armed && status_session < requested_session;
}

// A bounded velocity and acceleration follower, evaluated at the SDK's fixed
// 1 ms command interval. Wall-time overruns never become a larger pose jump.
class RtPoseFollower {
 public:
  void reset(const Six &pose) {
    position_ = {pose[0], pose[1], pose[2]};
    rotation_ = orientation(pose);
    velocity_.setZero(); angular_velocity_.setZero();
  }
  Six step(const Six &target, double speed, double angular_speed, double acceleration, double angular_acceleration) {
    constexpr double dt = 0.001;
    const Eigen::Vector3d distance = Eigen::Vector3d(target[0], target[1], target[2]) - position_;
    const auto delta = step_vector(distance, velocity_, speed, acceleration, dt);
    position_ += delta;
    Eigen::Quaterniond error = orientation(target) * rotation_.conjugate();
    if(error.w() < 0) error.coeffs() *= -1;
    const Eigen::AngleAxisd angle(error);
    const auto angular_step = step_vector(angle.axis() * angle.angle(), angular_velocity_, angular_speed, angular_acceleration, dt);
    if(angular_step.norm() > 1e-15)
      rotation_ = (Eigen::Quaterniond(Eigen::AngleAxisd(angular_step.norm(), angular_step.normalized())) * rotation_).normalized();
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = position_;
    transform.linear() = rotation_.toRotationMatrix();
    return transform_pose(transform);
  }
 private:
  static Eigen::Vector3d limit(Eigen::Vector3d vector, double maximum) {
    const double magnitude = vector.norm();
    if(magnitude > maximum) vector *= maximum / magnitude;
    return vector;
  }
  static Eigen::Vector3d step_vector(const Eigen::Vector3d &error, Eigen::Vector3d &velocity,
      double speed, double acceleration, double dt) {
    const auto wanted = limit(error * 8.0, speed);
    velocity += limit(wanted - velocity, acceleration * dt);
    // A newly moved goal can be closer than the stopping distance. Brake at
    // the acceleration limit instead of snapping to it or clearing velocity.
    return velocity * dt;
  }
  Eigen::Vector3d position_{Eigen::Vector3d::Zero()}, velocity_{Eigen::Vector3d::Zero()}, angular_velocity_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond rotation_{Eigen::Quaterniond::Identity()};
};

struct RtCommand { Command value{}; std::int64_t received_ns{}; std::uint64_t session{}; };
enum class RtExit { none, requested, command_timeout, state_timeout, parameter_change, safety_limit, invalid_state };
inline RtExit rt_session_gate(const RtCommand &command, const Command &configured, std::uint64_t session,
    std::int64_t measured_ns, std::int64_t now_ns, double command_timeout_s, double state_timeout_s, bool stop_requested) {
  if(stop_requested) return RtExit::requested;
  if(command.session != session || command.received_ns <= 0 || (now_ns - command.received_ns) * 1e-9 > command_timeout_s)
    return RtExit::command_timeout;
  if(measured_ns <= 0 || (now_ns - measured_ns) * 1e-9 > state_timeout_s) return RtExit::state_timeout;
  if(!same_session_parameters(command.value, configured)) return RtExit::parameter_change;
  return RtExit::none;
}

class RtStopProgress {
 public:
  void reset_for_session() noexcept {
    explicit_stop_required_ = false;
    confirmation_started_ = false;
  }
  void start_attempted() noexcept { explicit_stop_required_ = true; }
  void loop_returned_normally(bool callback_finished) noexcept {
    if(callback_finished) explicit_stop_required_ = false;
  }
  void stop_acknowledged() noexcept { explicit_stop_required_ = false; }
  bool needs_stop_rpc() const noexcept { return explicit_stop_required_; }
  bool begin_confirmation() noexcept {
    if(confirmation_started_) return false;
    confirmation_started_ = true;
    return true;
  }
 private:
  bool explicit_stop_required_{false};
  bool confirmation_started_{false};
};

// stopMove/setFinished end a motion; they do not leave the controller's RCI
// main state. operationState strips the substate after '/' and maps 'rci' to
// rtControlling, so confirmation must follow an explicit exit from RT mode.
// Keep the sequence shared with the fake-device lifecycle regression.
template<class StopMotion, class SuspendMonitor, class LeaveMode, class ConfirmIdle>
void finish_rt_session(bool explicit_stop, StopMotion stop_motion, SuspendMonitor suspend_monitor,
    LeaveMode leave_mode, ConfirmIdle confirm_idle) {
  if(explicit_stop) stop_motion();
  suspend_monitor();
  leave_mode();
  confirm_idle();
}
struct RtSnapshot {
  Command measured{}, accepted{};
  JointTorqueSample joint_torque{};
  std::int64_t measured_ns{}, accepted_ns{};
  std::uint64_t sequence{};
  double elbow{};
  bool accepted_valid{};
};
}  // namespace ar3_cartesian_impedance
