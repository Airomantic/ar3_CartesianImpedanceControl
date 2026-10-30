#include "ar3_cartesian_impedance/rt_core.hpp"
#include "ar3_cartesian_impedance/rt_startup_recovery.hpp"
#include "ar3_cartesian_impedance/load.hpp"
#include "ar3_cartesian_impedance/wrench_guard.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rokae/robot.h>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <memory>
#include <sstream>
#include <sys/file.h>
#include <thread>
#include <unistd.h>

namespace ar3_cartesian_impedance {
using Clock = std::chrono::steady_clock;
using ArrayMsg = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
static std::int64_t monotonic_ns() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
static double age_s(std::int64_t stamp) { return (monotonic_ns() - stamp) * 1e-9; }

class RobotProcessLock {
 public:
  explicit RobotProcessLock(std::string ip) {
    for(char &c : ip) if(!std::isalnum(static_cast<unsigned char>(c)) && c != '.') c = '_';
    const auto path = "/tmp/ar3_cartesian_impedance_" + ip + ".lock";
    fd_ = ::open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    if(fd_ < 0) throw std::runtime_error("cannot open robot process lock");
    if(::flock(fd_, LOCK_EX | LOCK_NB) < 0) {
      ::close(fd_); fd_ = -1;
      throw std::runtime_error("another Cartesian impedance node owns robot " + ip);
    }
  }
  ~RobotProcessLock() { if(fd_ >= 0) ::close(fd_); }
 private:
  int fd_{-1};
};

enum class Phase { offline, connecting, monitoring, armed, initializing, active, stopping, stopped, stop_failed, fault };
using LoopExit = RtExit;
struct StatusSample { Phase phase{Phase::connecting}; std::uint64_t session{}; std::array<char, 1024> detail{}; };

class CartesianImpedanceNode final : public rclcpp::Node {
 public:
  CartesianImpedanceNode() : Node("cartesian_impedance_node") {
    robot_ip_ = parameter<std::string>("robot_ip", "192.168.0.160");
    local_ip_ = parameter<std::string>("local_ip", "");
    frame_id_ = parameter<std::string>("frame_id", "AR3-3_07R-W4C5C5_base");
    connect_ = parameter("connect_robot", true);
    allow_motion_ = parameter("enable_motion", false);
    auto_power_ = parameter("auto_power_on", false);
    recover_stale_rt_ = parameter("recover_stale_rt_on_startup", false);
    override_enabled_ = parameter("load_override_enabled", false);
    override_load_ = payload_load(parameter("load_mass_kg", 1.1),
      parameter("load_cog_m", std::vector<double>{0, 0.008, 0.091}),
      parameter("load_inertia_kg_m2", std::vector<double>{0, 0, 0}));
    timeout_s_ = positive("command_timeout_s", 0.5);
    state_timeout_s_ = positive("rt_state_timeout_s", 0.01);
    startup_timeout_s_ = positive("startup_timeout_s", 5.0);
    stop_timeout_s_ = positive("stop_timeout_s", 3.0);
    max_step_m_ = positive("max_translation_step_m", 0.03);
    max_step_rad_ = positive("max_rotation_step_rad", 0.15);
    radius_ = positive("workspace_radius_m", 0.25);
    speed_m_s_ = positive("speed_mm_s", 20.0) / 1000.0;
    acceleration_ = positive("linear_acceleration_m_s2", 0.1);
    angular_acceleration_ = positive("angular_acceleration_rad_s2", 0.5);
    const double publish_rate = positive("publish_rate_hz", 50.0);
    priority_ = parameter("rt_priority", 0);
    cpu_ = parameter("rt_cpu", -1);
    network_tolerance_ = parameter("network_tolerance_percent", 20);
    stiffness_ = six_parameter("stiffness", {500, 500, 500, 50, 50, 50});
    damping_ = six_parameter("damping_metadata", {});
    mass_ = six_parameter("mass_metadata", {});
    max_stiffness_ = six_parameter("max_stiffness", {1000, 1000, 1000, 100, 100, 100});
    max_wrench_ = six_parameter("max_wrench", {20, 20, 20, 3, 3, 3});
    max_measured_wrench_ = six_parameter("max_measured_wrench", {40, 40, 40, 6, 6, 6});
    const auto velocity = six_parameter("max_velocity", {0.03, 0.03, 0.03, 0.15, 0.15, 0.15});
    angular_speed_ = std::min({velocity[3], velocity[4], velocity[5]});
    speed_m_s_ = std::min({speed_m_s_, velocity[0], velocity[1], velocity[2]});
    if(connect_ && local_ip_.empty()) throw std::invalid_argument("RT requires explicit local_ip on the robot network");
    if(publish_rate > 100 || speed_m_s_ <= 0 || speed_m_s_ > 0.1 || angular_speed_ <= 0 || angular_speed_ > 0.5 ||
       max_step_m_ > 0.1 || max_step_rad_ > 0.5 || network_tolerance_ < 0 || network_tolerance_ > 100 ||
       priority_ < 0 || priority_ > 99 || cpu_ < -1 || state_timeout_s_ > 0.1)
      throw std::invalid_argument("invalid RT rate/speed/step/network/thread parameters");
    for(std::size_t i = 0; i < 6; ++i) {
      if(damping_[i] != 0 || mass_[i] != 0) throw std::invalid_argument("D/M must be zero metadata: neither is applied in this RT implementation");
      if(stiffness_[i] > max_stiffness_[i] || max_stiffness_[i] > (i < 3 ? 3000 : 300) ||
         max_wrench_[i] > (i < 3 ? 60 : 30)) throw std::invalid_argument("stiffness/wrench exceeds SDK limits");
    }
    state_pub_ = create_publisher<ArrayMsg>(topic("state"), 1);
    accepted_pub_ = create_publisher<ArrayMsg>(topic("accepted_command"), 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(topic("tcp_pose"), 1);
    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic("measured_wrench"), 1);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>(topic("ready"), rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(topic("status"), rclcpp::QoS(1).transient_local());
    metadata_pub_ = create_publisher<std_msgs::msg::String>(topic("metadata"), rclcpp::QoS(1).transient_local());
    publish_metadata();
    command_sub_ = create_subscription<ArrayMsg>(topic("command"), rclcpp::QoS(1), [this](ArrayMsg::ConstSharedPtr message) {
      if(!user_armed_ || stop_requested_.load()) return;
      try {
        const auto command = decode(message->data, damping_, mass_, max_wrench_, max_stiffness_);
        consume_snapshot();
        if(!latest_.measured_ns || age_s(latest_.measured_ns) > timeout_s_) throw std::runtime_error("no fresh measured TCP");
        validate_target(latest_.measured.pose, command.pose, arm_origin_, max_step_m_, max_step_rad_, radius_);
        commands_.publish({command, monotonic_ns(), requested_session_.load()});
      } catch(const std::exception &error) {
        RCLCPP_ERROR(get_logger(), "command rejected: %s", error.what());
        request_stop();
      }
    });
    enable_srv_ = create_service<std_srvs::srv::SetBool>(topic("enable"), [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
      if(!request->data) { request_stop(); response->success = true; response->message = stop_response(); return; }
      if(user_armed_ && !stop_requested_.load()) {
        response->success = true; response->message = "armed: existing session retained; waiting for commands"; return;
      }
      consume_snapshot();
      const auto phase = phase_.load();
      if(!allow_motion_ || !connect_ || !connected_ready_.load() || (phase != Phase::monitoring && phase != Phase::stopped) ||
         !latest_.measured_ns || age_s(latest_.measured_ns) > timeout_s_) {
        response->success = false;
        response->message = "enable requires enable_motion=true, fresh RT state and confirmed idle/stopped";
        return;
      }
      const auto preflight = check_wrench_limits(latest_.measured.wrench, max_measured_wrench_);
      if(preflight) {
        response->success = false;
        response->message = "enable rejected before motion: " + describe_wrench_violation(preflight, latest_.measured.pose);
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }
      arm_origin_ = latest_.measured.pose;
      stop_confirmed_.store(false);
      stop_requested_.store(false);
      arm_time_ns_.store(monotonic_ns());
      requested_session_.fetch_add(1);
      user_armed_ = true;
      response->success = true;
      response->message = "armed: waiting for a NEW command; RT K/wrench remain fixed for this session";
    });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(topic("stop"), [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      request_stop(); response->success = phase_.load() != Phase::stop_failed; response->message = stop_response();
    });
    publish_timer_ = create_wall_timer(std::chrono::duration<double>(1 / publish_rate), [this] { publish_tick(); });
    RCLCPP_INFO(get_logger(), "RT measured six-channel source=tau_ext_base; Fxyz/Txyz mapping assumed pending validation; origin undocumented; not raw wrist sensor");
    worker_ = std::thread([this] { worker_main(); });
  }

  ~CartesianImpedanceNode() override {
    shutdown_.store(true); stop_requested_.store(true);
    if(worker_.joinable()) worker_.join();
  }

 private:
  template<class T> T parameter(const std::string &name, const T &value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor; descriptor.read_only = true;
    return declare_parameter<T>(name, value, descriptor);
  }
  double positive(const std::string &name, double fallback) {
    const double value = parameter(name, fallback);
    if(!std::isfinite(value) || value <= 0) throw std::invalid_argument(name + " must be finite and positive");
    return value;
  }
  Six six_parameter(const std::string &name, const Six &fallback) {
    const auto values = parameter(name, std::vector<double>(fallback.begin(), fallback.end()));
    if(values.size() != 6 || !std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value) && value >= 0; }))
      throw std::invalid_argument(name + " requires six finite nonnegative values");
    Six result{}; std::copy(values.begin(), values.end(), result.begin()); return result;
  }
  static std::string topic(const char *suffix) { return std::string("/ar3/cartesian_impedance/") + suffix; }
  static void check(const std::error_code &ec, const char *operation) {
    if(ec) throw std::runtime_error(std::string(operation) + ": " + ec.message());
  }
  // Trigger is also the client's asynchronous stop-status poll. Repeated
  // polls must not restart the deadline or replace a terminal stop failure.
  void request_stop() { user_armed_ = false; stop_requested_.store(true); }
  std::string stop_response() const {
    if(stop_confirmed_.load()) {
      if(phase_.load() == Phase::fault)
        return "stopped: controller idle confirmed; RT monitoring unavailable, recover controller before restarting node";
      if(phase_.load() == Phase::stopping)
        return "stopping: controller idle confirmed; restoring RT monitoring only";
      return "stopped: RT motion ended; controller idle was confirmed; monitoring only";
    }
    if(phase_.load() == Phase::stop_failed) return "stop failed: RT stop/idle unconfirmed; failure latched, inspect status/log and recover controller before restarting node";
    return "stopping: waiting for RT loop exit and controller idle";
  }
  void set_phase(Phase phase, const std::string &detail) {
    StatusSample sample; sample.phase = phase; sample.session = active_session_;
    std::strncpy(sample.detail.data(), detail.c_str(), sample.detail.size() - 1);
    statuses_.publish(sample); phase_.store(phase);
  }
  void publish_metadata() {
    std_msgs::msg::String message;
    message.data = R"JSON({"schema":"pose6,wrench6,K6,D6,M6","motion_backend":"rt","control_mode":"cartesianImpedance","pose_stream_hz":1000,"session_fixed_K_wrench":true,"pose":"TCP xyz[m],rpy[rad];Rz*Ry*Rx","frame":"robot_base","pose_source":"RtSupportedFields.tcpPose_m","measured_wrench_source":"xCoreSDK.RtSupportedFields.tauExt_inBase (tau_ext_base)","controller_reported":true,"raw_sensor":false,"wrench_component_mapping":"assumed_Fxyz_Txyz_pending_vendor_or_physical_validation","wrench_origin":"not_documented_by_sdk_field","force_frame":"world; base wrench rotated; diagonal stiffness must remain representable","state_K":"configured_or_session_setpoint;not_controller_readback","D":"controller_default_not_applied;zero_unspecified","M":"not_supported;zero_unspecified","accepted_command":"smoothed_pose_returned_from_RT_callback;not_controller_ack_or_measured_completion","load_source":")JSON";
    message.data += override_enabled_ ? "startup_parameter_override" : "controller_toolset";
    message.data += "\",\"load_readback\":false}";
    metadata_pub_->publish(message);
  }
  void consume_snapshot() { snapshots_.read(latest_); }
  void publish_tick() {
    StatusSample status;
    if(statuses_.read(status) && !obsolete_session_status(status.session, requested_session_.load(), user_armed_)) {
      std_msgs::msg::String message; message.data = status.detail.data(); status_pub_->publish(message);
      RCLCPP_INFO(get_logger(), "%s", status.detail.data());
      if(stop_requested_.load())
        user_armed_ = false;
    }
    consume_snapshot();
    const bool fresh = latest_.measured_ns && age_s(latest_.measured_ns) <= timeout_s_;
    std_msgs::msg::Bool ready; ready.data = fresh && connected_ready_.load() && phase_.load() != Phase::fault; ready_pub_->publish(ready);
    if(!fresh || !connected_ready_.load() || latest_.sequence == published_sequence_) return;
    published_sequence_ = latest_.sequence;
    ArrayMsg state; state.data = encode(latest_.measured); state_pub_->publish(state);
    // Reconstruct sample ROS time from its measured monotonic age; never stamp
    // a cached 1 ms packet as if newly measured by this ROS timer.
    const auto stamp = now() - rclcpp::Duration::from_seconds(age_s(latest_.measured_ns));
    geometry_msgs::msg::PoseStamped pose; pose.header.stamp = stamp; pose.header.frame_id = frame_id_;
    pose.pose.position.x = latest_.measured.pose[0]; pose.pose.position.y = latest_.measured.pose[1]; pose.pose.position.z = latest_.measured.pose[2];
    const auto q = orientation(latest_.measured.pose);
    pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y(); pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);
    geometry_msgs::msg::WrenchStamped wrench; wrench.header = pose.header;
    const auto &w = latest_.measured.wrench;
    wrench.wrench.force.x = w[0]; wrench.wrench.force.y = w[1]; wrench.wrench.force.z = w[2];
    wrench.wrench.torque.x = w[3]; wrench.wrench.torque.y = w[4]; wrench.wrench.torque.z = w[5]; wrench_pub_->publish(wrench);
    if(latest_.accepted_valid && phase_.load() == Phase::active && !stop_requested_.load()) {
      ArrayMsg accepted; accepted.data = encode(latest_.accepted); accepted_pub_->publish(accepted);
    }
  }

  // These functions and SDK objects belong to worker_ or its blocking SDK loop,
  // never to ROS callbacks. There are no concurrent SDK service/RPC calls.
  bool read_sample(unsigned budget, Clock::duration first_wait = Clock::duration::zero()) {
    const unsigned packets = drain_rt_packets([this, first_wait](bool first) {
      return robot_->updateRobotState(first ? first_wait : Clock::duration::zero());
    }, budget);
    if(!packets) return false;
    Matrix16 pose{}; Six wrench{}; double elbow{};
    if(robot_->getStateData(rokae::RtSupportedFields::tcpPose_m, pose) ||
       robot_->getStateData(rokae::RtSupportedFields::tauExt_inBase, wrench) ||
       robot_->getStateData(rokae::RtSupportedFields::elbow_m, elbow)) throw std::runtime_error("requested RT state field unavailable");
    for(double value : wrench) if(!std::isfinite(value)) throw std::runtime_error("nonfinite RT external wrench channel");
    if(!std::isfinite(elbow)) throw std::runtime_error("nonfinite RT elbow");
    sample_.measured = Command{matrix_pose(pose), wrench, session_config_.stiffness, {}, {}};
    sample_.elbow = elbow; sample_.measured_ns = monotonic_ns(); ++sample_.sequence;
    return true;
  }
  void monitor_sample() {
    if(read_sample(4096, 1ms)) { sample_.accepted_valid = false; snapshots_.publish(sample_); }
  }
  void check_cancelled() {
    if(stop_requested_.load() || shutdown_.load()) throw std::runtime_error("stop requested during initialization");
    if(age_s(session_start_ns_) > startup_timeout_s_) throw std::runtime_error("RT initialization deadline exceeded");
    RtCommand newest;
    if(commands_.read(newest) && newest.session == active_session_) live_command_ = newest;
    if(age_s(live_command_.received_ns) > timeout_s_) throw std::runtime_error("command watchdog expired during initialization");
    if(!same_session_parameters(live_command_.value, session_config_)) throw std::runtime_error("K/wrench/D/M are fixed during an RT session");
  }
  template<class F> void setup_rpc(const char *label, F call) {
    check_cancelled();
    RCLCPP_INFO(get_logger(), "RT setup: %s", label);
    std::error_code ec; call(ec); check(ec, label);
    monitor_sample(); check_cancelled();
  }
  void cache_calibration() {
    std::error_code ec; const auto toolset = robot_->toolset(ec); check(ec, "toolset");
    for(std::size_t i = 0; i < 3; ++i) { tool_pose_[i] = toolset.end.trans[i]; tool_pose_[i + 3] = toolset.end.rpy[i]; }
    tool_matrix_ = pose_matrix(tool_pose_);
    const auto controller = payload_load(toolset.load.mass, {toolset.load.cog.begin(), toolset.load.cog.end()},
      {toolset.load.inertia.begin(), toolset.load.inertia.end()});
    selected_load_ = select_payload_load(override_enabled_, controller, override_load_);
    base_in_world_ = robot_->baseFrame(ec); check(ec, "baseFrame"); pose_transform(base_in_world_);
    RCLCPP_INFO(get_logger(), "Configured TCP offset (flange->TCP)=[%.4f %.4f %.4f; %.4f %.4f %.4f]",
      tool_pose_[0], tool_pose_[1], tool_pose_[2], tool_pose_[3], tool_pose_[4], tool_pose_[5]);
    RCLCPP_INFO(get_logger(), "world_T_base=[%.4f %.4f %.4f; %.4f %.4f %.4f]; RT TCP commands and state are directly base coordinates",
      base_in_world_[0], base_in_world_[1], base_in_world_[2], base_in_world_[3], base_in_world_[4], base_in_world_[5]);
    RCLCPP_INFO(get_logger(), "RT load source=%s mass=%.3fkg COG[m]=[%.5f %.5f %.5f] inertia[kg*m^2]=[%.6f %.6f %.6f] (configuration, not force readback)",
      override_enabled_ ? "startup_parameter_override" : "controller_toolset", selected_load_.mass_kg,
      selected_load_.cog_m[0], selected_load_.cog_m[1], selected_load_.cog_m[2],
      selected_load_.inertia_kg_m2[0], selected_load_.inertia_kg_m2[1], selected_load_.inertia_kg_m2[2]);
  }
  void connect_sdk() {
    process_lock_ = std::make_unique<RobotProcessLock>(robot_ip_);
    robot_ = std::make_unique<rokae::ArRobot>(); robot_->connectToRobot(robot_ip_, local_ip_);
    std::error_code ec;
    const auto operation = robot_->operationState(ec); check(ec, "read operationState before RT takeover");
    if(operation == rokae::OperationState::rtControlling && recover_stale_rt_) recover_stale_rt_once();
    else if(operation != rokae::OperationState::idle)
      throw std::runtime_error(std::string("RT takeover refused: operationState=") + operation_name(operation) +
        "(" + std::to_string(static_cast<int>(operation)) + "); stop the existing controller operation and disable RT on the controller before restarting this node");
    cache_calibration();
    robot_->setRtNetworkTolerance(network_tolerance_, ec); check(ec, "setRtNetworkTolerance");
    session_config_.stiffness = stiffness_;
    open_rt_monitor();
    set_phase(Phase::monitoring, "monitoring: RT 1ms state; no motion until enable and new command");
  }
  void recover_stale_rt_once() {
    static_assert(static_cast<int>(rokae::OperationState::idle) == 0 && static_cast<int>(rokae::OperationState::rtControlling) == 2);
    static_assert(static_cast<int>(rokae::PowerState::on) == 0 && static_cast<int>(rokae::PowerState::off) == 1);
    RCLCPP_WARN(get_logger(), "RT startup recovery explicitly enabled; checking >=1s and >=20 samples: joint position range <=20 microrad, instantaneous |dq|<=0.01, time-weighted |mean(dq)|<=0.001 and mean(|dq|)<=0.002 rad/s. These are diagnostic limits, not documented SDK precision. SDK cannot identify remote control owners; this option requires confirmed exclusive control");
    RtStartupRecoveryCheck stationary(monotonic_ns(), std::min(3.0, stop_timeout_s_));
    while(true) {
      if(shutdown_.load() || stop_requested_.load()) throw std::runtime_error("stop/shutdown during RT startup recovery");
      stationary.check_deadline(monotonic_ns());
      std::error_code ec;
      const auto operation = robot_->operationState(ec); check(ec, "startup recovery operationState");
      const auto power = robot_->powerState(ec); check(ec, "startup recovery powerState");
      RtStartupRecoveryCheck::Joints position{}, velocity{};
      if(operation == rokae::OperationState::rtControlling) {
        // These joint values exist only for the startup stationarity check;
        // they are never added to the state topic or the training dataset.
        position = robot_->jointPos(ec); check(ec, "startup recovery jointPos");
        velocity = robot_->jointVel(ec); check(ec, "startup recovery jointVel");
      }
      const auto decision = stationary.observe(static_cast<int>(operation), static_cast<int>(power), position, velocity, monotonic_ns());
      if(decision == StartupRecoveryDecision::already_idle) {
        RCLCPP_INFO(get_logger(), "RT startup recovery: controller became idle before any mode change");
        return;
      }
      if(decision == StartupRecoveryDecision::exit_rt_mode) break;
      std::this_thread::sleep_for(50ms);
    }
    stationary.check_deadline(monotonic_ns());
    std::error_code ec;
    const auto final_operation = robot_->operationState(ec); check(ec, "startup recovery final operationState");
    const auto final_power = robot_->powerState(ec); check(ec, "startup recovery final powerState");
    stationary.check_deadline(monotonic_ns());
    if(final_power != rokae::PowerState::on && final_power != rokae::PowerState::off)
      throw std::runtime_error("RT startup recovery refused: unsafe power state before mode change");
    if(final_operation == rokae::OperationState::idle) return;
    if(final_operation != rokae::OperationState::rtControlling)
      throw std::runtime_error(std::string("RT startup recovery refused before mode change: operationState=") +
        operation_name(final_operation) + "(" + std::to_string(static_cast<int>(final_operation)) + ")");
    if(shutdown_.load() || stop_requested_.load()) throw std::runtime_error("stop/shutdown before RT startup recovery mode change");
    RCLCPP_WARN(get_logger(), "RT startup recovery: stationary checks passed; leaving previous RT mode once with Idle(preserve power), no stopMove or motion command");
    // This is a single authorized cleanup attempt. Until idle is confirmed,
    // rt_mode_enabled_ remains false so failures cannot force another cleanup
    // of a controller session this node has not yet acquired.
    robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec, false);
    check(ec, "startup recovery setMotionControlMode(Idle, preserve power)");
    while(true) {
      if(shutdown_.load() || stop_requested_.load()) throw std::runtime_error("stop/shutdown while confirming RT startup recovery idle");
      stationary.check_deadline(monotonic_ns());
      const auto operation = robot_->operationState(ec); check(ec, "startup recovery confirm idle");
      const auto power = robot_->powerState(ec); check(ec, "startup recovery confirm power");
      if(power != rokae::PowerState::on && power != rokae::PowerState::off)
        throw std::runtime_error("RT startup recovery refused: unsafe power state while confirming idle");
      stationary.check_deadline(monotonic_ns());
      if(operation == rokae::OperationState::idle) {
        RCLCPP_INFO(get_logger(), "RT startup recovery: controller idle confirmed; proceeding to monitoring, motion remains disarmed");
        return;
      }
      if(operation != rokae::OperationState::rtControlling)
        throw std::runtime_error(std::string("RT startup recovery refused during idle confirmation: operationState=") +
          operation_name(operation) + "(" + std::to_string(static_cast<int>(operation)) + ")");
      std::this_thread::sleep_for(10ms);
    }
  }
  void invalidate_monitor() {
    connected_ready_.store(false);
    sample_.measured_ns = 0; sample_.accepted_ns = 0; sample_.accepted_valid = false;
    ++sample_.sequence; snapshots_.publish(sample_);
  }
  void suspend_rt_monitor() {
    invalidate_monitor();
    if(receiving_) { robot_->stopReceiveRobotState(); receiving_ = false; }
    if(rt_) rt_->disconnectNetwork();
  }
  void open_rt_monitor() {
    invalidate_monitor();
    std::error_code ec;
    rt_mode_enabled_ = true;  // failed RPC outcome still requires mode cleanup
    robot_->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec, false);
    check(ec, "setMotionControlMode(RtCommand, preserve power)");
    // The SDK reconnects an existing controller here after disconnectNetwork.
    rt_ = robot_->getRtMotionController().lock();
    if(!rt_) throw std::runtime_error("RT controller unavailable");
    rt_->setEndEffectorFrame(tool_matrix_, ec); check(ec, "setEndEffectorFrame(existing TCP)");
    receiving_ = true;  // start may partially succeed before throwing
    robot_->startReceiveRobotState(1ms, {rokae::RtSupportedFields::tcpPose_m, rokae::RtSupportedFields::tauExt_inBase, rokae::RtSupportedFields::elbow_m});
    rt_->setCpuAffinity(cpu_);
    const auto deadline = Clock::now() + std::chrono::duration<double>(startup_timeout_s_);
    while(!sample_.measured_ns && Clock::now() < deadline && !shutdown_.load()) monitor_sample();
    if(!sample_.measured_ns || age_s(sample_.measured_ns) > state_timeout_s_)
      throw std::runtime_error("fresh RT monitoring state unavailable");
    if(shutdown_.load()) throw std::runtime_error("shutdown during RT monitoring initialization");
    connected_ready_.store(true);
  }
  void run_session() {
    session_start_ns_ = monotonic_ns(); session_config_ = live_command_.value;
    set_phase(Phase::initializing, "initializing: RT session parameters before startMove; K/wrench fixed, D/M not applied");
    monitor_sample();
    if(age_s(sample_.measured_ns) > state_timeout_s_) throw std::runtime_error("fresh RT preflight state unavailable");
    const auto preflight = check_wrench_limits(sample_.measured.wrench, max_measured_wrench_);
    if(preflight) throw std::runtime_error("pre-motion feedback check: " + describe_wrench_violation(preflight, sample_.measured.pose));
    // World force axes are documented. Only exactly representable diagonal
    // stiffness is accepted; general rotated anisotropy is rejected.
    const auto world_k = stiffness_in_world(session_config_.stiffness, base_in_world_);
    const auto world_wrench = wrench_in_base(session_config_.wrench, base_in_world_);
    if(auto_power_) {
      setup_rpc("automatic mode", [this](auto &ec) { robot_->setOperateMode(rokae::OperateMode::automatic, ec); });
      setup_rpc("power on", [this](auto &ec) { robot_->setPowerState(true, ec); });
    }
    setup_rpc("check automatic mode", [this](auto &ec) {
      if(robot_->operateMode(ec) != rokae::OperateMode::automatic && !ec) throw std::runtime_error("automatic mode required");
    });
    setup_rpc("check power", [this](auto &ec) {
      if(robot_->powerState(ec) != rokae::PowerState::on && !ec) throw std::runtime_error("robot must be powered on");
    });
    setup_rpc("setLoad(existing controller payload)", [this](auto &ec) {
      rokae::Load load; load.mass = selected_load_.mass_kg; load.cog = selected_load_.cog_m; load.inertia = selected_load_.inertia_kg_m2;
      rt_->setLoad(load, ec);
    });
    setup_rpc("setFcCoor(world, existing TCP offset)", [this](auto &ec) { rt_->setFcCoor(tool_matrix_, rokae::FrameType::world, ec); });
    setup_rpc("setCartesianImpedance(session K in world)", [this, world_k](auto &ec) { rt_->setCartesianImpedance(world_k, ec); });
    setup_rpc("setCartesianImpedanceDesiredTorque(session wrench in world)", [this, world_wrench](auto &ec) { rt_->setCartesianImpedanceDesiredTorque(world_wrench, ec); });
    setup_rpc("setFilterFrequency", [this](auto &ec) { rt_->setFilterFrequency(20, 20, 20, ec); });
    if(!rt_->setFilterLimit(true, 20)) throw std::runtime_error("setFilterLimit failed");
    monitor_sample(); check_cancelled();
    if(age_s(sample_.measured_ns) > state_timeout_s_) throw std::runtime_error("fresh RT start pose unavailable");
    const auto before_start = check_wrench_limits(sample_.measured.wrench, max_measured_wrench_);
    if(before_start) throw std::runtime_error("before startMove: " + describe_wrench_violation(before_start, sample_.measured.pose));
    origin_ = sample_.measured.pose; output_pose_ = origin_; elbow_ = sample_.elbow;
    validate_target(origin_, live_command_.value.pose, origin_, max_step_m_, max_step_rad_, radius_);
    follower_.reset(origin_); loop_exit_ = LoopExit::none; wrench_violation_ = {};
    // Manually consume nonblocking packets inside the callback to distinguish
    // a fresh update (>0) from repeated getStateData cache reads.
    rt_->setControlLoop<rokae::CartesianPosition>([this] { return rt_callback(); }, priority_, false);
    check_cancelled(); stop_progress_.start_attempted();
    rt_->startMove(rokae::RtControllerMode::cartesianImpedance);
    // startMove is a synchronous RPC and may have accumulated tens of 1 ms
    // packets. Drain on the worker, then hand the latest real pose to the RT
    // callback. Its normal eight-packet bound remains strict during motion.
    monitor_sample(); check_cancelled();
    if(age_s(sample_.measured_ns) > state_timeout_s_) throw std::runtime_error("RT handoff state stale after startMove");
    origin_ = sample_.measured.pose; output_pose_ = origin_; elbow_ = sample_.elbow;
    validate_target(origin_, live_command_.value.pose, origin_, max_step_m_, max_step_rad_, radius_);
    follower_.reset(origin_);
    set_phase(Phase::active, "active: RT 1000Hz continuous TCP pose; session K/wrench applied; D/M not applied");
    rt_->startLoop(true);
    // A normal callback setFinished return tells the SDK to stop motion itself.
    // Explicit stopMove is reserved for incomplete/exceptional start/loop exit.
    stop_progress_.loop_returned_normally(loop_exit_ != LoopExit::none);
  }
  rokae::CartesianPosition rt_callback() noexcept {
    rokae::CartesianPosition output;
    try {
      const bool fresh = read_sample(8);
      RtCommand newest;
      if(commands_.read(newest) && newest.session == active_session_) live_command_ = newest;
      loop_exit_ = rt_session_gate(live_command_, session_config_, active_session_, sample_.measured_ns,
        monotonic_ns(), timeout_s_, state_timeout_s_, stop_requested_.load() || shutdown_.load());
      if(loop_exit_ == LoopExit::none) {
        validate_target(sample_.measured.pose, live_command_.value.pose, origin_, max_step_m_, max_step_rad_, radius_);
        validate_target(sample_.measured.pose, sample_.measured.pose, origin_, max_step_m_, max_step_rad_, radius_);
        wrench_violation_ = check_wrench_limits(sample_.measured.wrench, max_measured_wrench_);
        if(wrench_violation_) loop_exit_ = LoopExit::safety_limit;
      }
      if(loop_exit_ == LoopExit::none) output_pose_ = follower_.step(live_command_.value.pose, speed_m_s_, angular_speed_, acceleration_, angular_acceleration_);
      if(loop_exit_ == LoopExit::none)
        validate_target(sample_.measured.pose, output_pose_, origin_, max_step_m_, max_step_rad_, radius_);
      output.pos = pose_matrix(output_pose_); output.elbow = elbow_; output.hasElbow = true;
      sample_.accepted_valid = loop_exit_ == LoopExit::none;
      if(sample_.accepted_valid) { sample_.accepted = session_config_; sample_.accepted.pose = output_pose_; sample_.accepted_ns = monotonic_ns(); }
      if(fresh) snapshots_.publish(sample_);
      if(loop_exit_ != LoopExit::none) { stop_requested_.store(true); output.setFinished(); }
    } catch(...) {
      loop_exit_ = LoopExit::invalid_state;
      stop_requested_.store(true);
      output.pos = pose_matrix(output_pose_); output.elbow = elbow_; output.hasElbow = true; output.setFinished();
    }
    return output;
  }
  std::string loop_reason() const {
    switch(loop_exit_) {
      case LoopExit::requested: return "stop requested";
      case LoopExit::command_timeout: return "command watchdog expired";
      case LoopExit::state_timeout: return "RT state watchdog expired";
      case LoopExit::parameter_change: return "session K/wrench/D/M changed; stop and re-enable explicitly";
      case LoopExit::safety_limit: return "measured external feedback exceeded stop limit: " + describe_wrench_violation(wrench_violation_, sample_.measured.pose);
      case LoopExit::invalid_state: return "RT state/target validation failed";
      default: return "RT loop ended";
    }
  }
  static const char *operation_name(rokae::OperationState operation) noexcept {
    using State = rokae::OperationState;
    switch(operation) {
      case State::idle: return "idle";
      case State::jog: return "jog";
      case State::rtControlling: return "rtControlling";
      case State::drag: return "drag";
      case State::rlProgram: return "rlProgram";
      case State::demo: return "demo";
      case State::dynamicIdentify: return "dynamicIdentify";
      case State::frictionIdentify: return "frictionIdentify";
      case State::loadIdentify: return "loadIdentify";
      case State::moving: return "moving";
      case State::jogging: return "jogging";
      default: return "unknown";
    }
  }
  bool stop_and_confirm(const std::string &reason, bool restore_monitor = true) {
    stop_requested_.store(true); sample_.accepted_valid = false;
    if(stop_confirmed_.load()) return true;
    if(!stop_progress_.begin_confirmation()) return false;
    invalidate_monitor();
    set_phase(Phase::stopping, "stopping: " + reason);
    auto last_operation = rokae::OperationState::unknown;
    try {
      const auto deadline = Clock::now() + std::chrono::duration<double>(stop_timeout_s_);
      finish_rt_session(stop_progress_.needs_stop_rpc(),
        [this] {
          if(!rt_) throw std::runtime_error("RT controller unavailable for required stopMove");
          rt_->stopMove(); stop_progress_.stop_acknowledged();
        },
        [this] { suspend_rt_monitor(); },
        [this] {
          // Normal setFinished has already stopped motion. Leaving RT mode is
          // a distinct operation: operationState reports the RCI main state
          // until eci.disable, even when no RT motion is executing.
          if(rt_mode_enabled_) {
            std::error_code ec;
            RCLCPP_INFO(get_logger(), "RT stop cleanup: setMotionControlMode(Idle, preserve power)");
            robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec, false);
            check(ec, "setMotionControlMode(Idle, preserve power)");
            rt_mode_enabled_ = false;
          }
        },
        [this, deadline, &last_operation] {
          while(Clock::now() < deadline) {
            std::error_code ec;
            const auto operation = robot_->operationState(ec); check(ec, "confirm RT stop idle");
            if(operation != last_operation) {
              RCLCPP_INFO(get_logger(), "RT stop confirmation: operationState=%s(%d)", operation_name(operation), static_cast<int>(operation));
              last_operation = operation;
            }
            if(operation == rokae::OperationState::idle) {
              stop_progress_.stop_acknowledged(); stop_confirmed_.store(true); return;
            }
            if(operation == rokae::OperationState::unknown) throw std::runtime_error("controller operation state unknown");
            std::this_thread::sleep_for(10ms);
          }
          throw std::runtime_error("controller idle confirmation timed out");
        });
    } catch(const std::exception &error) {
      const auto detail = std::string("stop failed: ") + error.what() + "; operationState=" + operation_name(last_operation) +
        "(" + std::to_string(static_cast<int>(last_operation)) + "); trigger=" + reason;
      stop_confirmed_.store(false);
      set_phase(Phase::stop_failed, detail);
      // Log here as well as publishing the terminal status: shutdown or a
      // slow ROS timer must not conceal a failed physical stop confirmation.
      RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
      return false;
    }
    if(restore_monitor && !shutdown_.load()) {
      try {
        RCLCPP_INFO(get_logger(), "RT stop cleanup: idle confirmed; restoring monitoring only (no startMove)");
        open_rt_monitor();
      } catch(const std::exception &error) {
        // Motion has been confirmed stopped. A monitoring failure must not be
        // relabelled as uncertain physical stopping or allow another enable.
        std::string detail = std::string("stopped: controller idle confirmed; RT monitoring restoration failed: ") + error.what();
        suspend_rt_monitor();
        if(rt_mode_enabled_) {
          std::error_code ec; robot_->setMotionControlMode(rokae::MotionControlMode::Idle, ec, false);
          if(ec) detail += "; monitor cleanup Idle failed: " + ec.message();
          else rt_mode_enabled_ = false;
        }
        set_phase(Phase::fault, detail);
        RCLCPP_ERROR(get_logger(), "%s", detail.c_str());
        return true;
      }
    }
    set_phase(Phase::stopped, receiving_
      ? "stopped: controller idle confirmed; RT monitoring restored, no motion until a new enable and command"
      : "stopped: controller idle confirmed; RT mode and monitoring closed");
    return true;
  }
  void stop_on_worker_exit(const std::string &reason) {
    // A previous stop may have reopened RT for monitoring. Shutdown still
    // exits that mode, without a duplicate stopMove or reopening it again.
    if(stop_confirmed_.load() && rt_mode_enabled_) {
      stop_progress_.reset_for_session(); stop_confirmed_.store(false);
    }
    if(!stop_confirmed_.load()) stop_and_confirm(reason, false);
  }
  void worker_main() noexcept {
    if(!connect_) { stop_confirmed_.store(true); set_phase(Phase::offline, "offline: RT backend; no robot connection or fabricated state"); return; }
    try {
      set_phase(Phase::connecting, "connecting: RT robot/state initialization"); connect_sdk();
      while(!shutdown_.load()) {
        if(receiving_) monitor_sample();
        else std::this_thread::sleep_for(10ms);
        const auto requested = requested_session_.load();
        if(active_session_ != requested) {
          stop_progress_.reset_for_session();
          active_session_ = requested;
        }
        if(stop_requested_.load()) {
          if(!stop_confirmed_.load()) stop_and_confirm("stop requested");
          continue;
        }
        if(requested == 0 || requested == completed_session_) continue;
        if(phase_.load() != Phase::armed) set_phase(Phase::armed, "armed: waiting for fresh RT command");
        RtCommand command;
        if(commands_.read(command) && command.session == active_session_) live_command_ = command;
        if(live_command_.session != active_session_) {
          if(age_s(arm_time_ns_.load()) > timeout_s_) { completed_session_ = active_session_; stop_and_confirm("first command watchdog expired"); }
          continue;
        }
        try { run_session(); }
        catch(const std::exception &error) {
          RCLCPP_ERROR(get_logger(), "RT session error: %s", error.what());
          completed_session_ = active_session_; stop_and_confirm(error.what()); continue;
        }
        completed_session_ = active_session_; stop_and_confirm(loop_reason());
      }
      connected_ready_.store(false);
      stop_on_worker_exit("node shutdown");
    } catch(const std::exception &error) {
      connected_ready_.store(false);
      stop_requested_.store(true);
      if(robot_ && rt_mode_enabled_) stop_on_worker_exit(error.what());
      else set_phase(Phase::fault, std::string("fault: ") + error.what());
    }
    connected_ready_.store(false);
    if(robot_) {
      if(receiving_) robot_->stopReceiveRobotState();
      if(rt_) rt_->disconnectNetwork();
      std::error_code ec; robot_->disconnectFromRobot(ec);
    }
  }

  std::string robot_ip_, local_ip_, frame_id_;
  bool connect_{}, allow_motion_{}, auto_power_{}, recover_stale_rt_{}, override_enabled_{}, user_armed_{};
  double timeout_s_{}, state_timeout_s_{}, startup_timeout_s_{}, stop_timeout_s_{}, max_step_m_{}, max_step_rad_{}, radius_{};
  double speed_m_s_{}, angular_speed_{}, acceleration_{}, angular_acceleration_{};
  int priority_{}, cpu_{}, network_tolerance_{};
  Six stiffness_{}, damping_{}, mass_{}, max_stiffness_{}, max_wrench_{}, max_measured_wrench_{}, arm_origin_{};
  PayloadLoad override_load_, selected_load_;
  std::atomic<bool> shutdown_{false}, stop_requested_{false}, stop_confirmed_{false}, connected_ready_{false};
  std::atomic<std::uint64_t> requested_session_{0};
  std::atomic<std::int64_t> arm_time_ns_{0};
  std::atomic<Phase> phase_{Phase::connecting};
  LatestMailbox<RtCommand> commands_;
  LatestMailbox<RtSnapshot> snapshots_;
  LatestMailbox<StatusSample> statuses_;
  RtSnapshot latest_;
  std::uint64_t published_sequence_{};
  std::thread worker_;
  // Worker/RT-loop-owned data below. ROS never accesses SDK or these objects.
  std::unique_ptr<RobotProcessLock> process_lock_;
  std::unique_ptr<rokae::ArRobot> robot_;
  std::shared_ptr<rokae::RtMotionControlCobot<7>> rt_;
  bool receiving_{false}, rt_mode_enabled_{false};
  RtStopProgress stop_progress_;
  std::uint64_t active_session_{}, completed_session_{};
  std::int64_t session_start_ns_{};
  RtCommand live_command_;
  Command session_config_;
  RtSnapshot sample_;
  Six tool_pose_{}, base_in_world_{}, origin_{}, output_pose_{};
  Matrix16 tool_matrix_{};
  double elbow_{};
  RtPoseFollower follower_;
  LoopExit loop_exit_{LoopExit::none};
  WrenchViolation wrench_violation_{};
  rclcpp::Publisher<ArrayMsg>::SharedPtr state_pub_, accepted_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_, metadata_pub_;
  rclcpp::Subscription<ArrayMsg>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};
}  // namespace ar3_cartesian_impedance

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<ar3_cartesian_impedance::CartesianImpedanceNode>()); }
  catch(const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("cartesian_impedance_node"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
