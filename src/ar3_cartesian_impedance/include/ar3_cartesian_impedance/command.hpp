#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace ar3_cartesian_impedance {
using Six = std::array<double, 6>;
constexpr std::size_t kCommandSize = 30;

struct Command {
  Six pose{}, wrench{}, stiffness{}, damping{}, mass{};
};

inline Eigen::Quaterniond orientation(const Six &pose) {
  // xCoreSDK utility.h: R = Rz(rz) * Ry(ry) * Rx(rx).
  return Eigen::Quaterniond(Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()));
}

inline Six wrench_in_base(const Six &tool_wrench, const Six &tcp_pose) {
  const auto rotation = orientation(tcp_pose);
  const Eigen::Vector3d force = rotation * Eigen::Vector3d(tool_wrench[0], tool_wrench[1], tool_wrench[2]);
  const Eigen::Vector3d torque = rotation * Eigen::Vector3d(tool_wrench[3], tool_wrench[4], tool_wrench[5]);
  // Same TCP origin: rotate only; no translation/moment-arm shift.
  return {force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z()};
}

inline Command decode(const std::vector<double> &data, const Six &damping, const Six &mass,
                      const Six &max_wrench, const Six &max_stiffness) {
  if(data.size() != kCommandSize) throw std::invalid_argument("command requires 30 values: pose6,wrench6,K6,D6,M6");
  if(!std::all_of(data.begin(), data.end(), [](double v) { return std::isfinite(v); }))
    throw std::invalid_argument("command contains NaN or infinity");
  Command result;
  std::copy_n(data.begin(), 6, result.pose.begin());
  std::copy_n(data.begin() + 6, 6, result.wrench.begin());
  std::copy_n(data.begin() + 12, 6, result.stiffness.begin());
  std::copy_n(data.begin() + 18, 6, result.damping.begin());
  std::copy_n(data.begin() + 24, 6, result.mass.begin());
  for(std::size_t i = 0; i < 6; ++i) {
    if(std::abs(result.wrench[i]) > max_wrench[i]) throw std::invalid_argument("desired wrench exceeds configured limit");
    if(result.stiffness[i] < 0 || result.stiffness[i] > max_stiffness[i])
      throw std::invalid_argument("stiffness exceeds configured limit");
    if(std::abs(result.damping[i] - damping[i]) > 1e-9 || std::abs(result.mass[i] - mass[i]) > 1e-9)
      throw std::invalid_argument("RT D/M are unspecified metadata and are not applied: values must remain zero");
  }
  return result;
}

inline std::vector<double> encode(const Command &command) {
  std::vector<double> result;
  result.reserve(kCommandSize);
  for(const auto *v : {&command.pose, &command.wrench, &command.stiffness, &command.damping, &command.mass})
    result.insert(result.end(), v->begin(), v->end());
  return result;
}

inline void validate_target(const Six &current, const Six &target, const Six &origin,
                            double max_step_m, double max_step_rad, double workspace_radius_m) {
  double distance = 0, origin_distance = 0;
  for(std::size_t i = 0; i < 3; ++i) {
    distance += std::pow(target[i] - current[i], 2);
    origin_distance += std::pow(target[i] - origin[i], 2);
  }
  if(std::sqrt(distance) > max_step_m) throw std::invalid_argument("target translation step exceeds limit");
  if(orientation(current).angularDistance(orientation(target)) > max_step_rad)
    throw std::invalid_argument("target orientation step exceeds limit");
  if(std::sqrt(origin_distance) > workspace_radius_m) throw std::invalid_argument("target exceeds session workspace radius");
}
}  // namespace ar3_cartesian_impedance
