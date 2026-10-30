#pragma once

#include "ar3_cartesian_impedance/command.hpp"

namespace ar3_cartesian_impedance {

// A pose denotes parent_T_child: translation and Rz(rz)*Ry(ry)*Rx(rx).
inline Eigen::Isometry3d pose_transform(const Six &pose) {
  for(const auto value : pose)
    if(!std::isfinite(value)) throw std::invalid_argument("coordinate transform contains nonfinite values");
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = orientation(pose).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  return transform;
}

inline Six transform_pose(const Eigen::Isometry3d &transform) {
  const auto &r = transform.linear();
  const auto &p = transform.translation();
  const double pitch = std::atan2(-r(2, 0), std::hypot(r(0, 0), r(1, 0)));
  double roll, yaw;
  // Match xCoreSDK Utils::transArrayToPosture, including its gimbal-lock branch.
  if(std::abs(pitch) > 1.5707963267948966 - 1e-12) {
    roll = 0;
    yaw = std::atan2(-r(0, 1), r(1, 1));
  } else {
    roll = std::atan2(r(2, 1), r(2, 2));
    yaw = std::atan2(r(1, 0), r(0, 0));
  }
  return {p.x(), p.y(), p.z(), roll, pitch, yaw};
}

}  // namespace ar3_cartesian_impedance
