// Copyright (c) 2023 Franka Robotics GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <Eigen/Core>
#include <array>
#include <rclcpp/duration.hpp>
#include <utility>
/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class MotionGeneratorCartesian
{
 public:
  // using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range (0, 1].
   * @param[in] x_ee Start joint positions.
   * @param[in] x_ee_goal Target joint positions.
   */
  MotionGeneratorCartesian(double speed_factor, const Vector3d& x_ee, const Vector3d& x_ee_goal);

  /**
   * Sends joint position calculations
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] trajectory_time Amount of time, that has passed since the start of the trajectory.
   *
   * @return Joint positions to use inside a control loop and a boolean indicating whether the
   * motion is finished.
   */
  std::pair<Vector3d, bool> getDesiredEEPosition(const rclcpp::Duration& trajectory_time);

 private:
  // using Vector7i = Eigen::Matrix<int, 7, 1>;
  using Vector3i = Eigen::Matrix<int, 3, 1>;

  bool calculateDesiredValues(double time, Vector3d* delta_x_ee_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  // static const int kJoints = 7;

  // Vector7d q_start_;
  // Vector7d delta_q_;

  Vector3d x_ee_start_;
  Vector3d x_ee_goal_;
  Vector3d delta_x_ee_;  // Distance between start and goal
  Vector3d dx_ee_max_sync_;

  // Vector7d dq_max_sync_;
  Vector3d t_1_sync_;
  Vector3d t_2_sync_;
  Vector3d t_f_sync_;
  Vector3d x_ee_1_;

  double time_ = 0.0;

  // Vector7d dq_max_ = (Vector7d() << 2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5).finished();  // in m/s
  // Vector7d ddq_max_start_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();         // in m/s^2
  // Vector7d ddq_max_goal_ = (Vector7d() << 5, 5, 5, 5, 5, 5, 5).finished();          // in m/s^2

  Vector3d dx_ee_max_ = (Vector3d() << 2.0, 2.0, 2.0).finished();         // in m/s
  Vector3d ddx_ee_max_start_ = (Vector3d() << 4.5, 4.5, 4.5).finished();  // in m/s
  Vector3d ddx_ee_max_goal_ = (Vector3d() << 4.5, 4.5, 4.5).finished();   // in m/s
};
