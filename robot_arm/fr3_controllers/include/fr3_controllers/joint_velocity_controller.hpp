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

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers {
using CmdType = std_msgs::msg::Float64MultiArray;

/**
 * Joint velocity controller
 */
class JointVelocityController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;
  [[nodiscard]] auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;
  auto update(const rclcpp::Time& time, const rclcpp::Duration& period) -> controller_interface::return_type override;
  auto on_init() -> CallbackReturn override;
  auto on_configure(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;
  auto on_activate(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  const int num_joints = 7;

  bool is_gazebo{false};
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);

  Vector7d q_;
  Vector7d q_goal_;
  Vector7d qd_;
  Vector7d qd_filtered_;
  Vector7d qd_goal_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  double max_accel_ = 4;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  // std::vector<double> desired_velocities_;  // Raw desired velocities from topic
  // std::vector<double> smoothed_velocities_; // Smoothed velocities sent to robot

  void updateJointStates();
};

}  // namespace fr3_controllers
