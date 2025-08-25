// Copyright (c) 2025
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "franka_semantic_components/franka_robot_model.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers
{

/// Operational Space Control PD Controller for the Franka Research 3
class OSCPDController : public controller_interface::ControllerInterface
{
 public:
  using Vector3d = Eigen::Vector3d;
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  std::string arm_id_;
  const int num_joints = 7;

  // Joint states
  Vector7d q_;
  Vector7d dq_;

  // Control gains (separate for position and orientation)
  double kp_pos_gain_;  // Proportional gain for position
  double kd_pos_gain_;  // Derivative gain for position
  double kp_ori_gain_;  // Proportional gain for orientation
  double kd_ori_gain_;  // Derivative gain for orientation

  // Desired position from topic and fixed orientation
  Eigen::Vector3d position_des_;
  Eigen::Quaterniond orientation_des_;  // Fixed desired orientation (pointing down)
  bool goal_received_;
  bool orientation_initialized_;

  // Topic subscription
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;

  // Robot model for computing Jacobian
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  // Interface names
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  void updateJointStates();
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  Vector6d computeError(const Eigen::Vector3d& current_position, const Eigen::Quaterniond& current_orientation);
  Eigen::Matrix<double, 6, 7> getEndEffectorJacobian();
};

}  // namespace fr3_controllers
