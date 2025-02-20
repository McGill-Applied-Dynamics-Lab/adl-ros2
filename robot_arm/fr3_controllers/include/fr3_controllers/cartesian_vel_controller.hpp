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

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <fr3_controllers/robot_utils.hpp>
#include <franka_semantic_components/franka_cartesian_velocity_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "geometry_msgs/msg/twist_stamped.hpp"
// #include "motion_generator.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers
{
using CmdType = geometry_msgs::msg::TwistStamped;
using Vector3d = Eigen::Matrix<double, 3, 1>;

/**
 * The cartesian pose controller
 */
class CartesianVelController : public controller_interface::ControllerInterface
{
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

 private:
  void initialize_controller();
  void update_desired_command();
  void update_current_state(double dt);
  void print_robot_states();
  Vector3d clamp(Vector3d value, Vector3d min_val, Vector3d max_val);

  std::unique_ptr<franka_semantic_components::FrankaCartesianVelocityInterface> franka_cartesian_velocity_;

  const bool k_elbow_activated_{false};
  bool to_initialize_flag_{true};
  bool new_cmd_msg_{false};

  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;

  const double T_blend_ = 0.1;  // e.g., 100 ms blending horizon

  //! Robot states
  Vector3d x_ee_start_;  // Initial end-effector position
  Eigen::Quaterniond start_orientation_;

  Vector3d x_ee_des_;     // Desired end-effector position. From the command message
  Vector3d x_ee_;         // Current end-effector position
  Vector3d x_ee_prev_;    // Prev end-effector position
  Vector3d x_ee_c_;       // Commanded end-effector position
  Vector3d x_ee_c_prev_;  // Prev commanded end-effector position

  Vector3d dx_ee_des_;   // Desired end-effector velocity
  Vector3d dx_ee_;       // Current end-effector velocity
  Vector3d dx_ee_prev_;  // Current end-effector velocity
  Vector3d dx_ee_c_;     // Commanded end-effector velocity

  Vector3d ddx_ee_;    // Current end-effector acceleration
  Vector3d ddx_ee_c_;  // Commanded end-effector acceleration

  Vector3d dddx_ee_c_;  // Commanded end-effector jerk

  //! ROS Interfaces (sub, pub, ...)
  bool subscriber_is_active_ = false;
  rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<CmdType>> received_command_msg_ptr_{nullptr};
  std::shared_ptr<CmdType> last_command_msg_;

  //! Limits (of end-effector)
  // TODO: Update to real max values
  Vector3d dx_ee_max_ = (Vector3d() << 2.0, 2.0, 2.0).finished();       // in m/s
  Vector3d ddx_ee_max_ = (Vector3d() << 4.5, 4.5, 4.5).finished();      // in m/s^2
  Vector3d dddx_ee_max_ = (Vector3d() << 1000, 1000, 1000).finished();  // in m/s^3
};

}  // namespace fr3_controllers
