// Copyright (c) 2026 Franka Robotics GmbH
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

#include "franka_example_controllers/tmr/mobile_cartesian_velocity_with_ik_example_controller.hpp"
#include "franka_example_controllers/tmr/swerve_ik.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace franka_example_controllers {

controller_interface::CallbackReturn MobileCartesianVelocityWithIkExampleController::on_init() {
  RCLCPP_INFO(rclcpp::get_logger("MobileCartesianVelocityWithIkExampleController"),
              "Controller initialized");

  // Set wheel positions relative to base
  wheel_positions_ << 0.3, -0.2, -0.3, 0.2;

  steering_angles_.setZero();
  wheel_velocities_.setZero();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MobileCartesianVelocityWithIkExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
      mobile_robot_type_ + "_joint_0/position", mobile_robot_type_ + "_joint_1/velocity",
      mobile_robot_type_ + "_joint_2/position", mobile_robot_type_ + "_joint_3/velocity"};
  return config;
}

controller_interface::InterfaceConfiguration
MobileCartesianVelocityWithIkExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::return_type MobileCartesianVelocityWithIkExampleController::update(
    const rclcpp::Time&,
    const rclcpp::Duration& period) {
  elapsed_time_ += period.seconds();

  // Sinusoidal Cartesian velocities
  double vx = vx_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);
  double vy = vy_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);
  double wz = wz_amp_ * std::sin(2.0 * M_PI * freq_ * elapsed_time_);

  // Compute swerve IK
  franka_example_controllers::computeSwerveIK(vx, vy, wz, wheel_positions_, wheel_radius_,
                                              steering_angles_, wheel_velocities_, commands_);

  // Send to command interfaces
  command_interfaces_[0].set_value(commands_[0].steering_angle);
  command_interfaces_[1].set_value(commands_[0].wheel_velocity);
  command_interfaces_[2].set_value(commands_[1].steering_angle);
  command_interfaces_[3].set_value(commands_[1].wheel_velocity);

  return controller_interface::return_type::OK;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MobileCartesianVelocityWithIkExampleController,
                       controller_interface::ControllerInterface)
