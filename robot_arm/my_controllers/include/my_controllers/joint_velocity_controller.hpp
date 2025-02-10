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

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace my_controllers {

/**
 * The joint velocity example controller
 */
class JointVelocityController : public controller_interface::ControllerInterface {
 public:
  [[nodiscard]] auto command_interface_configuration() const -> controller_interface::InterfaceConfiguration override;
  [[nodiscard]] auto state_interface_configuration() const -> controller_interface::InterfaceConfiguration override;
  auto update(const rclcpp::Time& time, const rclcpp::Duration& period) -> controller_interface::return_type override;
  auto on_init() -> CallbackReturn override;
  auto on_configure(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;
  auto on_activate(const rclcpp_lifecycle::State& previous_state) -> CallbackReturn override;

 private:
  std::string arm_id_;
  std::string robot_description_;
  bool is_gazebo{false};
  const int num_joints = 7;
  rclcpp::Duration elapsed_time_ = rclcpp::Duration(0, 0);
};

}  // namespace my_controllers
