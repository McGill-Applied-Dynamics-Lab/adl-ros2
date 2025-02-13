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
#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_controllers
{
using CmdType = geometry_msgs::msg::PoseStamped;

/**
 * The cartesian pose example controller
 */
class CartesianPoseController : public controller_interface::ControllerInterface
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
  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  double elapsed_time_{0.0};
  double initial_robot_time_{0.0};
  double robot_time_{0.0};
  std::string robot_description_;
  std::string arm_id_;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<CmdType>::SharedPtr command_subscriber_ = nullptr;

  realtime_tools::RealtimeBox<std::shared_ptr<CmdType>> received_command_msg_ptr_{nullptr};
  std::shared_ptr<CmdType> last_command_msg_;
};

}  // namespace fr3_controllers
