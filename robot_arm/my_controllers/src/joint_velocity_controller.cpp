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

#include "my_controllers/joint_velocity_controller.hpp"

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <exception>
#include <my_controllers/default_robot_behavior_utils.hpp>
#include <my_controllers/robot_utils.hpp>
#include <string>

double clamp(double desired, double lower, double upper) { return std::max(lower, std::min(desired, upper)); }
namespace my_controllers {

CallbackReturn JointVelocityController::on_init() {
  try {
    auto_declare<bool>("gazebo", false);
    auto_declare<std::string>("robot_description", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration JointVelocityController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointVelocityController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn JointVelocityController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  //! Parameters
  is_gazebo = get_node()->get_parameter("gazebo").as_bool();

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty()) {
    robot_description_ = result[0].value_to_string();
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  //! Services
  if (!is_gazebo) {
    auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
        "service_server/set_full_collision_behavior");
    auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

    auto future_result = client->async_send_request(request);
    future_result.wait_for(robot_utils::time_out);

    auto success = future_result.get();
    if (!success) {
      RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  //! Subscriber
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
      "~/commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { rt_command_ptr_.writeFromNonRT(msg); });

  //! Initialize joint states
  qd_goal_ << 0, 0, 0, 0, 0, 0, 0;
  qd_filtered_.setZero();

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointVelocityController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();

  elapsed_time_ = rclcpp::Duration(0, 0);
  qd_filtered_ = qd_;

  // reset command buffer if a command came through callback when controller was inactive
  rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointVelocityController::update(const rclcpp::Time&, const rclcpp::Duration& period) {
  auto joint_commands = rt_command_ptr_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands)) {
    return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                          "command size (%zu) does not match number of interfaces (%zu)",
                          (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (int i = 0; i < 7; ++i) {
    qd_goal_(i) = (*joint_commands)->data[i];
  }

  updateJointStates();
  elapsed_time_ = elapsed_time_ + period;

  // const double kAlpha = 0.99;
  // qd_filtered_ = (1 - kAlpha) * qd_filtered_ + kAlpha * qd_;

  for (int i = 0; i < 7; ++i) {
    // Calculate max allowed velocity change
    double delta_v = max_accel_ * period.seconds();

    // Smooth towards desired velocity
    qd_filtered_(i) = clamp(qd_goal_(i), qd_filtered_(i) - delta_v, qd_filtered_(i) + delta_v);

    command_interfaces_[i].set_value(qd_filtered_(i));
  }

  RCLCPP_INFO(get_node()->get_logger(), "Joint %d: qd = %f \t qd_des = %f", 3, qd_(6), qd_filtered_(6));
  // RCLCPP_INFO(get_node()->get_logger(), "period %f", 1, period.seconds());

  return controller_interface::return_type::OK;
}

void JointVelocityController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    qd_(i) = velocity_interface.get_value();
  }
}

}  // namespace my_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(my_controllers::JointVelocityController, controller_interface::ControllerInterface)
