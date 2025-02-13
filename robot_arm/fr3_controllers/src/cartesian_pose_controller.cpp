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

#include <cassert>
#include <cmath>
#include <exception>
#include <fr3_controllers/cartesian_pose_controller.hpp>
#include <fr3_controllers/default_robot_behavior_utils.hpp>
#include <string>

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/commands";
// constexpr auto DEFAULT_ACKERMANN_OUT_TOPIC = "~/cmd_ackermann";
// constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
// constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
// constexpr auto DEFAULT_RESET_ODOM_SERVICE = "~/reset_odometry";
}  // namespace

namespace fr3_controllers
{

CallbackReturn CartesianPoseController::on_init()
{
  franka_cartesian_pose_ = std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
      franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  //! Set default collision behavior
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(robot_utils::time_out);

  auto success = future_result.get();
  if (!success)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  }
  else
  {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  //! Parameters
  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(get_node(), "/robot_state_publisher");
  parameters_client->wait_for_service();

  auto future = parameters_client->get_parameters({"robot_description"});
  auto result = future.get();
  if (!result.empty())
  {
    robot_description_ = result[0].value_to_string();
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description parameter.");
  }

  arm_id_ = robot_utils::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

  //! Subscriber
  const CmdType empty_cmd = []()
  {
    CmdType cmd;
    cmd.header.stamp = rclcpp::Time(0, 0);
    cmd.pose.position.x = 0.0;
    cmd.pose.position.y = 0.0;
    cmd.pose.position.z = 0.0;
    cmd.pose.orientation.x = 0.0;
    cmd.pose.orientation.y = 0.0;
    cmd.pose.orientation.z = 0.0;
    cmd.pose.orientation.w = 1.0;
    return cmd;
  }();

  // CmdType empty_cmd;
  // empty_cmd.data.resize(7, 0.0);
  received_command_msg_ptr_.set(std::make_shared<CmdType>(empty_cmd));

  // last_command_msg_ = std::make_shared<CmdType>();
  // received_command_msg_ptr_.set([this](std::shared_ptr<CmdType>& stored_value) { stored_value = last_command_msg_;
  // });

  auto cmd_callback =
      [this](const std::shared_ptr<CmdType> msg) -> void  //{ input_joint_command_.writeFromNonRT(msg); };
  {
    if (!subscriber_is_active_)
    {
      RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
      return;
    }
    if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
    {
      RCLCPP_WARN_ONCE(get_node()->get_logger(),
                       "Received TwistStamped with zero timestamp, setting it to current "
                       "time, this message will only be shown once");
      msg->header.stamp = get_node()->get_clock()->now();
    }
    received_command_msg_ptr_.set(std::move(msg));
  };

  // [this](const std::shared_ptr<CmdType> msg) -> void {
  //   if (!subscriber_is_active_) {
  //     RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
  //     return;
  //   }
  //   // if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
  //   //   RCLCPP_WARN_ONCE(get_node()->get_logger(),
  //   //                    "Received TwistStamped with zero timestamp, setting it to current "
  //   //                    "time, this message will only be shown once");
  //   //   msg->header.stamp = get_node()->get_clock()->now();
  //   // }
  //   received_command_msg_ptr_.set([msg](std::shared_ptr<CmdType>& stored_value) { stored_value =
  //   std::move(msg); });
  // }

  // initialize command subscriber
  command_subscriber_ =
      get_node()->create_subscription<CmdType>(DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(), cmd_callback);

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration CartesianPoseController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration CartesianPoseController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  // add the robot time interface
  config.names.push_back(arm_id_ + "/robot_time");
  return config;
}

controller_interface::return_type CartesianPoseController::update(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& /*period*/)
{
  if (initialization_flag_)
  {
    // Get initial orientation and translation
    std::tie(orientation_, position_) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
    initial_robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = 0.0;

    initialization_flag_ = false;
  }
  else
  {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }

  //! Get last command
  std::shared_ptr<CmdType> last_command_msg;
  received_command_msg_ptr_.get(last_command_msg);

  // double radius = 0.1;
  // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_));

  // double delta_x = radius * std::sin(angle);
  // double delta_z = radius * (std::cos(angle) - 1);

  Eigen::Quaterniond new_orientation;
  Eigen::Vector3d new_position;

  new_position = position_;
  new_orientation = orientation_;

  // new_position(0) -= delta_x;
  // new_position(2) -= delta_z;
  // if (elapsed_time_ > 2.0) {
  //   new_position(0) += 0.2;
  // }

  if (last_command_msg)
  {
    auto pose = last_command_msg->pose;
    RCLCPP_INFO(get_node()->get_logger(), "Received command: position (%f, %f, %f)", pose.position.x, pose.position.y,
                pose.position.z);
  }

  if (franka_cartesian_pose_->setCommand(new_orientation, new_position))
  {
    return controller_interface::return_type::OK;
  }
  else
  {
    RCLCPP_FATAL(get_node()->get_logger(), "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianPoseController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  subscriber_is_active_ = true;

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_cartesian_pose_->release_interfaces();

  subscriber_is_active_ = false;

  return CallbackReturn::SUCCESS;
}

}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::CartesianPoseController, controller_interface::ControllerInterface)
