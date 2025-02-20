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
Vector3d clamp(Vector3d value, Vector3d min_val, Vector3d max_val)
{
  return value.array().max(min_val.array()).min(max_val.array()).matrix();
}

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

  received_command_msg_ptr_.set(std::make_shared<CmdType>(empty_cmd));

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
    new_cmd_msg_ = true;
  };

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

CallbackReturn CartesianPoseController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  to_initialize_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  subscriber_is_active_ = true;

  // motion_generator_ = std::make_unique<MotionGenerator>(0.2, q_, q_goal_);

  // std::tie(start_orientation_, start_position_) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();

  // CmdType start_pose;
  // start_pose.pose.position.x = start_position_(0);
  // start_pose.pose.position.y = start_position_(1);
  // start_pose.pose.position.z = start_position_(2);

  // received_command_msg_ptr_.set(std::make_shared<CmdType>(start_pose));

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_cartesian_pose_->release_interfaces();

  subscriber_is_active_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianPoseController::update(const rclcpp::Time& /*time*/,
                                                                  const rclcpp::Duration& period)
{
  if (to_initialize_flag_)
  {
    initialize_controller();
    new_cmd_msg_ = true;  // To update the desired pose to current, even if no message
  }
  else
  {
    robot_time_ = state_interfaces_.back().get_value();
    elapsed_time_ = robot_time_ - initial_robot_time_;
  }
  double dt = period.seconds();

  update_desired_command();
  update_current_state(dt);

  Eigen::Quaterniond new_orientation;
  Eigen::Vector3d new_position;

  // new_position = commanded_position;
  new_orientation = start_orientation_;

  // //? Their example
  // double radius = 0.2;
  // double angle = M_PI / 4 * (1 - std::cos(M_PI / 3.0 * elapsed_time_));
  //
  // double delta_x = radius * std::sin(angle);
  // double delta_z = radius * (std::cos(angle) - 1);
  //
  // new_position = start_position_;
  // new_orientation = start_orientation_;
  //
  // new_position(0) -= delta_x;
  // new_position(2) -= delta_z;

  //? From subscriber

  // //! Method 1 - Clamped Integration
  // Vector3d x_ee_err;
  // x_ee_err.setZero();
  // x_ee_err(0) = x_ee_des_(0) - x_ee_c_(0);
  //
  // // 3. Compute a “desired” velocity to reduce the error.
  // // Here we use a simple proportional idea (but you must also clamp it)
  // Vector3d desired_vel = x_ee_err / dt;  // crude approximation
  // // RCLCPP_INFO(get_node()->get_logger(), "desired_vel: %f", desired_vel(0));
  //
  // desired_vel = clamp(desired_vel, -dx_ee_max_, dx_ee_max_);
  // // RCLCPP_INFO(get_node()->get_logger(), "desired_vel, clamped: %f", desired_vel(0));
  //
  // // 4. Compute the acceleration needed to reach desired_vel.
  // Vector3d desired_acc = (desired_vel - dx_ee_c_) / dt;
  // // RCLCPP_INFO(get_node()->get_logger(), "desired_acc: %f", desired_acc(0));
  //
  // desired_acc = clamp(desired_acc, -ddx_ee_max_, ddx_ee_max_);
  // // RCLCPP_INFO(get_node()->get_logger(), "desired_acc, clamped: %f", desired_acc(0));
  //
  // // 5. Limit the jerk. Compute the change in acceleration and clamp it.
  // Vector3d des_jerk = (desired_acc - ddx_ee_c_) / dt;
  // // RCLCPP_INFO(get_node()->get_logger(), "acc_des, ddx_ee_c_, dt, des_jerk: %f, %f, %f, %f", desired_acc(0),
  // //             ddx_ee_c_(0), dt, des_jerk(0));
  //
  // des_jerk = clamp(des_jerk, -dddx_ee_max_, dddx_ee_max_);
  // // RCLCPP_INFO(get_node()->get_logger(), "desired_jerk, clamped: %f", des_jerk(0));

  //! Method 2 - Blending traj
  Vector3d x_ee_err = x_ee_des_ - x_ee_c_;
  x_ee_err(1) = 0.0;  // TODO: Remove this line
  x_ee_err(2) = 0.0;  // TODO: Remove this line

  // Compute vel from error
  Vector3d vel_from_err = x_ee_err / T_blend_;

  // Now, blend from the current velocity toward that "ideal" velocity.
  // This gives you a desired velocity that gradually tracks the target without snapping.
  Eigen::Vector3d new_vel = dx_ee_c_ + (vel_from_err - dx_ee_c_) * (dt / T_blend_);

  // Clamp
  new_vel = clamp(new_vel, -dx_ee_max_, dx_ee_max_);

  Vector3d new_acc = (new_vel - dx_ee_c_) / dt;
  new_acc = clamp(new_acc, -ddx_ee_max_, ddx_ee_max_);

  Vector3d new_jerk = (new_acc - ddx_ee_c_) / dt;
  new_jerk = clamp(new_jerk, -dddx_ee_max_, dddx_ee_max_);

  // 7. Integrate to update the velocity and then the position.
  dddx_ee_c_ = new_jerk;
  ddx_ee_c_ += new_jerk * dt;
  dx_ee_c_ += ddx_ee_c_ * dt;
  x_ee_c_ += dx_ee_c_ * dt;

  //? OLD TRIES
  // auto dx = pose.position.x;
  // new_position(0) = desired_pose.position.x;
  // new_position(1) = desired_pose.position.y;
  // new_position(2) = desired_pose.position.z;
  //
  // const double kAlpha = 0.5;
  // new_position(0) = kAlpha * (desired_pose.position.x) +
  //                   (1 - kAlpha) * commanded_position(0);  // alpha*x_i + (1 - alpha) * y_{i - 1}
  //
  // auto vel = (commanded_position(0) - new_position(0)) / 0.001;

  // RCLCPP_INFO(get_node()->get_logger(),
  //             "(x_des, x, x_c', x_c, x_d, x_d_c, x_dd, x_dd_c): (%f, %f, %f, %f, %f, %f, %f, %f)", x_ee_des_(0),
  //             x_ee_(0), x_ee_c_prev_(0), x_ee_c_(0), dx_ee_(0), desired_vel(0), ddx_ee_(0), desired_acc(0));

  // RCLCPP_INFO(get_node()->get_logger(), "(x_des, x, err, x_c, x_d_c, x_dd_c): (%f, %f, %f, %f, %f, %f)",
  // x_ee_des_(0),
  //             x_ee_(0), x_ee_err(0), x_ee_c_(0), dx_ee_c_(0), desired_acc(0));

  // Update prev values
  x_ee_prev_ = x_ee_;
  dx_ee_prev_ = dx_ee_;
  // dx_ee_c_ = desired_vel;
  // ddx_ee_c_ = desired_acc;

  print_robot_states();

  // Send
  new_position(0) = x_ee_c_(0);
  new_position(1) = x_ee_c_(1);
  new_position(2) = x_ee_c_(2);

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

void CartesianPoseController::initialize_controller()
{
  // Get initial orientation and translation
  std::tie(start_orientation_, x_ee_start_) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  initial_robot_time_ = state_interfaces_.back().get_value();
  elapsed_time_ = 0.0;

  // Init command message
  CmdType start_pose;
  start_pose.pose.position.x = x_ee_start_(0);
  start_pose.pose.position.y = x_ee_start_(1);
  start_pose.pose.position.z = x_ee_start_(2);

  RCLCPP_INFO(get_node()->get_logger(), "Start (x, y, z): (%f, %f, %f)", x_ee_start_(0), x_ee_start_(1),
              x_ee_start_(2));

  received_command_msg_ptr_.set(std::make_shared<CmdType>(start_pose));

  // Init robot states
  x_ee_ = x_ee_start_;
  dx_ee_ = x_ee_start_;
  x_ee_c_ = x_ee_start_;

  x_ee_prev_ = x_ee_;
  dx_ee_prev_.setZero();

  dx_ee_.setZero();
  dx_ee_c_.setZero();
  ddx_ee_.setZero();
  ddx_ee_c_.setZero();
  dddx_ee_c_.setZero();

  to_initialize_flag_ = false;
}

void CartesianPoseController::update_desired_command()
{
  if (!new_cmd_msg_)
  {
    return;
  }
  //! Get last command
  std::shared_ptr<CmdType> last_command_msg;
  received_command_msg_ptr_.get(last_command_msg);

  x_ee_des_(0) = last_command_msg->pose.position.x;
  x_ee_des_(1) = last_command_msg->pose.position.y;
  x_ee_des_(2) = last_command_msg->pose.position.z;

  // TODO: Check if this is needed
  //  I think this could be the same value I store from the prev call
  Eigen::Quaterniond commanded_orientation;
  Eigen::Vector3d commanded_position;

  std::tie(commanded_orientation, commanded_position) = franka_cartesian_pose_->getCommandedOrientationAndTranslation();
  x_ee_c_prev_(0) = commanded_position(0);
  x_ee_c_prev_(1) = commanded_position(1);
  x_ee_c_prev_(2) = commanded_position(2);

  new_cmd_msg_ = false;

  RCLCPP_INFO(get_node()->get_logger(), "Des ee (x, y, z): (%f, %f, %f)", x_ee_des_(0), x_ee_des_(1), x_ee_des_(2));
}

void CartesianPoseController::update_current_state(double dt)
{
  Eigen::Quaterniond current_orientation;
  Eigen::Vector3d current_position;

  std::tie(current_orientation, current_position) = franka_cartesian_pose_->getCurrentOrientationAndTranslation();
  x_ee_(0) = current_position(0);
  x_ee_(1) = current_position(1);
  x_ee_(2) = current_position(2);

  dx_ee_ = (x_ee_ - x_ee_prev_) / dt;
  ddx_ee_ = (dx_ee_ - dx_ee_prev_) / dt;

  // x_ee_c_ = x_ee_c_prev_;
  // dx_ee_c_ = dx_ee_;
  // ddx_ee_c_ = ddx_ee_;
}

void CartesianPoseController::print_robot_states()
{
  RCLCPP_INFO(get_node()->get_logger(), "Robot States:");
  // RCLCPP_INFO(get_node()->get_logger(), "\t--- Measured ---");
  // RCLCPP_INFO(get_node()->get_logger(), "\tx_ee_:     \t(%10.6f, %10.6f, %10.6f)", x_ee_(0), x_ee_(1), x_ee_(2));
  // RCLCPP_INFO(get_node()->get_logger(), "\tdx_ee_:    \t(%10.6f, %10.6f, %10.6f)", dx_ee_(0), dx_ee_(1), dx_ee_(2));
  // RCLCPP_INFO(get_node()->get_logger(), "\tddx_ee_:   \t(%10.6f, %10.6f, %10.6f)", ddx_ee_(0), ddx_ee_(1),
  // ddx_ee_(2));

  // RCLCPP_INFO(get_node()->get_logger(), "\t--- Command ---");
  RCLCPP_INFO(get_node()->get_logger(), "\tx_ee_des:  \t(%10.6f, %10.6f, %10.6f)", x_ee_des_(0), x_ee_des_(1),
              x_ee_des_(2));
  RCLCPP_INFO(get_node()->get_logger(), "\tx_ee_c_:   \t(%10.6f, %10.6f, %10.6f)", x_ee_c_(0), x_ee_c_(1), x_ee_c_(2));
  RCLCPP_INFO(get_node()->get_logger(), "\tdx_ee_c_:  \t(%10.6f, %10.6f, %10.6f)", dx_ee_c_(0), dx_ee_c_(1),
              dx_ee_c_(2));
  RCLCPP_INFO(get_node()->get_logger(), "\tddx_ee_c_: \t(%10.6f, %10.6f, %10.6f)", ddx_ee_c_(0), ddx_ee_c_(1),
              ddx_ee_c_(2));
  RCLCPP_INFO(get_node()->get_logger(), "\tdddx_ee_c_:\t(%10.6f, %10.6f, %10.6f)", dddx_ee_c_(0), dddx_ee_c_(1),
              dddx_ee_c_(2));
}

}  // namespace fr3_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::CartesianPoseController, controller_interface::ControllerInterface)
