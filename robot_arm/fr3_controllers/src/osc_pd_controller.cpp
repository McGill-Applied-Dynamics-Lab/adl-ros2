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

#include <Eigen/Eigen>
#include <cassert>
#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <exception>
#include <fr3_controllers/osc_pd_controller.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace fr3_controllers
{

controller_interface::InterfaceConfiguration OSCPDController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i)
  {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration OSCPDController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Joint position and velocity interfaces
  for (int i = 1; i <= num_joints; ++i)
  {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }

  // Robot model interfaces
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names())
  {
    config.names.push_back(franka_robot_model_name);
  }

  config.names.push_back(arm_id_ + "/robot_time");

  //   config.names.push_back(arm_id_ + "/" + k_robot_model_interface_name);
  //   config.names.push_back(arm_id_ + "/" + k_robot_state_interface_name);

  return config;
}

controller_interface::return_type OSCPDController::update(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& /*period*/)
{
  // Don't send commands until we receive a goal
  if (!goal_received_)
  {
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                         "Waiting for goal to be received. No commands will be sent until a goal is received.");
    for (auto& command_interface : command_interfaces_)
    {
      command_interface.set_value(0.0);
    }

    return controller_interface::return_type::OK;
  }

  // Update joint states
  updateJointStates();

  // Get current end-effector pose
  std::array<double, 16> pose_array = franka_robot_model_->getPoseMatrix(franka::Frame::kEndEffector);

  // Convert pose matrix to position
  Eigen::Matrix4d pose_matrix;
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      pose_matrix(i, j) = pose_array[j * 4 + i];  // Column-major to row-major
    }
  }

  Eigen::Vector3d current_position = pose_matrix.block<3, 1>(0, 3);

  // Compute position error
  Vector3d position_error = computePositionError(current_position);

  // Get position Jacobian (first 3 rows of end-effector Jacobian)
  Eigen::Matrix<double, 3, 7> jacobian = getPositionJacobian();

  // Compute Cartesian velocity (J * q_dot) - only position part
  Vector3d cartesian_velocity = jacobian * dq_;

  // Compute desired Cartesian force using PD control law
  Vector3d cartesian_force = kp_gain_ * position_error - kd_gain_ * cartesian_velocity;

  // Convert to joint torques: tau = J^T * F_cartesian
  Vector7d tau_d = jacobian.transpose() * cartesian_force;

  // Apply torque commands
  for (int i = 0; i < num_joints; ++i)
  {
    command_interfaces_[i].set_value(tau_d(i));
  }

  //! Debugging: Printing
  //   // Print joint states
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Joint positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", q_(0), q_(1), q_(2), q_(3),
  //                        q_(4), q_(5), q_(6));

  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Joint velocities: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", dq_(0), dq_(1), dq_(2),
  //                        dq_(3), dq_(4), dq_(5), dq_(6));

  //   // Print current and desired positions
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Current position: [%.3f, %.3f,
  //   %.3f]",
  //                        current_position.x(), current_position.y(), current_position.z());

  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Desired position: [%.3f, %.3f,
  //   %.3f]",
  //                        position_des_.x(), position_des_.y(), position_des_.z());

  //   // Print position error
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Position error: [%.3f, %.3f,
  //   %.3f]",
  //                        position_error.x(), position_error.y(), position_error.z());

  //   // Print Jacobian (first row only to save space)
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Jacobian row 0: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", jacobian(0, 0), jacobian(0,
  //                        1), jacobian(0, 2), jacobian(0, 3), jacobian(0, 4), jacobian(0, 5), jacobian(0, 6));

  //   // Print Cartesian velocity
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Cartesian velocity: [%.3f, %.3f, %.3f]", cartesian_velocity.x(), cartesian_velocity.y(),
  //                        cartesian_velocity.z());
  //   // Print Cartesian force
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Cartesian force: [%.3f, %.3f,
  //   %.3f]",
  //                        cartesian_force.x(), cartesian_force.y(), cartesian_force.z());

  // Print commanded torques
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "Commanded torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", tau_d(0), tau_d(1), tau_d(2),
                       tau_d(3), tau_d(4), tau_d(5), tau_d(6));

  return controller_interface::return_type::OK;
}

CallbackReturn OSCPDController::on_init()
{
  try
  {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("kp_gain", 100.0);                              // Default proportional gain
    auto_declare<double>("kd_gain", 10.0);                               // Default derivative gain
    auto_declare<std::string>("goal_topic", "/osc_pd_controller/goal");  // Topic for desired position
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn OSCPDController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  arm_id_ = get_node()->get_parameter("arm_id").as_string();

  // Configure gains
  kp_gain_ = get_node()->get_parameter("kp_gain").as_double();
  kd_gain_ = get_node()->get_parameter("kd_gain").as_double();

  if (kp_gain_ <= 0.0)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kp_gain must be positive, got: %f", kp_gain_);
    return CallbackReturn::FAILURE;
  }
  if (kd_gain_ < 0.0)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kd_gain must be positive, got: %f", kd_gain_);
    return CallbackReturn::FAILURE;
  }

  // Initialize goal state
  goal_received_ = false;
  position_des_.setZero();

  // Create goal subscription
  std::string goal_topic = get_node()->get_parameter("goal_topic").as_string();
  goal_sub_ = get_node()->create_subscription<geometry_msgs::msg::PointStamped>(
      goal_topic, 10, std::bind(&OSCPDController::goalCallback, this, std::placeholders::_1));

  // Initialize robot model
  franka_robot_model_ =
      std::make_unique<franka_semantic_components::FrankaRobotModel>(franka_semantic_components::FrankaRobotModel(
          arm_id_ + "/" + k_robot_model_interface_name, arm_id_ + "/" + k_robot_state_interface_name));

  RCLCPP_INFO(get_node()->get_logger(), "OSC PD Controller configured successfully");
  RCLCPP_INFO(get_node()->get_logger(), "Gains: kp=%.2f, kd=%.2f", kp_gain_, kd_gain_);
  RCLCPP_INFO(get_node()->get_logger(), "Listening for goals on topic: %s", goal_topic.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn OSCPDController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Extract robot model interfaces (they come after joint interfaces)
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> robot_model_interfaces;

  // Joint interfaces are first (2 * num_joints), robot model interfaces come after
  size_t joint_interface_count = 2 * num_joints;
  for (size_t i = joint_interface_count; i < state_interfaces_.size(); ++i)
  {
    robot_model_interfaces.push_back(std::ref(state_interfaces_[i]));
  }

  // Assign robot model interfaces
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // Initialize joint states
  updateJointStates();

  RCLCPP_INFO(get_node()->get_logger(), "OSC PD Controller activated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn OSCPDController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  franka_robot_model_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void OSCPDController::updateJointStates()
{
  for (auto i = 0; i < num_joints; ++i)
  {
    const auto& position_interface = state_interfaces_.at(2 * i);
    const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");

    q_(i) = position_interface.get_value();
    dq_(i) = velocity_interface.get_value();
  }
}

void OSCPDController::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  position_des_ << msg->point.x, msg->point.y, msg->point.z;
  goal_received_ = true;

  RCLCPP_INFO(get_node()->get_logger(), "Received new goal: [%.3f, %.3f, %.3f]", position_des_.x(), position_des_.y(),
              position_des_.z());
}

OSCPDController::Vector3d OSCPDController::computePositionError(const Eigen::Vector3d& current_position)
{
  return position_des_ - current_position;
}

Eigen::Matrix<double, 3, 7> OSCPDController::getPositionJacobian()
{
  // Get the full end-effector Jacobian from the robot model
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  // Convert to Eigen matrix and extract only position part (first 3 rows)
  Eigen::Matrix<double, 3, 7> jacobian;
  for (int i = 0; i < 3; ++i)
  {  // Only position rows (0, 1, 2)
    for (int j = 0; j < 7; ++j)
    {
      jacobian(i, j) = jacobian_array[j * 6 + i];  // Column-major storage, skip orientation rows
    }
  }

  return jacobian;
}

}  // namespace fr3_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::OSCPDController, controller_interface::ControllerInterface)
