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
  Eigen::Matrix3d rotation_matrix = pose_matrix.block<3, 3>(0, 0);
  Eigen::Quaterniond current_orientation(rotation_matrix);

  //   // Initialize desired orientation to current orientation on first goal
  //   if (goal_received_ && !orientation_initialized_)
  //   {
  //     orientation_des_ = current_orientation;
  //     orientation_initialized_ = true;
  //     RCLCPP_INFO(get_node()->get_logger(),
  //                 "Initialized desired orientation to current: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]",
  //                 orientation_des_.w(), orientation_des_.x(), orientation_des_.y(), orientation_des_.z());
  //   }

  // Compute Cartesian error (position + orientation)
  Vector6d cartesian_error = computeError(current_position, current_orientation);

  // Get full end-effector Jacobian (6x7)
  Eigen::Matrix<double, 6, 7> jacobian = getEndEffectorJacobian();

  // Compute Cartesian velocity (J * q_dot) - full 6DOF
  Vector6d cartesian_velocity = jacobian * dq_;

  // Compute desired Cartesian force/torque using PD control law with separate gains
  Vector6d cartesian_force;

  // Position control (first 3 elements)
  cartesian_force.head<3>() = kp_pos_gain_ * cartesian_error.head<3>() - kd_pos_gain_ * cartesian_velocity.head<3>();

  // Orientation control (last 3 elements)
  cartesian_force.tail<3>() = kp_ori_gain_ * cartesian_error.tail<3>() - kd_ori_gain_ * cartesian_velocity.tail<3>();

  // Convert to joint torques: tau = J^T * F_cartesian
  Vector7d tau_d = jacobian.transpose() * cartesian_force;

  // Apply torque commands
  for (int i = 0; i < num_joints; ++i)
  {
    command_interfaces_[i].set_value(tau_d(i));
  }

  //! Debugging: Printing
  //   // Print current and desired positions
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Current position: [%.3f, %.3f,
  //   %.3f]",
  //                        current_position.x(), current_position.y(), current_position.z());
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "Desired position: [%.3f, %.3f,
  //   %.3f]",
  //                        position_des_.x(), position_des_.y(), position_des_.z());

  //   // Print current and desired orientations (if initialized)
  //   if (orientation_initialized_)
  //   {
  //     RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                          "Current orientation: [w=%.3f, x=%.3f, y=%.3f, z=%.3f]", current_orientation.w(),
  //                          current_orientation.x(), current_orientation.y(), current_orientation.z());
  //   }

  // Print Cartesian error (position + orientation)
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                       "Cartesian error (pos+ori): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", cartesian_error(0),
                       cartesian_error(1), cartesian_error(2), cartesian_error(3), cartesian_error(4),
                       cartesian_error(5));

  //   // Print Cartesian force/torque
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Cartesian force/torque: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", cartesian_force(0),
  //                        cartesian_force(1), cartesian_force(2), cartesian_force(3), cartesian_force(4),
  //                        cartesian_force(5));

  //   // Print commanded torques
  //   RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
  //                        "Commanded torques: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]", tau_d(0), tau_d(1),
  //                        tau_d(2), tau_d(3), tau_d(4), tau_d(5), tau_d(6));

  return controller_interface::return_type::OK;
}

CallbackReturn OSCPDController::on_init()
{
  try
  {
    auto_declare<std::string>("arm_id", "fr3");
    auto_declare<double>("kp_pos_gain", 100.0);                          // Default proportional gain for position
    auto_declare<double>("kd_pos_gain", 10.0);                           // Default derivative gain for position
    auto_declare<double>("kp_ori_gain", 50.0);                           // Default proportional gain for orientation
    auto_declare<double>("kd_ori_gain", 5.0);                            // Default derivative gain for orientation
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
  kp_pos_gain_ = get_node()->get_parameter("kp_pos_gain").as_double();
  kd_pos_gain_ = get_node()->get_parameter("kd_pos_gain").as_double();
  kp_ori_gain_ = get_node()->get_parameter("kp_ori_gain").as_double();
  kd_ori_gain_ = get_node()->get_parameter("kd_ori_gain").as_double();

  if (kp_pos_gain_ <= 0.0)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kp_pos_gain must be positive, got: %f", kp_pos_gain_);
    return CallbackReturn::FAILURE;
  }
  if (kd_pos_gain_ < 0.0)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kd_pos_gain must be non-negative, got: %f", kd_pos_gain_);
    return CallbackReturn::FAILURE;
  }
  if (kp_ori_gain_ <= 0.0)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kp_ori_gain must be positive, got: %f", kp_ori_gain_);
    return CallbackReturn::FAILURE;
  }
  if (kd_ori_gain_ < 0.0)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kd_ori_gain must be non-negative, got: %f", kd_ori_gain_);
    return CallbackReturn::FAILURE;
  }

  // Initialize goal state
  goal_received_ = false;
  orientation_initialized_ = false;
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
  RCLCPP_INFO(get_node()->get_logger(), "Position gains: kp=%.2f, kd=%.2f", kp_pos_gain_, kd_pos_gain_);
  RCLCPP_INFO(get_node()->get_logger(), "Orientation gains: kp=%.2f, kd=%.2f", kp_ori_gain_, kd_ori_gain_);
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

  // Set desired orientation to point down (identity quaternion)
  orientation_des_ = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);  // Pointing downards

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

  //   RCLCPP_INFO(get_node()->get_logger(), "Received new goal: [%.3f, %.3f, %.3f]", position_des_.x(),
  //   position_des_.y(),
  //               position_des_.z());
}

OSCPDController::Vector6d OSCPDController::computeError(const Eigen::Vector3d& current_position,
                                                        const Eigen::Quaterniond& current_orientation)
{
  Vector6d error;

  // Position error
  error.head<3>() = position_des_ - current_position;

  //! Option 1 - Error with quaternions
  //   // Orientation error using quaternion difference
  //   Eigen::Quaterniond orientation_error = orientation_des_ * current_orientation.inverse();

  //   // Convert quaternion error to axis-angle representation
  //   if (orientation_error.w() < 0)
  //   {
  //     orientation_error.coeffs() *= -1;  // Ensure shortest path
  //   }

  //   // For small rotations, the axis-angle is approximately 2 * [x, y, z] of the quaternion
  //   error.tail<3>() = 2.0 * orientation_error.vec();

  // --- 2. Orientation error using quaternion difference
  // Ensure both quaternions are normalized
  Eigen::Quaterniond q = current_orientation.normalized();
  Eigen::Quaterniond qd = orientation_des_.normalized();  // assume this is a member variable or passed in

  // Compute error quaternion: q_e = q_d * q.inverse()
  Eigen::Quaterniond q_e = qd * q.conjugate();

  // Ensure shortest path (handle double cover of SO(3))
  if (q_e.w() < 0.0)
    q_e.coeffs() *= -1.0;

  // Orientation error is imaginary part of q_e (x, y, z)
  error.tail<3>() = q_e.vec();  // .vec() returns (x, y, z)

  //   //! Option 2 - As a vector pointing downwards
  //   // --- 2. Orientation error (align current Z-axis with desired Z-axis)
  //   // Get current rotation matrix from quaternion
  //   Eigen::Matrix3d R = current_orientation.toRotationMatrix();

  //   // Current Z-axis of end-effector in world frame
  //   Eigen::Vector3d z_axis_current = R.col(2);

  //   // Desired Z-axis in world frame (downward)
  //   Eigen::Vector3d z_axis_desired(0.0, 0.0, -1.0);

  //   // Orientation error: rotation needed to align current Z to desired Z
  //   Eigen::Vector3d orientation_error = z_axis_current.cross(z_axis_desired);

  //   error.tail<3>() = orientation_error;

  return error;
}

Eigen::Matrix<double, 6, 7> OSCPDController::getEndEffectorJacobian()
{
  // Get the full end-effector Jacobian from the robot model
  std::array<double, 42> jacobian_array = franka_robot_model_->getZeroJacobian(franka::Frame::kEndEffector);

  // Convert to Eigen matrix (6x7)
  Eigen::Matrix<double, 6, 7> jacobian;
  for (int i = 0; i < 6; ++i)
  {
    for (int j = 0; j < 7; ++j)
    {
      jacobian(i, j) = jacobian_array[j * 6 + i];  // Column-major storage
    }
  }

  return jacobian;
}

}  // namespace fr3_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(fr3_controllers::OSCPDController, controller_interface::ControllerInterface)
