#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <memory>
#include <vector>
#include <string>
#include <array>
#include <fstream>

//Franka includes
#include <franka/model.h>
#include "franka_hardware/model.hpp"
#include <franka/robot_state.h>


class FrankaRimNode : public rclcpp::Node {
public:
  FrankaRimNode();

private:
  void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mass_matrix_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr coriolis_vector_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_vector_publisher_;

  std::shared_ptr<franka::Model> franka_model_;
  
  franka_hardware::Model* robot_model_;

  std::string model_source_path_value_;
};
