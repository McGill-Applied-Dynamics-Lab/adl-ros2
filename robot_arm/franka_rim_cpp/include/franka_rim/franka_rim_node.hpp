#pragma once

#include <memory>
#include <vector>
#include <string>
#include <array>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "hardware_interface/loaned_state_interface.hpp"
#include "semantic_components/semantic_component_interface.hpp"

//Franka includes
#include <franka/model.h>
#include "franka_hardware/model.hpp"
#include <franka/robot_state.h>

#include "franka_rim/franka_robot_model.hpp"


class FrankaRimNode : 
  public rclcpp::Node
  // public semantic_components::SemanticComponentInterface<franka_hardware::Model> 
{
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

  // Interface
  // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> model_interfaces_;

  std::unique_ptr<franka_rim::FrankaRobotModel> franka_robot_model_;
  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};
  const std::string arm_id_{"fr3"};

  std::unique_ptr<franka::Robot> robot_;
  std::unique_ptr<franka::Model> model_;
  // rclcpp::Subscription<franka_msgs::msg::FrankaState>::SharedPtr subscription_;
  // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mass_pub_, coriolis_pub_, gravity_pub_;
  std::string robot_ip_{"10.69.54.223"};  // Replace with your robot's IP

};
