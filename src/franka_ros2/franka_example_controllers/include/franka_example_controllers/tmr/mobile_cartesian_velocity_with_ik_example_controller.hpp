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

#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include "controller_interface/controller_interface.hpp"
#include "franka_example_controllers/tmr/swerve_ik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace franka_example_controllers {

class MobileCartesianVelocityWithIkExampleController
    : public controller_interface::ControllerInterface {
 public:
  MobileCartesianVelocityWithIkExampleController() = default;

  controller_interface::CallbackReturn on_init() override;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

 private:
  // Robot params
  std::string mobile_robot_type_ = "tmrv0_2";
  static constexpr size_t kNumberOfWheels = 2;
  double wheel_radius_ = 0.05;
  Eigen::Vector4d wheel_positions_;

  // Sinusoidal trajectory
  double elapsed_time_ = 0.0;
  double freq_ = 0.5;
  double vx_amp_ = 0.2;
  double vy_amp_ = 0.0;
  double wz_amp_ = M_PI / 8.0;

  // IK state
  Eigen::Vector4d steering_angles_;
  Eigen::Vector4d wheel_velocities_;

  std::array<WheelCommand, 2> commands_;
};

}  // namespace franka_example_controllers
