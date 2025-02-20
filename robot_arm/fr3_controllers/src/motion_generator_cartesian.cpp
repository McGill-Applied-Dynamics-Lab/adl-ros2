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

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fr3_controllers/motion_generator_cartesian.hpp>
#include <utility>

MotionGeneratorCartesian::MotionGeneratorCartesian(double speed_factor, const Vector3d& x_ee, const Vector3d& x_ee_goal)
    : x_ee_start_(x_ee), x_ee_goal_(x_ee_goal)
{
  assert(speed_factor > 0);
  assert(speed_factor <= 1);
  delta_x_ee_ = x_ee_goal - x_ee_start_;
  dx_ee_max_ *= speed_factor;
  ddx_ee_max_start_ *= speed_factor;
  ddx_ee_max_goal_ *= speed_factor;
  calculateSynchronizedValues();
}

bool MotionGeneratorCartesian::calculateDesiredValues(double time, Vector3d* delta_x_ee_d) const
{
  Vector3i sign_delta_x_ee;
  sign_delta_x_ee << delta_x_ee_.cwiseSign().cast<int>();
  Vector3d t_d = t_2_sync_ - t_1_sync_;
  Vector3d delta_t_2_sync = t_f_sync_ - t_2_sync_;
  std::array<bool, 3> position_motion_finished{};

  for (auto i = 0; i < 3; i++)
  {
    if (std::abs(delta_x_ee_[i]) < kDeltaQMotionFinished)
    {
      (*delta_x_ee_d)[i] = 0;
      position_motion_finished.at(i) = true;
    }
    else
    {
      if (time < t_1_sync_[i])
      {
        (*delta_x_ee_d)[i] = -1.0 / std::pow(t_1_sync_[i], 3.0) * dx_ee_max_sync_[i] * sign_delta_x_ee[i] *
                             (0.5 * time - t_1_sync_[i]) * std::pow(time, 3.0);
      }
      else if (time >= t_1_sync_[i] && time < t_2_sync_[i])
      {
        (*delta_x_ee_d)[i] = x_ee_1_[i] + (time - t_1_sync_[i]) * dx_ee_max_sync_[i] * sign_delta_x_ee[i];
      }
      else if (time >= t_2_sync_[i] && time < t_f_sync_[i])
      {
        (*delta_x_ee_d)[i] =
            delta_x_ee_[i] +
            0.5 *
                (1.0 / std::pow(delta_t_2_sync[i], 3.0) * (time - t_1_sync_[i] - 2.0 * delta_t_2_sync[i] - t_d[i]) *
                     std::pow((time - t_1_sync_[i] - t_d[i]), 3.0) +
                 (2.0 * time - 2.0 * t_1_sync_[i] - delta_t_2_sync[i] - 2.0 * t_d[i])) *
                dx_ee_max_sync_[i] * sign_delta_x_ee[i];
      }
      else
      {
        (*delta_x_ee_d)[i] = delta_x_ee_[i];
        position_motion_finished.at(i) = true;
      }
    }
  }
  return std::all_of(position_motion_finished.cbegin(), position_motion_finished.cend(),
                     [](bool each_joint_finished) { return each_joint_finished; });
}

void MotionGeneratorCartesian::calculateSynchronizedValues()
{
  Vector3d dx_ee_max_reach(dx_ee_max_);
  Vector3d t_f = Vector3d::Zero();
  Vector3d delta_t_2 = Vector3d::Zero();
  Vector3d t_1 = Vector3d::Zero();
  Vector3d delta_t_2_sync = Vector3d::Zero();

  Vector3i sign_delta_x_ee;

  sign_delta_x_ee << delta_x_ee_.cwiseSign().cast<int>();

  for (auto i = 0; i < 3; i++)
  {
    if (std::abs(delta_x_ee_[i]) > kDeltaQMotionFinished)
    {
      if (std::abs(delta_x_ee_[i]) < (3.0 / 4.0 * (std::pow(dx_ee_max_[i], 2.0) / ddx_ee_max_start_[i]) +
                                      3.0 / 4.0 * (std::pow(dx_ee_max_[i], 2.0) / ddx_ee_max_goal_[i])))
      {
        dx_ee_max_reach[i] =
            std::sqrt(4.0 / 3.0 * delta_x_ee_[i] * sign_delta_x_ee[i] * (ddx_ee_max_start_[i] * ddx_ee_max_goal_[i]) /
                      (ddx_ee_max_start_[i] + ddx_ee_max_goal_[i]));
      }
      t_1[i] = 1.5 * dx_ee_max_reach[i] / ddx_ee_max_start_[i];
      delta_t_2[i] = 1.5 * dx_ee_max_reach[i] / ddx_ee_max_goal_[i];
      t_f[i] = t_1[i] / 2.0 + delta_t_2[i] / 2.0 + std::abs(delta_x_ee_[i]) / dx_ee_max_reach[i];
    }
  }
  double max_t_f = t_f.maxCoeff();
  for (auto i = 0; i < 3; i++)
  {
    if (std::abs(delta_x_ee_[i]) > kDeltaQMotionFinished)
    {
      double param_a = 1.5 / 2.0 * (ddx_ee_max_goal_[i] + ddx_ee_max_start_[i]);
      double param_b = -1.0 * max_t_f * ddx_ee_max_goal_[i] * ddx_ee_max_start_[i];
      double param_c = std::abs(delta_x_ee_[i]) * ddx_ee_max_goal_[i] * ddx_ee_max_start_[i];
      double delta = param_b * param_b - 4.0 * param_a * param_c;
      if (delta < 0.0)
      {
        delta = 0.0;
      }
      dx_ee_max_sync_[i] = (-1.0 * param_b - std::sqrt(delta)) / (2.0 * param_a);
      t_1_sync_[i] = 1.5 * dx_ee_max_sync_[i] / ddx_ee_max_start_[i];
      delta_t_2_sync[i] = 1.5 * dx_ee_max_sync_[i] / ddx_ee_max_goal_[i];
      t_f_sync_[i] = (t_1_sync_)[i] / 2.0 + delta_t_2_sync[i] / 2.0 + std::abs(delta_x_ee_[i] / dx_ee_max_sync_[i]);
      t_2_sync_[i] = (t_f_sync_)[i] - delta_t_2_sync[i];
      x_ee_1_[i] = (dx_ee_max_sync_)[i] * sign_delta_x_ee[i] * (0.5 * (t_1_sync_)[i]);
    }
  }
}

std::pair<MotionGeneratorCartesian::Vector3d, bool> MotionGeneratorCartesian::getDesiredEEPosition(
    const rclcpp::Duration& trajectory_time)
{
  time_ = trajectory_time.seconds();

  Vector3d delta_x_ee_d;
  bool motion_finished = calculateDesiredValues(time_, &delta_x_ee_d);

  std::array<double, 3> x_ee{};
  Eigen::VectorXd::Map(x_ee.data(), 3) = (x_ee_start_ + delta_x_ee_d);
  return std::make_pair(x_ee_start_ + delta_x_ee_d, motion_finished);
}
