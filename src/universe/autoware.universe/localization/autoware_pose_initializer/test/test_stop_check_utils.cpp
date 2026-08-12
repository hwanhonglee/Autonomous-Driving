// Copyright 2026 Hwanhong Lee
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

#include <autoware/pose_initializer/stop_check_utils.hpp>

#include <gtest/gtest.h>

// HH_260812 - Reproduce PC3's 1.11 mm/s lateral noise and retain the 5 mm/s boundary.
TEST(StopCheckUtils, AppliesThreeAxisDeadband)
{
  geometry_msgs::msg::TwistStamped stationary_noise;
  stationary_noise.twist.linear.y = 0.00111;
  const auto stopped =
    autoware::pose_initializer::apply_stop_velocity_deadband(stationary_noise, 0.005);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.z, 0.0);

  geometry_msgs::msg::TwistStamped threshold_motion;
  threshold_motion.twist.linear.y = 0.005;
  const auto moving =
    autoware::pose_initializer::apply_stop_velocity_deadband(threshold_motion, 0.005);
  EXPECT_DOUBLE_EQ(moving.twist.linear.y, 0.005);

  geometry_msgs::msg::TwistStamped diagonal_motion;
  diagonal_motion.twist.linear.x = 0.004;
  diagonal_motion.twist.linear.y = 0.004;
  const auto diagonal =
    autoware::pose_initializer::apply_stop_velocity_deadband(diagonal_motion, 0.005);
  EXPECT_DOUBLE_EQ(diagonal.twist.linear.x, 0.004);
  EXPECT_DOUBLE_EQ(diagonal.twist.linear.y, 0.004);
}
