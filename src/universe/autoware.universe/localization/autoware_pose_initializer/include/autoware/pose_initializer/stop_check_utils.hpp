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

#ifndef AUTOWARE__POSE_INITIALIZER__STOP_CHECK_UTILS_HPP_
#define AUTOWARE__POSE_INITIALIZER__STOP_CHECK_UTILS_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace autoware::pose_initializer
{

// HH_260812 - Remove only sub-threshold three-axis sensor noise before the unchanged stop check.
inline geometry_msgs::msg::TwistStamped apply_stop_velocity_deadband(
  const geometry_msgs::msg::TwistStamped & input, const double threshold)
{
  auto output = input;
  const auto & linear = input.twist.linear;
  const double squared_speed = linear.x * linear.x + linear.y * linear.y + linear.z * linear.z;
  if (squared_speed < threshold * threshold) {
    output.twist.linear.x = 0.0;
    output.twist.linear.y = 0.0;
    output.twist.linear.z = 0.0;
  }
  return output;
}

}  // namespace autoware::pose_initializer

#endif  // AUTOWARE__POSE_INITIALIZER__STOP_CHECK_UTILS_HPP_
