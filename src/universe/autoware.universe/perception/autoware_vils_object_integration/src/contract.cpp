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

#include "autoware_vils_object_integration/contract.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <string>

namespace autoware::vils_object_integration
{
namespace
{

// HH_260810 - Validate every scalar and covariance element that can influence association or prediction.
template<typename Range>
bool all_finite(const Range & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const auto value) {
      return std::isfinite(static_cast<double>(value));
    });
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & quaternion)
{
  if (!std::isfinite(quaternion.x) || !std::isfinite(quaternion.y) ||
    !std::isfinite(quaternion.z) || !std::isfinite(quaternion.w))
  {
    return false;
  }
  const auto norm_squared = quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w;
  return norm_squared > 1.0e-8 && std::abs(norm_squared - 1.0) <= 0.05;
}

template<typename Range>
bool bounded_covariance(const Range & values, const double maximum)
{
  return maximum > 0.0 && all_finite(values) &&
         std::all_of(
    values.begin(), values.end(), [maximum](const auto value) {
      return std::abs(static_cast<double>(value)) <= maximum;
    });
}

std::uint8_t primary_classification(
  const autoware_perception_msgs::msg::TrackedObject & object)
{
  if (object.classification.empty()) {
    return autoware_perception_msgs::msg::ObjectClassification::UNKNOWN;
  }
  const auto best = std::max_element(
    object.classification.begin(), object.classification.end(),
    [](const auto & lhs, const auto & rhs) {return lhs.probability < rhs.probability;});
  return best->label;
}

bool valid_dimensions(
  const geometry_msgs::msg::Vector3 & dimensions, const ValidationLimits & limits)
{
  const std::array<double, 3> values{dimensions.x, dimensions.y, dimensions.z};
  return all_finite(values) && std::all_of(
    values.begin(), values.end(), [&](const double value) {
      return value >= limits.min_dimension_m && value <= limits.max_dimension_m;
    });
}

bool valid_classification(const autoware_perception_msgs::msg::TrackedObject & object)
{
  if (object.classification.empty()) {
    return false;
  }
  double probability_sum = 0.0;
  std::array<bool, autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN + 1U>
  observed_labels{};
  for (const auto & classification : object.classification) {
    if (!std::isfinite(classification.probability) || classification.probability < 0.0F ||
      classification.probability > 1.0F ||
      classification.label > autoware_perception_msgs::msg::ObjectClassification::PEDESTRIAN)
    {
      return false;
    }
    if (observed_labels.at(classification.label)) {
      return false;
    }
    observed_labels.at(classification.label) = true;
    probability_sum += classification.probability;
  }
  return probability_sum > 0.0 && probability_sum <= 1.001;
}

double planar_distance_squared(
  const autoware_perception_msgs::msg::TrackedObject & lhs,
  const autoware_perception_msgs::msg::TrackedObject & rhs)
{
  const auto & lhs_position = lhs.kinematics.pose_with_covariance.pose.position;
  const auto & rhs_position = rhs.kinematics.pose_with_covariance.pose.position;
  const auto dx = lhs_position.x - rhs_position.x;
  const auto dy = lhs_position.y - rhs_position.y;
  return dx * dx + dy * dy;
}

}  // namespace

std::optional<SourceMode> parse_source_mode(const std::string & value)
{
  if (value == "real_only") {
    return SourceMode::RealOnly;
  }
  if (value == "shadow") {
    return SourceMode::Shadow;
  }
  if (value == "hybrid_optional") {
    return SourceMode::HybridOptional;
  }
  if (value == "hybrid_required") {
    return SourceMode::HybridRequired;
  }
  if (value == "virtual_only_required") {
    return SourceMode::VirtualOnlyRequired;
  }
  return std::nullopt;
}

std::string to_string(const SourceMode mode)
{
  switch (mode) {
    case SourceMode::RealOnly:
      return "real_only";
    case SourceMode::Shadow:
      return "shadow";
    case SourceMode::HybridOptional:
      return "hybrid_optional";
    case SourceMode::HybridRequired:
      return "hybrid_required";
    case SourceMode::VirtualOnlyRequired:
      return "virtual_only_required";
  }
  return "invalid";
}

bool is_canonical_selection_mode(const SourceMode mode)
{
  return mode == SourceMode::HybridOptional || mode == SourceMode::HybridRequired ||
         mode == SourceMode::VirtualOnlyRequired;
}

bool is_required_mode(const SourceMode mode)
{
  return mode == SourceMode::HybridRequired || mode == SourceMode::VirtualOnlyRequired;
}

ValidationResult validate_tracked_objects(
  const autoware_perception_msgs::msg::TrackedObjects & objects,
  const std::string & expected_frame_id, const ValidationLimits & limits)
{
  if (objects.header.frame_id != expected_frame_id) {
    return {false, "frame_mismatch"};
  }
  if (limits.max_objects == 0U ||
    limits.max_objects > std::numeric_limits<std::uint32_t>::max() ||
    objects.objects.size() > limits.max_objects)
  {
    return {false, "object_count_out_of_range"};
  }
  if (!std::isfinite(limits.max_abs_position_m) ||
    !std::isfinite(limits.max_abs_velocity_mps) ||
    !std::isfinite(limits.max_abs_acceleration_mps2) ||
    !std::isfinite(limits.max_covariance) || !std::isfinite(limits.min_dimension_m) ||
    !std::isfinite(limits.max_dimension_m) || limits.max_abs_position_m <= 0.0 ||
    limits.max_abs_velocity_mps <= 0.0 ||
    limits.max_abs_acceleration_mps2 <= 0.0 || limits.max_covariance <= 0.0 ||
    limits.min_dimension_m <= 0.0 ||
    limits.max_dimension_m <= limits.min_dimension_m || limits.max_footprint_points == 0U)
  {
    return {false, "unapproved_validation_limits"};
  }

  std::set<std::array<std::uint8_t, 16>> observed_object_ids;
  for (const auto & object : objects.objects) {
    const auto & object_id = object.object_id.uuid;
    if (std::all_of(object_id.begin(), object_id.end(), [](const auto byte) {return byte == 0U;}) ||
      !observed_object_ids.insert(object_id).second)
    {
      return {false, "invalid_or_duplicate_object_id"};
    }
    if (!std::isfinite(object.existence_probability) || object.existence_probability < 0.0F ||
      object.existence_probability > 1.0F)
    {
      return {false, "invalid_existence_probability"};
    }
    if (!valid_classification(object)) {
      return {false, "invalid_classification"};
    }

    const auto & pose = object.kinematics.pose_with_covariance.pose;
    const std::array<double, 3> position{pose.position.x, pose.position.y, pose.position.z};
    if (!all_finite(position) ||
      std::any_of(
        position.begin(), position.end(), [&](const double value) {
          return std::abs(value) > limits.max_abs_position_m;
        }))
    {
      return {false, "invalid_position"};
    }
    if (object.kinematics.orientation_availability >
      autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE)
    {
      return {false, "invalid_orientation_availability"};
    }
    const auto orientation_is_required = object.kinematics.orientation_availability !=
      autoware_perception_msgs::msg::TrackedObjectKinematics::UNAVAILABLE;
    const std::array<double, 4> orientation_values{
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    if ((orientation_is_required && !finite_quaternion(pose.orientation)) ||
      (!orientation_is_required && !all_finite(orientation_values)) ||
      !bounded_covariance(
        object.kinematics.pose_with_covariance.covariance, limits.max_covariance))
    {
      return {false, "invalid_pose_or_covariance"};
    }

    const auto & twist = object.kinematics.twist_with_covariance.twist;
    const std::array<double, 6> twist_values{
      twist.linear.x, twist.linear.y, twist.linear.z,
      twist.angular.x, twist.angular.y, twist.angular.z};
    if (!all_finite(twist_values) ||
      std::any_of(
        twist_values.begin(), twist_values.end(), [&](const double value) {
          return std::abs(value) > limits.max_abs_velocity_mps;
        }) || !bounded_covariance(
        object.kinematics.twist_with_covariance.covariance, limits.max_covariance))
    {
      return {false, "invalid_twist_or_covariance"};
    }

    const auto & acceleration = object.kinematics.acceleration_with_covariance.accel;
    const std::array<double, 6> acceleration_values{
      acceleration.linear.x, acceleration.linear.y, acceleration.linear.z,
      acceleration.angular.x, acceleration.angular.y, acceleration.angular.z};
    if (!all_finite(acceleration_values) ||
      std::any_of(
        acceleration_values.begin(), acceleration_values.end(), [&](const double value) {
          return std::abs(value) > limits.max_abs_acceleration_mps2;
        }) || !bounded_covariance(
        object.kinematics.acceleration_with_covariance.covariance, limits.max_covariance))
    {
      return {false, "invalid_acceleration_or_covariance"};
    }

    if (!valid_dimensions(object.shape.dimensions, limits)) {
      return {false, "invalid_shape_dimensions"};
    }
    if (object.shape.type > autoware_perception_msgs::msg::Shape::POLYGON ||
      object.shape.footprint.points.size() > limits.max_footprint_points)
    {
      return {false, "invalid_shape_type_or_footprint_count"};
    }
    if (object.shape.type == autoware_perception_msgs::msg::Shape::POLYGON &&
      object.shape.footprint.points.size() < 3U)
    {
      return {false, "polygon_footprint_too_small"};
    }
    for (const auto & point : object.shape.footprint.points) {
      const std::array<double, 3> values{point.x, point.y, point.z};
      if (!all_finite(values) ||
        std::any_of(
          values.begin(), values.end(), [&](const double value) {
            return std::abs(value) > limits.max_dimension_m;
          }))
      {
        return {false, "invalid_shape_footprint"};
      }
    }
  }
  return {true, "accepted"};
}

autoware_perception_msgs::msg::TrackedObjects fuse_physical_main(
  const autoware_perception_msgs::msg::TrackedObjects & physical,
  const autoware_perception_msgs::msg::TrackedObjects & virtual_objects,
  const double association_distance_m, const bool require_matching_primary_classification,
  FusionStats & stats)
{
  auto output = physical;
  stats = {};
  stats.physical_count = physical.objects.size();
  stats.virtual_count = virtual_objects.objects.size();

  if (association_distance_m <= 0.0 || !std::isfinite(association_distance_m)) {
    return output;
  }
  const auto threshold_squared = association_distance_m * association_distance_m;

  for (const auto & virtual_object : virtual_objects.objects) {
    const auto virtual_label = primary_classification(virtual_object);
    const auto matched = std::any_of(
      physical.objects.begin(), physical.objects.end(), [&](const auto & physical_object) {
        // HH_260810 - A virtual object may never duplicate a physical UUID, regardless of pose.
        if (physical_object.object_id.uuid == virtual_object.object_id.uuid) {
          return true;
        }
        if (require_matching_primary_classification &&
        primary_classification(physical_object) != virtual_label)
        {
          return false;
        }
        return planar_distance_squared(physical_object, virtual_object) <= threshold_squared;
      });
    if (matched) {
      ++stats.matched_virtual_count;
      continue;
    }
    output.objects.push_back(virtual_object);
    ++stats.unmatched_virtual_count;
  }
  return output;
}

}  // namespace autoware::vils_object_integration
