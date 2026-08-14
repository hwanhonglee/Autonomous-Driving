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

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

namespace autoware::vils_object_integration
{
namespace
{

using autoware_perception_msgs::msg::ObjectClassification;
using autoware_perception_msgs::msg::Shape;
using autoware_perception_msgs::msg::TrackedObject;
using autoware_perception_msgs::msg::TrackedObjects;

// HH_260810 - Use explicit positive owner-like bounds so valid fixtures cannot pass on defaults.
ValidationLimits approved_test_limits()
{
  ValidationLimits limits;
  limits.max_objects = 16U;
  limits.max_abs_position_m = 1000.0;
  limits.max_abs_velocity_mps = 100.0;
  limits.max_abs_acceleration_mps2 = 30.0;
  limits.max_covariance = 1000.0;
  limits.min_dimension_m = 0.01;
  limits.max_dimension_m = 20.0;
  limits.max_footprint_points = 32U;
  return limits;
}

// HH_260810 - Construct the smallest fully finite tracked object accepted by the payload contract.
TrackedObject make_object(
  const std::uint8_t id, const double x, const double y, const std::uint8_t label)
{
  TrackedObject object;
  object.object_id.uuid[0] = id;
  object.existence_probability = 0.9F;

  ObjectClassification classification;
  classification.label = label;
  classification.probability = 1.0F;
  object.classification.push_back(classification);

  auto & pose = object.kinematics.pose_with_covariance.pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.orientation.w = 1.0;
  object.kinematics.orientation_availability =
    autoware_perception_msgs::msg::TrackedObjectKinematics::AVAILABLE;

  object.shape.type = Shape::BOUNDING_BOX;
  object.shape.dimensions.x = 4.0;
  object.shape.dimensions.y = 2.0;
  object.shape.dimensions.z = 1.5;
  return object;
}

// HH_260810 - Give every validation test the exact map frame required by the PC2 contract.
TrackedObjects make_objects()
{
  TrackedObjects objects;
  objects.header.frame_id = "map";
  return objects;
}

TEST(SourceModeContract, ParsesOnlyExactDocumentedNames)
{
  EXPECT_EQ(parse_source_mode("real_only"), SourceMode::RealOnly);
  EXPECT_EQ(parse_source_mode("shadow"), SourceMode::Shadow);
  EXPECT_EQ(parse_source_mode("hybrid_optional"), SourceMode::HybridOptional);
  EXPECT_EQ(parse_source_mode("hybrid_required"), SourceMode::HybridRequired);
  EXPECT_EQ(parse_source_mode("virtual_only_required"), SourceMode::VirtualOnlyRequired);
  EXPECT_FALSE(parse_source_mode("hybrid"));
  EXPECT_FALSE(parse_source_mode("SHADOW"));
}

TEST(SourceModeContract, SeparatesCanonicalAndRequiredAuthority)
{
  EXPECT_FALSE(is_canonical_selection_mode(SourceMode::RealOnly));
  EXPECT_FALSE(is_canonical_selection_mode(SourceMode::Shadow));
  EXPECT_TRUE(is_canonical_selection_mode(SourceMode::HybridOptional));
  EXPECT_TRUE(is_canonical_selection_mode(SourceMode::HybridRequired));
  EXPECT_TRUE(is_canonical_selection_mode(SourceMode::VirtualOnlyRequired));

  EXPECT_FALSE(is_required_mode(SourceMode::HybridOptional));
  EXPECT_TRUE(is_required_mode(SourceMode::HybridRequired));
  EXPECT_TRUE(is_required_mode(SourceMode::VirtualOnlyRequired));
  EXPECT_EQ(to_string(SourceMode::Shadow), "shadow");
}

TEST(PayloadContract, AcceptsAValidObjectAndFreshEmptySnapshot)
{
  auto populated = make_objects();
  populated.objects.push_back(make_object(1U, 2.0, -3.0, ObjectClassification::CAR));
  const auto populated_result = validate_tracked_objects(populated, "map", approved_test_limits());
  EXPECT_TRUE(populated_result.valid) << populated_result.reason;

  const auto empty_result = validate_tracked_objects(make_objects(), "map", approved_test_limits());
  EXPECT_TRUE(empty_result.valid) << empty_result.reason;
}

TEST(PayloadContract, RejectsFailClosedLimitsAndWrongFrame)
{
  const auto default_result = validate_tracked_objects(make_objects(), "map", ValidationLimits{});
  EXPECT_FALSE(default_result.valid);
  EXPECT_EQ(default_result.reason, "object_count_out_of_range");

  auto wrong_frame = make_objects();
  wrong_frame.header.frame_id = "base_link";
  const auto frame_result = validate_tracked_objects(wrong_frame, "map", approved_test_limits());
  EXPECT_FALSE(frame_result.valid);
  EXPECT_EQ(frame_result.reason, "frame_mismatch");
}

TEST(PayloadContract, RejectsNonFiniteLimitsAndDuplicateIdentifiers)
{
  auto nonfinite_limits = approved_test_limits();
  nonfinite_limits.max_abs_position_m = std::numeric_limits<double>::quiet_NaN();
  const auto limits_result = validate_tracked_objects(make_objects(), "map", nonfinite_limits);
  EXPECT_FALSE(limits_result.valid);
  EXPECT_EQ(limits_result.reason, "unapproved_validation_limits");

  auto duplicate_ids = make_objects();
  duplicate_ids.objects.push_back(make_object(80U, 0.0, 0.0, ObjectClassification::CAR));
  duplicate_ids.objects.push_back(make_object(80U, 5.0, 0.0, ObjectClassification::CAR));
  const auto duplicate_result =
    validate_tracked_objects(duplicate_ids, "map", approved_test_limits());
  EXPECT_FALSE(duplicate_result.valid);
  EXPECT_EQ(duplicate_result.reason, "invalid_or_duplicate_object_id");
}

TEST(PayloadContract, RejectsNonFiniteAndOutOfRangeFields)
{
  auto invalid_position = make_objects();
  invalid_position.objects.push_back(
    make_object(2U, std::numeric_limits<double>::quiet_NaN(), 0.0, ObjectClassification::CAR));
  const auto position_result =
    validate_tracked_objects(invalid_position, "map", approved_test_limits());
  EXPECT_FALSE(position_result.valid);
  EXPECT_EQ(position_result.reason, "invalid_position");

  auto invalid_covariance = make_objects();
  invalid_covariance.objects.push_back(make_object(3U, 0.0, 0.0, ObjectClassification::CAR));
  invalid_covariance.objects.front().kinematics.pose_with_covariance.covariance[0] =
    std::numeric_limits<double>::infinity();
  const auto covariance_result =
    validate_tracked_objects(invalid_covariance, "map", approved_test_limits());
  EXPECT_FALSE(covariance_result.valid);
  EXPECT_EQ(covariance_result.reason, "invalid_pose_or_covariance");

  auto invalid_shape = make_objects();
  invalid_shape.objects.push_back(make_object(4U, 0.0, 0.0, ObjectClassification::CAR));
  invalid_shape.objects.front().shape.dimensions.x = 0.0;
  const auto shape_result = validate_tracked_objects(invalid_shape, "map", approved_test_limits());
  EXPECT_FALSE(shape_result.valid);
  EXPECT_EQ(shape_result.reason, "invalid_shape_dimensions");
}

TEST(FusionContract, KeepsPhysicalOnOverlapAndAppendsOnlyUnmatchedVirtual)
{
  auto physical = make_objects();
  physical.objects.push_back(make_object(10U, 0.0, 0.0, ObjectClassification::CAR));

  auto virtual_objects = make_objects();
  virtual_objects.objects.push_back(make_object(20U, 0.5, 0.0, ObjectClassification::CAR));
  virtual_objects.objects.push_back(make_object(30U, 10.0, 0.0, ObjectClassification::PEDESTRIAN));

  FusionStats stats;
  const auto output = fuse_physical_main(physical, virtual_objects, 1.0, true, stats);
  ASSERT_EQ(output.objects.size(), 2U);
  EXPECT_EQ(output.objects[0].object_id.uuid[0], 10U);
  EXPECT_EQ(output.objects[1].object_id.uuid[0], 30U);
  EXPECT_EQ(stats.physical_count, 1U);
  EXPECT_EQ(stats.virtual_count, 2U);
  EXPECT_EQ(stats.matched_virtual_count, 1U);
  EXPECT_EQ(stats.unmatched_virtual_count, 1U);
}

TEST(FusionContract, ClassificationGatePreventsCrossClassSuppression)
{
  auto physical = make_objects();
  physical.objects.push_back(make_object(40U, 0.0, 0.0, ObjectClassification::CAR));

  auto virtual_objects = make_objects();
  virtual_objects.objects.push_back(make_object(50U, 0.1, 0.0, ObjectClassification::PEDESTRIAN));

  FusionStats gated_stats;
  const auto gated = fuse_physical_main(physical, virtual_objects, 1.0, true, gated_stats);
  EXPECT_EQ(gated.objects.size(), 2U);
  EXPECT_EQ(gated_stats.unmatched_virtual_count, 1U);

  FusionStats ungated_stats;
  const auto ungated = fuse_physical_main(physical, virtual_objects, 1.0, false, ungated_stats);
  EXPECT_EQ(ungated.objects.size(), 1U);
  EXPECT_EQ(ungated_stats.matched_virtual_count, 1U);
}

TEST(FusionContract, InvalidAssociationThresholdReturnsPhysicalOnly)
{
  auto physical = make_objects();
  physical.objects.push_back(make_object(60U, 0.0, 0.0, ObjectClassification::CAR));
  auto virtual_objects = make_objects();
  virtual_objects.objects.push_back(make_object(70U, 10.0, 0.0, ObjectClassification::CAR));

  FusionStats stats;
  const auto output = fuse_physical_main(physical, virtual_objects, 0.0, true, stats);
  ASSERT_EQ(output.objects.size(), 1U);
  EXPECT_EQ(output.objects.front().object_id.uuid[0], 60U);
  EXPECT_EQ(stats.physical_count, 1U);
  EXPECT_EQ(stats.virtual_count, 1U);
  EXPECT_EQ(stats.matched_virtual_count, 0U);
  EXPECT_EQ(stats.unmatched_virtual_count, 0U);
}

TEST(FusionContract, PhysicalIdentifierWinsEvenWhenPositionsDoNotOverlap)
{
  auto physical = make_objects();
  physical.objects.push_back(make_object(90U, 0.0, 0.0, ObjectClassification::CAR));
  auto virtual_objects = make_objects();
  virtual_objects.objects.push_back(
    make_object(90U, 50.0, 0.0, ObjectClassification::PEDESTRIAN));

  FusionStats stats;
  const auto output = fuse_physical_main(physical, virtual_objects, 1.0, true, stats);
  ASSERT_EQ(output.objects.size(), 1U);
  EXPECT_EQ(output.objects.front().object_id.uuid[0], 90U);
  EXPECT_EQ(stats.matched_virtual_count, 1U);
}

}  // namespace
}  // namespace autoware::vils_object_integration
