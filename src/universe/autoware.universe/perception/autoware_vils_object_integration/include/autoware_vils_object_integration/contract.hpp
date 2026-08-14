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

#ifndef AUTOWARE_VILS_OBJECT_INTEGRATION__CONTRACT_HPP_
#define AUTOWARE_VILS_OBJECT_INTEGRATION__CONTRACT_HPP_

#include <autoware_perception_msgs/msg/tracked_objects.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace autoware::vils_object_integration
{

// HH_260810 - Keep the PC2 source choice in one enum so incompatible Boolean profiles cannot coexist.
enum class SourceMode : std::uint8_t
{
  RealOnly = 0,
  Shadow = 1,
  HybridOptional = 2,
  HybridRequired = 3,
  VirtualOnlyRequired = 4,
};

// HH_260810 - Parse and render the exact mode names shared by the preparation documents and launch files.
std::optional<SourceMode> parse_source_mode(const std::string & value);
std::string to_string(SourceMode mode);
bool is_canonical_selection_mode(SourceMode mode);
bool is_required_mode(SourceMode mode);

// HH_260810 - Keep all content bounds explicit and owner-reviewable instead of embedding acceptance guesses in callbacks.
struct ValidationLimits
{
  std::size_t max_objects{0U};
  double max_abs_position_m{0.0};
  double max_abs_velocity_mps{0.0};
  double max_abs_acceleration_mps2{0.0};
  double max_covariance{0.0};
  double min_dimension_m{0.0};
  double max_dimension_m{0.0};
  std::size_t max_footprint_points{0U};
};

// HH_260810 - Return one stable machine-readable rejection reason for every payload validation decision.
struct ValidationResult
{
  bool valid{false};
  std::string reason{"unvalidated"};
};

ValidationResult validate_tracked_objects(
  const autoware_perception_msgs::msg::TrackedObjects & objects,
  const std::string & expected_frame_id, const ValidationLimits & limits);

// HH_260810 - Record physical-main association results without changing canonical message fields for provenance.
struct FusionStats
{
  std::size_t physical_count{0U};
  std::size_t virtual_count{0U};
  std::size_t matched_virtual_count{0U};
  std::size_t unmatched_virtual_count{0U};
};

autoware_perception_msgs::msg::TrackedObjects fuse_physical_main(
  const autoware_perception_msgs::msg::TrackedObjects & physical,
  const autoware_perception_msgs::msg::TrackedObjects & virtual_objects,
  double association_distance_m, bool require_matching_primary_classification,
  FusionStats & stats);

}  // namespace autoware::vils_object_integration

#endif  // AUTOWARE_VILS_OBJECT_INTEGRATION__CONTRACT_HPP_
