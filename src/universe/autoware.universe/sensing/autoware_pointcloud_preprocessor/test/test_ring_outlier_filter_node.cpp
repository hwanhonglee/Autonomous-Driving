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

#include <gtest/gtest.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "autoware/point_types/types.hpp"
#include "autoware/pointcloud_preprocessor/outlier_filter/ring_outlier_filter_node.hpp"

namespace autoware::pointcloud_preprocessor
{

class TestableRingOutlierFilterComponent : public RingOutlierFilterComponent
{
public:
  explicit TestableRingOutlierFilterComponent(const rclcpp::NodeOptions & options)
  : RingOutlierFilterComponent(options)
  {
  }

  void apply_filter(const PointCloud2ConstPtr & input, PointCloud2 & output)
  {
    TransformInfo transform_info;
    transform_info.need_transform = false;
    faster_filter(input, nullptr, output, transform_info);
  }
};

// HH_260811 - Reproduce the live ring-filter path and reject a zero PointCloud2 row_step.
TEST(RingOutlierFilterComponent, SetsCompactOutputRowStep)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {{"distance_ratio", 1.03},
     {"object_length_threshold", 0.1},
     {"num_points_threshold", 0},
     {"max_rings_num", 128},
     {"max_points_num_per_ring", 6000},
     {"publish_outlier_pointcloud", false},
     {"min_azimuth_deg", 0.0},
     {"max_azimuth_deg", 360.0},
     {"max_distance", 200.0},
     {"vertical_bins", 128},
     {"horizontal_bins", 360},
     {"noise_threshold", 1}});
  auto filter = std::make_shared<TestableRingOutlierFilterComponent>(options);

  pcl::PointCloud<autoware::point_types::PointXYZIRCAEDT> pcl_input;
  pcl_input.push_back({1.0F, 0.0F, 0.0F, 10U, 1U, 0U, 0.0F, 0.0F, 1.0F, 0U});
  pcl_input.push_back({1.1F, 0.0F, 0.0F, 11U, 1U, 0U, 0.01F, 0.0F, 1.1F, 100U});

  auto input = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(pcl_input, *input);
  input->header.frame_id = "base_link";

  sensor_msgs::msg::PointCloud2 output;
  filter->apply_filter(input, output);

  ASSERT_GT(output.width, 0U);
  EXPECT_EQ(output.height, 1U);
  EXPECT_EQ(output.row_step, output.point_step * output.width);
  EXPECT_EQ(output.data.size(), output.row_step * output.height);
}

}  // namespace autoware::pointcloud_preprocessor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
