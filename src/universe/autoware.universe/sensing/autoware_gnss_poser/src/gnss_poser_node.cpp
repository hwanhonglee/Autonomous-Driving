// Copyright 2020 Tier IV, Inc.
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

#include "autoware/gnss_poser/gnss_poser_node.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::gnss_poser
{
GNSSPoser::GNSSPoser(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("gnss_poser", node_options),
  tf2_listener_(tf2_buffer_),
  tf2_broadcaster_(*this),
  base_frame_(declare_parameter<std::string>("base_frame")),
  gnss_base_frame_(declare_parameter<std::string>("gnss_base_frame")),
  map_frame_(declare_parameter<std::string>("map_frame")),
  use_gnss_ins_orientation_(declare_parameter<bool>("use_gnss_ins_orientation")),
  // HH_260811 - Load the maximum allowed timestamp gap between a fix and INS orientation.
  gnss_ins_orientation_timeout_sec_(declare_parameter<double>("gnss_ins_orientation_timeout_sec")),
  // HH_260812 - Keep NovAtel attitude bound to the primary receiver frame during failover.
  gnss_ins_orientation_fix_frame_(declare_parameter<std::string>("gnss_ins_orientation_fix_frame")),
  allow_position_only_fallback_(declare_parameter<bool>("allow_position_only_fallback")),
  course_heading_min_distance_m_(declare_parameter<double>("course_heading_min_distance_m")),
  position_only_yaw_stddev_rad_(declare_parameter<double>("position_only_yaw_stddev_rad")),
  // HH_260812 - Load the calibrated legacy-map translation instead of embedding it in code.
  projected_position_offset_enabled_(
    declare_parameter<bool>("projected_position_offset_enabled")),
  projected_position_offset_x_m_(declare_parameter<double>("projected_position_offset_x_m")),
  projected_position_offset_y_m_(declare_parameter<double>("projected_position_offset_y_m")),
  projected_position_offset_z_m_(declare_parameter<double>("projected_position_offset_z_m")),
  msg_gnss_ins_orientation_stamped_(
    std::make_shared<autoware_sensing_msgs::msg::GnssInsOrientationStamped>()),
  gnss_pose_pub_method_(static_cast<int>(declare_parameter<int>("gnss_pose_pub_method")))
{
  // HH_260812 - Reject invalid configured offsets before they can corrupt published map poses.
  if (
    !std::isfinite(projected_position_offset_x_m_) ||
    !std::isfinite(projected_position_offset_y_m_) ||
    !std::isfinite(projected_position_offset_z_m_))
  {
    throw std::invalid_argument("Projected position offsets must be finite.");
  }

  if (projected_position_offset_enabled_) {
    RCLCPP_INFO(
      get_logger(), "Applying projected position offset x=%.9f, y=%.9f, z=%.9f m",
      projected_position_offset_x_m_, projected_position_offset_y_m_,
      projected_position_offset_z_m_);
  }

  // HH_260812 - Use a valid identity quaternion for a cold-start position-only seed.
  fallback_orientation_.w = 1.0;

  // Subscribe to map_projector_info topic
  const auto adaptor = component_interface_utils::NodeAdaptor(this);
  adaptor.init_sub(
    sub_map_projector_info_, [this](const MapProjectorInfo::Message::ConstSharedPtr msg) {
      callback_map_projector_info(msg);
    });

  // Set up position buffer
  int buff_epoch = static_cast<int>(declare_parameter<int>("buff_epoch"));
  position_buffer_.set_capacity(buff_epoch);

  // Set subscribers and publishers
  nav_sat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", rclcpp::QoS{1},
    std::bind(&GNSSPoser::callback_nav_sat_fix, this, std::placeholders::_1));
  autoware_orientation_sub_ =
    create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "autoware_orientation", rclcpp::QoS{1},
      std::bind(&GNSSPoser::callback_gnss_ins_orientation_stamped, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", rclcpp::QoS{1});
  pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", rclcpp::QoS{1});
  fixed_pub_ = create_publisher<tier4_debug_msgs::msg::BoolStamped>("gnss_fixed", rclcpp::QoS{1});

  // Set msg_gnss_ins_orientation_stamped_ with temporary values (not to publish zero value
  // covariances)
  msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_x = 1.0;
  msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_y = 1.0;
  msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_z = 1.0;
}

void GNSSPoser::callback_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg)
{
  projector_info_ = *msg;
  received_map_projector_info_ = true;
}

void GNSSPoser::callback_nav_sat_fix(
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr nav_sat_fix_msg_ptr)
{
  // Return immediately if map_projector_info has not been received yet.
  if (!received_map_projector_info_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "map_projector_info has not been received yet. Check if the map_projection_loader is "
      "successfully launched.");
    return;
  }

  if (projector_info_.projector_type == MapProjectorInfo::Message::LOCAL) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
      "map_projector_info is local projector type. Unable to convert GNSS pose.");
    return;
  }

  // check fixed topic
  const bool is_status_fixed = is_fixed(nav_sat_fix_msg_ptr->status);

  // publish is_fixed topic
  tier4_debug_msgs::msg::BoolStamped is_fixed_msg;
  is_fixed_msg.stamp = this->now();
  is_fixed_msg.data = is_status_fixed;
  fixed_pub_->publish(is_fixed_msg);

  if (!is_status_fixed) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Not Fixed Topic. Skipping Calculate.");
    return;
  }

  // HH_260812 - Prefer fresh primary INS attitude, but retain a position-only initialization path.
  bool use_fresh_gnss_ins_orientation = false;
  if (use_gnss_ins_orientation_) {
    const bool primary_fix_frame =
      gnss_ins_orientation_fix_frame_.empty() ||
      nav_sat_fix_msg_ptr->header.frame_id == gnss_ins_orientation_fix_frame_;
    if (received_gnss_ins_orientation_ && primary_fix_frame) {
      const auto fix_stamp = rclcpp::Time(nav_sat_fix_msg_ptr->header.stamp);
      const auto orientation_stamp = rclcpp::Time(msg_gnss_ins_orientation_stamped_->header.stamp);
      const double stamp_difference_sec = std::abs((fix_stamp - orientation_stamp).seconds());
      use_fresh_gnss_ins_orientation = stamp_difference_sec <= gnss_ins_orientation_timeout_sec_;
    }

    if (!use_fresh_gnss_ins_orientation && !allow_position_only_fallback_) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Fresh primary GNSS INS orientation is unavailable. Skipping GNSS pose publication.");
      return;
    }

    if (!use_fresh_gnss_ins_orientation) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Fresh primary GNSS INS orientation is unavailable. Publishing a position-only GNSS "
        "pose with broad yaw uncertainty.");
    }
  }

  // get position
  geographic_msgs::msg::GeoPoint gps_point;
  gps_point.latitude = nav_sat_fix_msg_ptr->latitude;
  gps_point.longitude = nav_sat_fix_msg_ptr->longitude;
  gps_point.altitude = nav_sat_fix_msg_ptr->altitude;
  geometry_msgs::msg::Point position =
    autoware::geography_utils::project_forward(gps_point, projector_info_);
  position.z = autoware::geography_utils::convert_height(
    position.z, gps_point.latitude, gps_point.longitude, MapProjectorInfo::Message::WGS84,
    projector_info_.vertical_datum);

  geometry_msgs::msg::Pose gnss_antenna_pose{};

  // publish pose immediately
  if (!gnss_pose_pub_method_) {
    gnss_antenna_pose.position = position;
  } else {
    // fill position buffer
    position_buffer_.push_front(position);
    if (!position_buffer_.full()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Buffering Position. Output Skipped.");
      return;
    }
    // publish average pose or median pose of position buffer
    gnss_antenna_pose.position = (gnss_pose_pub_method_ == 1)
                                   ? get_average_position(position_buffer_)
                                   : get_median_position(position_buffer_);
  }

  // calc gnss antenna orientation
  geometry_msgs::msg::Quaternion orientation;
  if (use_fresh_gnss_ins_orientation) {
    orientation = msg_gnss_ins_orientation_stamped_->orientation.orientation;
    // HH_260812 - Retain the last validated yaw across a short INS outage.
    fallback_orientation_ = orientation;
    course_reference_position_ = gnss_antenna_pose.position;
  } else {
    if (!course_reference_position_) {
      course_reference_position_ = gnss_antenna_pose.position;
    }
    const double delta_x = gnss_antenna_pose.position.x - course_reference_position_->x;
    const double delta_y = gnss_antenna_pose.position.y - course_reference_position_->y;
    if (std::hypot(delta_x, delta_y) >= course_heading_min_distance_m_) {
      fallback_orientation_ = get_quaternion_by_position_difference(
        gnss_antenna_pose.position, *course_reference_position_);
      course_reference_position_ = gnss_antenna_pose.position;
    }
    orientation = fallback_orientation_;
  }

  gnss_antenna_pose.orientation = orientation;

  tf2::Transform tf_map2gnss_antenna{};
  tf2::fromMsg(gnss_antenna_pose, tf_map2gnss_antenna);

  // get TF from gnss_antenna to base_link
  auto tf_gnss_antenna2base_link_msg_ptr = std::make_shared<geometry_msgs::msg::TransformStamped>();

  const std::string gnss_frame = nav_sat_fix_msg_ptr->header.frame_id;
  get_static_transform(
    gnss_frame, base_frame_, tf_gnss_antenna2base_link_msg_ptr, nav_sat_fix_msg_ptr->header.stamp);
  tf2::Transform tf_gnss_antenna2base_link{};
  tf2::fromMsg(tf_gnss_antenna2base_link_msg_ptr->transform, tf_gnss_antenna2base_link);

  // transform pose from gnss_antenna(in map frame) to base_link(in map frame)
  tf2::Transform tf_map2base_link{};
  tf_map2base_link = tf_map2gnss_antenna * tf_gnss_antenna2base_link;

  geometry_msgs::msg::PoseStamped gnss_base_pose_msg{};
  gnss_base_pose_msg.header.stamp = nav_sat_fix_msg_ptr->header.stamp;
  gnss_base_pose_msg.header.frame_id = map_frame_;
  tf2::toMsg(tf_map2base_link, gnss_base_pose_msg.pose);

  // HH_260812 - Apply the calibrated legacy-map translation once to pose, covariance, and TF.
  // Disable this for a natively georeferenced C-track bundle whose projector origin is already
  // the C-track origin; otherwise the approximately 61 km/66 km translation would be duplicated.
  if (projected_position_offset_enabled_) {
    gnss_base_pose_msg.pose.position.x -= projected_position_offset_x_m_;
    gnss_base_pose_msg.pose.position.y -= projected_position_offset_y_m_;
    gnss_base_pose_msg.pose.position.z -= projected_position_offset_z_m_;
  }

  // publish gnss_base_link pose in map frame
  // HH_250205 //sensing/gnss/pose
  pose_pub_->publish(gnss_base_pose_msg);

  // publish gnss_base_link pose_cov in map frame
  // HH_250205 //sensing/gnss/pose_with_covariance
  geometry_msgs::msg::PoseWithCovarianceStamped gnss_base_pose_cov_msg;
  gnss_base_pose_cov_msg.header = gnss_base_pose_msg.header;
  // HH_260810 - Copied the already translated shared pose without applying a second offset.
  gnss_base_pose_cov_msg.pose.pose = gnss_base_pose_msg.pose;

  gnss_base_pose_cov_msg.pose.covariance[7 * 0] =
    can_get_covariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[0] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[7 * 1] =
    can_get_covariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[4] : 10.0;
  gnss_base_pose_cov_msg.pose.covariance[7 * 2] =
    can_get_covariance(*nav_sat_fix_msg_ptr) ? nav_sat_fix_msg_ptr->position_covariance[8] : 10.0;

  if (use_fresh_gnss_ins_orientation) {
    gnss_base_pose_cov_msg.pose.covariance[7 * 3] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_x, 2);
    gnss_base_pose_cov_msg.pose.covariance[7 * 4] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_y, 2);
    gnss_base_pose_cov_msg.pose.covariance[7 * 5] =
      std::pow(msg_gnss_ins_orientation_stamped_->orientation.rmse_rotation_z, 2);
  } else {
    gnss_base_pose_cov_msg.pose.covariance[7 * 3] = 0.1;
    gnss_base_pose_cov_msg.pose.covariance[7 * 4] = 0.1;
    // HH_260812 - Make yaw uncertainty explicit so NDT searches instead of trusting fallback yaw.
    gnss_base_pose_cov_msg.pose.covariance[7 * 5] = std::pow(position_only_yaw_stddev_rad_, 2);
  }

  pose_cov_pub_->publish(gnss_base_pose_cov_msg);

  // broadcast map to gnss_base_link
  publish_tf(map_frame_, gnss_base_frame_, gnss_base_pose_msg);
  // publish_tf("sensor_kit_base_link", gnss_base_frame_, gnss_base_pose_msg); //HH_250205
}

void GNSSPoser::callback_gnss_ins_orientation_stamped(
  const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg)
{
  // HH_260811 - Reject non-finite, degenerate, or invalid-uncertainty INS orientations.
  const auto & orientation = msg->orientation.orientation;
  const double norm_squared = orientation.x * orientation.x + orientation.y * orientation.y +
                              orientation.z * orientation.z + orientation.w * orientation.w;
  const bool finite_orientation = std::isfinite(orientation.x) && std::isfinite(orientation.y) &&
                                  std::isfinite(orientation.z) && std::isfinite(orientation.w) &&
                                  std::isfinite(norm_squared);
  const bool finite_rmse =
    std::isfinite(msg->orientation.rmse_rotation_x) &&
    std::isfinite(msg->orientation.rmse_rotation_y) &&
    std::isfinite(msg->orientation.rmse_rotation_z) && msg->orientation.rmse_rotation_x >= 0.0 &&
    msg->orientation.rmse_rotation_y >= 0.0 && msg->orientation.rmse_rotation_z >= 0.0;

  if (!finite_orientation || norm_squared < 1.0e-12 || !finite_rmse) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Invalid GNSS INS orientation received. Keeping GNSS pose publication inhibited.");
    return;
  }

  // HH_260811 - Normalize each accepted quaternion before enabling GNSS pose publication.
  *msg_gnss_ins_orientation_stamped_ = *msg;
  const double inverse_norm = 1.0 / std::sqrt(norm_squared);
  auto & normalized = msg_gnss_ins_orientation_stamped_->orientation.orientation;
  normalized.x *= inverse_norm;
  normalized.y *= inverse_norm;
  normalized.z *= inverse_norm;
  normalized.w *= inverse_norm;
  received_gnss_ins_orientation_ = true;
}

bool GNSSPoser::is_fixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

bool GNSSPoser::can_get_covariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
{
  return nav_sat_fix_msg.position_covariance_type >
         sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

geometry_msgs::msg::Point GNSSPoser::get_median_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  auto get_median = [](std::vector<double> array) {
    std::sort(std::begin(array), std::end(array));
    const size_t median_index = array.size() / 2;
    double median = (array.size() % 2)
                      ? (array.at(median_index))
                      : ((array.at(median_index) + array.at(median_index - 1)) / 2);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point median_point;
  median_point.x = get_median(array_x);
  median_point.y = get_median(array_y);
  median_point.z = get_median(array_z);
  return median_point;
}

geometry_msgs::msg::Point GNSSPoser::get_average_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point average_point;
  average_point.x =
    std::reduce(array_x.begin(), array_x.end()) / static_cast<double>(array_x.size());
  average_point.y =
    std::reduce(array_y.begin(), array_y.end()) / static_cast<double>(array_y.size());
  average_point.z =
    std::reduce(array_z.begin(), array_z.end()) / static_cast<double>(array_z.size());
  return average_point;
}

geometry_msgs::msg::Quaternion GNSSPoser::get_quaternion_by_heading(const int heading)
{
  int heading_conv = 0;
  // convert heading[0(North)~360] to yaw[-M_PI(West)~M_PI]
  if (heading >= 0 && heading <= 27000000) {
    heading_conv = 9000000 - heading;
  } else {
    heading_conv = 45000000 - heading;
  }
  const double yaw = (heading_conv * 1e-5) * M_PI / 180.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);

  return tf2::toMsg(quaternion);
}

geometry_msgs::msg::Quaternion GNSSPoser::get_quaternion_by_position_difference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point)
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

bool GNSSPoser::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool GNSSPoser::get_static_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr,
  const builtin_interfaces::msg::Time & stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(
      target_frame, source_frame,
      tf2::TimePoint(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());

    transform_stamped_ptr->header.stamp = stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void GNSSPoser::publish_tf(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::msg::PoseStamped & pose_msg)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}
}  // namespace autoware::gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_poser::GNSSPoser)
