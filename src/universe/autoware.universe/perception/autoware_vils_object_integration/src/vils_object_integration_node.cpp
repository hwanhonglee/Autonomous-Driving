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

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <openssl/sha.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rmw/types.h>
#include <vils_interfaces/msg/accepted_pc4_status.hpp>
#include <vils_interfaces/srv/arm_pc4_source.hpp>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::vils_object_integration
{
namespace
{
using TrackedObjects = autoware_perception_msgs::msg::TrackedObjects;
using AcceptedStatus = vils_interfaces::msg::AcceptedPc4Status;
using ArmSource = vils_interfaces::srv::ArmPc4Source;
using SteadyClock = std::chrono::steady_clock;

// HH_260810 - Encode DDS endpoint identities in a stable lowercase form for manifests and arm requests.
template<typename GidRange>
std::string gid_to_hex(const GidRange & gid)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (const auto byte : gid) {
    stream << std::setw(2) << static_cast<unsigned int>(byte);
  }
  return stream.str();
}

// HH_260810 - Hash the exact receiver-side ROS serialization and require a named serialization contract in metadata.
std::string serialized_sha256(const TrackedObjects & message)
{
  rclcpp::Serialization<TrackedObjects> serializer;
  rclcpp::SerializedMessage serialized;
  serializer.serialize_message(&message, &serialized);
  const auto & rcl_message = serialized.get_rcl_serialized_message();
  std::array<unsigned char, SHA256_DIGEST_LENGTH> digest{};
  SHA256(rcl_message.buffer, rcl_message.buffer_length, digest.data());
  return gid_to_hex(digest);
}

std::int64_t stamp_to_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

builtin_interfaces::msg::Time nanoseconds_to_stamp(const std::int64_t value)
{
  builtin_interfaces::msg::Time stamp;
  if (value <= 0) {
    return stamp;
  }
  stamp.sec = static_cast<std::int32_t>(value / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(value % 1000000000LL);
  return stamp;
}

bool parse_bool(const std::string & value, bool & output)
{
  if (value == "true" || value == "1") {
    output = true;
    return true;
  }
  if (value == "false" || value == "0") {
    output = false;
    return true;
  }
  return false;
}

bool is_lowercase_hex(const std::string & value, const std::size_t expected_size)
{
  return value.size() == expected_size &&
         std::all_of(
    value.begin(), value.end(), [](const char character) {
      return (character >= '0' && character <= '9') ||
      (character >= 'a' && character <= 'f');
    });
}

bool is_safe_session_id(const std::string & value)
{
  return !value.empty() && value.size() <= 128U &&
         std::all_of(
    value.begin(), value.end(), [](const unsigned char character) {
      return std::isalnum(character) != 0 || character == '-' || character == '_' ||
      character == '.' || character == ':';
    });
}

bool valid_stamp_fields(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U;
}

template<typename Integer>
bool parse_decimal_exact(const std::string & value, Integer & output)
{
  if (value.empty()) {
    return false;
  }
  const auto * begin = value.data();
  const auto * end = begin + value.size();
  const auto result = std::from_chars(begin, end, output, 10);
  return result.ec == std::errc{} && result.ptr == end;
}

std::string normalize_fqn(const std::string & node_namespace, const std::string & node_name)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + node_name;
  }
  if (node_namespace.back() == '/') {
    return node_namespace + node_name;
  }
  return node_namespace + "/" + node_name;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream output;
  for (const auto character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20U) {
          output << "\\u00" << std::hex << std::setw(2) << std::setfill('0') <<
            static_cast<unsigned int>(static_cast<unsigned char>(character));
        } else {
          output << character;
        }
        break;
    }
  }
  return output.str();
}

}  // namespace

class VilsObjectIntegrationNode : public rclcpp::Node
{
public:
  explicit VilsObjectIntegrationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vils_object_integration", options)
  {
    // HH_260810 - Load the complete contract as parameters and keep unapproved defaults fail closed.
    const auto mode_value = declare_parameter<std::string>("mode", "real_only");
    const auto parsed_mode = parse_source_mode(mode_value);
    if (!parsed_mode) {
      throw std::invalid_argument("invalid VILS source mode: " + mode_value);
    }
    mode_ = *parsed_mode;
    contract_approved_ = declare_parameter<bool>("contract_approved", false);
    enable_canonical_selection_ =
      declare_parameter<bool>("enable_canonical_selection", false);
    enable_required_modes_ = declare_parameter<bool>("enable_required_modes", false);

    physical_topic_ = declare_parameter<std::string>(
      "topics.physical_objects", "/perception/object_recognition/tracking/objects");
    virtual_topic_ = declare_parameter<std::string>(
      "topics.virtual_objects", "/perception/pc4/virtual_obstacles/tracked_objects");
    diagnostic_topic_ = declare_parameter<std::string>(
      "topics.pc4_diagnostics", "/diagnostics/pc4/object_adapter");
    output_topic_ = declare_parameter<std::string>(
      "topics.output_objects", "/perception/pc2/vils/candidate_tracked_objects");
    status_topic_ = declare_parameter<std::string>(
      "topics.accepted_status", "/perception/pc2/vils/accepted_pc4_status");
    velocity_topic_ = declare_parameter<std::string>(
      "topics.velocity_report", "/vehicle/status/velocity_status");
    gear_topic_ = declare_parameter<std::string>(
      "topics.gear_report", "/vehicle/status/gear_status");
    control_mode_topic_ = declare_parameter<std::string>(
      "topics.control_mode_report", "/vehicle/status/control_mode");

    expected_frame_id_ = declare_parameter<std::string>("contract.expected_frame_id", "map");
    expected_map_digest_ =
      declare_parameter<std::string>("contract.expected_map_digest", "__UNAPPROVED__");
    expected_transform_digest_ = declare_parameter<std::string>(
      "contract.expected_transform_digest", "__UNAPPROVED__");
    expected_serialization_contract_ = declare_parameter<std::string>(
      "contract.expected_serialization_contract", "__UNAPPROVED__");
    approved_object_node_fqn_ = declare_parameter<std::string>(
      "contract.approved_object_publisher_node_fqn", "__UNAPPROVED__");
    approved_diagnostic_node_fqn_ = declare_parameter<std::string>(
      "contract.approved_diagnostic_publisher_node_fqn", "__UNAPPROVED__");
    diagnostic_name_ = declare_parameter<std::string>(
      "contract.diagnostic_name", "pc4_object_adapter");
    diagnostic_hardware_id_ = declare_parameter<std::string>(
      "contract.diagnostic_hardware_id", "pc4/digital_twin");
    // HH_260810 - Require an explicit scenario decision before an empty actor snapshot can be accepted.
    allow_empty_snapshot_ = declare_parameter<bool>("contract.allow_empty_snapshot", false);

    virtual_ttl_ = std::chrono::milliseconds(
      declare_parameter<std::int64_t>("timing.virtual_ttl_ms", 0));
    metadata_join_timeout_ = std::chrono::milliseconds(
      declare_parameter<std::int64_t>("timing.metadata_join_timeout_ms", 0));
    max_source_age_sec_ = declare_parameter<double>("timing.max_source_age_sec", 0.0);
    max_future_skew_sec_ = declare_parameter<double>("timing.max_future_skew_sec", 0.0);
    max_fusion_skew_sec_ = declare_parameter<double>("timing.max_fusion_skew_sec", 0.0);
    safe_state_timeout_sec_ = declare_parameter<double>("timing.safe_state_timeout_sec", 1.0);

    const auto max_objects = declare_parameter<std::int64_t>("limits.max_objects", 0);
    limits_.max_objects = max_objects > 0 ? static_cast<std::size_t>(max_objects) : 0U;
    limits_.max_abs_position_m =
      declare_parameter<double>("limits.max_abs_position_m", 0.0);
    limits_.max_abs_velocity_mps =
      declare_parameter<double>("limits.max_abs_velocity_mps", 0.0);
    limits_.max_abs_acceleration_mps2 =
      declare_parameter<double>("limits.max_abs_acceleration_mps2", 0.0);
    limits_.max_covariance = declare_parameter<double>("limits.max_covariance", 0.0);
    limits_.min_dimension_m = declare_parameter<double>("limits.min_dimension_m", 0.0);
    limits_.max_dimension_m = declare_parameter<double>("limits.max_dimension_m", 0.0);
    const auto max_footprint_points =
      declare_parameter<std::int64_t>("limits.max_footprint_points", 0);
    limits_.max_footprint_points = max_footprint_points > 0 ?
      static_cast<std::size_t>(max_footprint_points) : 0U;
    const auto max_pending_samples =
      declare_parameter<std::int64_t>("limits.max_pending_samples", 0);
    max_pending_samples_ = max_pending_samples > 0 ?
      static_cast<std::size_t>(max_pending_samples) : 0U;
    association_distance_m_ =
      declare_parameter<double>("fusion.association_distance_m", 0.0);
    require_matching_classification_ =
      declare_parameter<bool>("fusion.require_matching_primary_classification", true);
    stationary_velocity_mps_ =
      declare_parameter<double>("arming.stationary_velocity_mps", 0.05);
    stationary_yaw_rate_rps_ =
      declare_parameter<double>("arming.stationary_yaw_rate_rps", 0.0);
    identity_timeout_sec_ = declare_parameter<double>("arming.identity_timeout_sec", 0.0);
    require_safe_vehicle_state_for_arm_ =
      declare_parameter<bool>("arming.require_safe_vehicle_state", true);
    provenance_log_path_ = declare_parameter<std::string>("evidence.provenance_log_path", "");

    // HH_260810 - Compare and bind fully resolved topic names so remaps cannot create a feedback loop.
    const auto resolve_topic = [this](const std::string & topic) {
        return get_node_topics_interface()->resolve_topic_name(topic, false);
      };
    physical_topic_ = resolve_topic(physical_topic_);
    virtual_topic_ = resolve_topic(virtual_topic_);
    diagnostic_topic_ = resolve_topic(diagnostic_topic_);
    output_topic_ = resolve_topic(output_topic_);
    status_topic_ = resolve_topic(status_topic_);
    velocity_topic_ = resolve_topic(velocity_topic_);
    gear_topic_ = resolve_topic(gear_topic_);
    control_mode_topic_ = resolve_topic(control_mode_topic_);

    validate_startup_contract();
    open_provenance_log();

    const auto reliable_qos = rclcpp::QoS{rclcpp::KeepLast(1)}.reliable().durability_volatile();
    // HH_260810 - Require one approved graph endpoint and bind metadata to its current DDS writer GID.
    physical_subscription_ = create_subscription<TrackedObjects>(
      physical_topic_, reliable_qos,
      std::bind(
        &VilsObjectIntegrationNode::on_physical_objects, this,
        std::placeholders::_1));
    virtual_subscription_ = create_subscription<TrackedObjects>(
      virtual_topic_, reliable_qos,
      std::bind(&VilsObjectIntegrationNode::on_virtual_objects, this, std::placeholders::_1));
    diagnostic_subscription_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostic_topic_, reliable_qos,
      std::bind(&VilsObjectIntegrationNode::on_diagnostics, this, std::placeholders::_1));

    // HH_260810 - Observe MANUAL PARK and standstill locally before accepting an explicit arm request.
    velocity_subscription_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
      velocity_topic_, reliable_qos,
      [this](const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        velocity_report_ = *message;
        velocity_receive_time_ = SteadyClock::now();
      });
    gear_subscription_ = create_subscription<autoware_vehicle_msgs::msg::GearReport>(
      gear_topic_, reliable_qos,
      [this](const autoware_vehicle_msgs::msg::GearReport::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        gear_report_ = *message;
        gear_receive_time_ = SteadyClock::now();
      });
    control_mode_subscription_ =
      create_subscription<autoware_vehicle_msgs::msg::ControlModeReport>(
      control_mode_topic_, reliable_qos,
      [this](const autoware_vehicle_msgs::msg::ControlModeReport::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        control_mode_report_ = *message;
        control_mode_receive_time_ = SteadyClock::now();
      });

    output_publisher_ = create_publisher<TrackedObjects>(output_topic_, reliable_qos);
    status_publisher_ = create_publisher<AcceptedStatus>(status_topic_, reliable_qos);
    arm_service_ = create_service<ArmSource>(
      "~/arm_pc4_source",
      std::bind(
        &VilsObjectIntegrationNode::on_arm_request, this,
        std::placeholders::_1, std::placeholders::_2));

    // HH_260810 - Expire pending joins and accepted snapshots on a steady clock independent of ROS time adjustments.
    maintenance_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&VilsObjectIntegrationNode::on_maintenance_timer, this));
    status_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&VilsObjectIntegrationNode::publish_status, this));

    last_transition_time_ = now();
    reason_ = contract_approved_ ? "awaiting_pc4_identity_and_manual_arm" :
      "contract_unapproved_fail_closed";
    if (!append_event("node_start", reason_, "", 0U, 0U) && contract_approved_) {
      throw std::runtime_error("failed to write initial VILS provenance event");
    }
  }

  ~VilsObjectIntegrationNode() override
  {
    append_event("node_stop", reason_, accepted_digest_, accepted_sequence_, accepted_count_);
    if (provenance_fd_ >= 0) {
      fsync(provenance_fd_);
      close(provenance_fd_);
      provenance_fd_ = -1;
    }
  }

private:
  struct PendingObject
  {
    TrackedObjects::ConstSharedPtr message;
    std::string digest;
    std::string writer_gid;
    std::string writer_node_fqn;
    SteadyClock::time_point receive_time;
  };

  struct Metadata
  {
    std::string digest;
    std::string session_id;
    std::string source_node_fqn;
    std::string source_writer_gid;
    std::string map_digest;
    std::string transform_digest;
    std::string serialization_contract;
    std::uint64_t sequence{0U};
    std::int64_t output_stamp_ns{0};
    std::size_t object_count{0U};
    bool source_ready{false};
    bool full_snapshot{false};
    SteadyClock::time_point receive_time;
  };

  static bool same_metadata_contract(const Metadata & lhs, const Metadata & rhs)
  {
    return lhs.digest == rhs.digest && lhs.session_id == rhs.session_id &&
           lhs.source_node_fqn == rhs.source_node_fqn &&
           lhs.source_writer_gid == rhs.source_writer_gid &&
           lhs.map_digest == rhs.map_digest &&
           lhs.transform_digest == rhs.transform_digest &&
           lhs.serialization_contract == rhs.serialization_contract &&
           lhs.sequence == rhs.sequence && lhs.output_stamp_ns == rhs.output_stamp_ns &&
           lhs.object_count == rhs.object_count && lhs.source_ready == rhs.source_ready &&
           lhs.full_snapshot == rhs.full_snapshot;
  }

  void validate_startup_contract()
  {
    // HH_260810 - Reject canonical and required modes unless their separate review gates are explicitly enabled.
    if (is_canonical_selection_mode(mode_) && !enable_canonical_selection_) {
      throw std::invalid_argument(
              "canonical VILS mode requested without enable_canonical_selection");
    }
    if (is_canonical_selection_mode(mode_) && !contract_approved_) {
      throw std::invalid_argument("canonical VILS mode requested with an unapproved contract");
    }
    // HH_260810 - A direct executable invocation may not bypass the pre-Stage-4 launch prohibition.
    if (is_canonical_selection_mode(mode_)) {
      throw std::invalid_argument(
              "canonical VILS modes are intentionally blocked until Stage 3 and READY/MRM review");
    }
    if (is_required_mode(mode_) && !enable_required_modes_) {
      throw std::invalid_argument("required VILS mode requested without enable_required_modes");
    }
    // HH_260810 - Required modes stay unavailable until the documented PC1/PC3 MRM gate exists.
    if (is_required_mode(mode_)) {
      throw std::invalid_argument(
              "required VILS modes are intentionally blocked until the MRM availability path is implemented");
    }
    if (mode_ == SourceMode::RealOnly) {
      throw std::invalid_argument(
              "the integration node must not be launched in real_only mode");
    }
    if (physical_topic_ == output_topic_ || virtual_topic_ == output_topic_ ||
      physical_topic_ == virtual_topic_)
    {
      throw std::invalid_argument("VILS input and output topics must be distinct");
    }
    if (!contract_approved_) {
      return;
    }
    const auto invalid_text = [](const std::string & value) {
        return value.empty() || value == "__UNAPPROVED__";
      };
    const auto finite = [](const double value) {return std::isfinite(value);};
    if (invalid_text(expected_frame_id_) || invalid_text(expected_map_digest_) ||
      invalid_text(expected_transform_digest_) || invalid_text(expected_serialization_contract_) ||
      invalid_text(approved_object_node_fqn_) || invalid_text(approved_diagnostic_node_fqn_) ||
      invalid_text(diagnostic_name_) || invalid_text(diagnostic_hardware_id_) ||
      approved_object_node_fqn_.front() != '/' || approved_diagnostic_node_fqn_.front() != '/' ||
      virtual_ttl_.count() <= 0 || metadata_join_timeout_.count() <= 0 ||
      !finite(max_source_age_sec_) || max_source_age_sec_ <= 0.0 ||
      !finite(max_future_skew_sec_) || max_future_skew_sec_ < 0.0 ||
      !finite(max_fusion_skew_sec_) || max_fusion_skew_sec_ <= 0.0 ||
      !finite(association_distance_m_) || association_distance_m_ <= 0.0 ||
      !finite(stationary_velocity_mps_) || stationary_velocity_mps_ < 0.0 ||
      !finite(stationary_yaw_rate_rps_) || stationary_yaw_rate_rps_ <= 0.0 ||
      !finite(identity_timeout_sec_) || identity_timeout_sec_ <= 0.0 ||
      !finite(safe_state_timeout_sec_) ||
      (is_canonical_selection_mode(mode_) && !require_safe_vehicle_state_for_arm_) ||
      (require_safe_vehicle_state_for_arm_ && safe_state_timeout_sec_ <= 0.0) ||
      max_pending_samples_ == 0U ||
      provenance_log_path_.empty())
    {
      throw std::invalid_argument("approved VILS contract contains unset or invalid values");
    }
    TrackedObjects empty;
    empty.header.frame_id = expected_frame_id_;
    const auto limits_result = validate_tracked_objects(empty, expected_frame_id_, limits_);
    if (!limits_result.valid) {
      throw std::invalid_argument(
              "approved VILS validation limits are invalid: " + limits_result.reason);
    }
  }

  void open_provenance_log()
  {
    if (provenance_log_path_.empty()) {
      return;
    }
    provenance_fd_ = open(
      provenance_log_path_.c_str(), O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC | O_NOFOLLOW,
      0600);
    if (provenance_fd_ < 0 && contract_approved_) {
      throw std::runtime_error("failed to open VILS provenance log: " + provenance_log_path_);
    }
    if (provenance_fd_ >= 0) {
      struct stat information {};
      if (fstat(provenance_fd_, &information) != 0 || !S_ISREG(information.st_mode) ||
        fchmod(provenance_fd_, 0600) != 0)
      {
        close(provenance_fd_);
        provenance_fd_ = -1;
        if (contract_approved_) {
          throw std::runtime_error("VILS provenance destination is not a private regular file");
        }
      }
    }
  }

  bool append_event(
    const std::string & event, const std::string & reason, const std::string & digest,
    const std::uint64_t sequence, const std::size_t object_count)
  {
    if (provenance_fd_ < 0) {
      return !contract_approved_;
    }
    // HH_260810 - Use system time for evidence writes so teardown never depends on a live ROS context.
    const auto wall_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const auto line = std::string{"{\"wall_time_ns\":"} + std::to_string(wall_ns) +
    ",\"event\":\"" + json_escape(event) + "\",\"reason\":\"" +
    json_escape(reason) + "\",\"mode\":\"" + json_escape(to_string(mode_)) +
    "\",\"armed\":" + (armed_ ? "true" : "false") +
    ",\"accepted\":" + (accepted_ ? "true" : "false") +
    ",\"session_id\":\"" + json_escape(observed_session_id_) +
    "\",\"writer_gid\":\"" + json_escape(observed_writer_gid_) +
    "\",\"diagnostic_writer_gid\":\"" +
    json_escape(observed_diagnostic_writer_gid_) +
    "\",\"sequence\":" + std::to_string(sequence) +
    ",\"object_count\":" + std::to_string(object_count) +
    ",\"snapshot_digest\":\"" + json_escape(digest) + "\"}\n";
    std::size_t offset = 0U;
    while (offset < line.size()) {
      const auto written = write(provenance_fd_, line.data() + offset, line.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      evidence_fault_ = true;
      return false;
    }
    if (fsync(provenance_fd_) != 0) {
      evidence_fault_ = true;
      return false;
    }
    return true;
  }

  bool validate_endpoint(
    const std::string & topic, const std::string & expected_type,
    const std::string & approved_node_fqn, std::string & resolved_node_fqn,
    std::string & resolved_writer_gid, std::string & reason)
  {
    std::vector<rclcpp::TopicEndpointInfo> endpoints;
    try {
      endpoints = get_publishers_info_by_topic(topic);
    } catch (const rclcpp::exceptions::RCLError &) {
      reason = "publisher_graph_query_failed";
      return false;
    } catch (const std::exception &) {
      reason = "publisher_graph_query_failed";
      return false;
    }
    if (endpoints.size() != 1U) {
      reason = "publisher_count_not_one";
      return false;
    }
    const auto & endpoint = endpoints.front();
    resolved_node_fqn = normalize_fqn(endpoint.node_namespace(), endpoint.node_name());
    if (resolved_node_fqn != approved_node_fqn) {
      reason = "publisher_node_not_approved";
      return false;
    }
    if (endpoint.topic_type() != expected_type) {
      reason = "publisher_type_mismatch";
      return false;
    }
    const auto & qos = endpoint.qos_profile();
    if (qos.reliability() != rclcpp::ReliabilityPolicy::Reliable ||
      qos.durability() != rclcpp::DurabilityPolicy::Volatile ||
      qos.history() != rclcpp::HistoryPolicy::KeepLast || qos.depth() != 1U)
    {
      reason = "publisher_qos_mismatch";
      return false;
    }
    resolved_writer_gid = gid_to_hex(endpoint.endpoint_gid());
    return true;
  }

  bool publish_output(const TrackedObjects & message)
  {
    if (evidence_fault_ && contract_approved_) {
      ++rejected_total_;
      disarm_and_clear("provenance_evidence_fault", true);
      return false;
    }
    if (is_canonical_selection_mode(mode_)) {
      try {
        const auto endpoints = get_publishers_info_by_topic(output_topic_);
        if (endpoints.size() != 1U ||
          normalize_fqn(
            endpoints.front().node_namespace(), endpoints.front().node_name()) !=
          get_fully_qualified_name() ||
          endpoints.front().topic_type() != "autoware_perception_msgs/msg/TrackedObjects")
        {
          ++rejected_total_;
          disarm_and_clear("canonical_output_owner_not_unique", true);
          return false;
        }
      } catch (const std::exception &) {
        ++rejected_total_;
        disarm_and_clear("canonical_output_graph_query_failed", true);
        return false;
      }
    }
    output_publisher_->publish(message);
    return true;
  }

  void on_virtual_objects(
    const TrackedObjects::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string node_fqn;
    std::string writer_gid;
    std::string rejection_reason;
    if (!contract_approved_) {
      reject("contract_unapproved", "", 0U, message->objects.size(), true);
      return;
    }
    if (!validate_endpoint(
        virtual_topic_, "autoware_perception_msgs/msg/TrackedObjects",
        approved_object_node_fqn_, node_fqn, writer_gid, rejection_reason))
    {
      reject(rejection_reason, "", 0U, message->objects.size(), true);
      return;
    }
    const auto content_result = validate_tracked_objects(*message, expected_frame_id_, limits_);
    if (!content_result.valid) {
      reject(content_result.reason, "", 0U, message->objects.size(), true);
      return;
    }
    if (!valid_stamp_fields(message->header.stamp)) {
      reject("invalid_source_stamp_fields", "", 0U, message->objects.size(), true);
      return;
    }
    const auto stamp_ns = stamp_to_nanoseconds(message->header.stamp);
    if (stamp_ns <= 0) {
      reject("nonpositive_source_stamp", "", 0U, message->objects.size(), true);
      return;
    }
    // HH_260810 - Serialize only an approved, bounded payload to avoid unauthorised hashing work.
    const auto digest = serialized_sha256(*message);
    // HH_260810 - Treat an already reviewed object digest as replay, never as a fresh heartbeat.
    if (!last_reviewed_digest_.empty() && digest == last_reviewed_digest_) {
      reject("replayed_object_digest", digest, last_sequence_, message->objects.size(), true);
      return;
    }
    const auto age = (now() - rclcpp::Time(message->header.stamp)).seconds();
    if (age > max_source_age_sec_) {
      reject("source_stamp_too_old", digest, 0U, message->objects.size(), true);
      return;
    }
    if (age < -max_future_skew_sec_) {
      reject("source_stamp_in_future", digest, 0U, message->objects.size(), true);
      return;
    }
    // HH_260810 - Defer monotonic stamp checks until metadata identifies the source session.
    if (pending_objects_.find(digest) != pending_objects_.end()) {
      reject("duplicate_pending_object_digest", digest, 0U, message->objects.size(), true);
      return;
    }
    if (pending_objects_.size() >= max_pending_samples_) {
      reject("pending_object_capacity_exceeded", digest, 0U, message->objects.size(), true);
      return;
    }
    pending_objects_[digest] = PendingObject{
      message, digest, writer_gid, node_fqn, SteadyClock::now()};
    try_join(digest);
  }

  std::optional<Metadata> parse_metadata(
    const diagnostic_msgs::msg::DiagnosticArray & message, std::string & reason)
  {
    const diagnostic_msgs::msg::DiagnosticStatus * selected = nullptr;
    for (const auto & status : message.status) {
      if (status.name == diagnostic_name_ && status.hardware_id == diagnostic_hardware_id_) {
        if (selected != nullptr) {
          reason = "duplicate_diagnostic_status";
          return std::nullopt;
        }
        selected = &status;
      }
    }
    if (selected == nullptr) {
      reason = "diagnostic_status_not_found";
      return std::nullopt;
    }
    if (selected->level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      reason = "diagnostic_status_not_ok";
      return std::nullopt;
    }

    std::map<std::string, std::string> values;
    for (const auto & item : selected->values) {
      if (!values.emplace(item.key, item.value).second) {
        reason = "duplicate_diagnostic_key";
        return std::nullopt;
      }
    }
    const std::array<const char *, 11> required_keys{
      "output_cdr_sha256", "session_id", "source_node_fqn", "source_writer_gid",
      "map_digest", "transform_digest", "serialization_contract", "sequence",
      "output_stamp_ns", "output_object_count", "source_ready"};
    for (const auto key : required_keys) {
      if (values.find(key) == values.end()) {
        reason = std::string{"missing_diagnostic_key:"} + key;
        return std::nullopt;
      }
    }
    if (values.find("full_snapshot") == values.end()) {
      reason = "missing_diagnostic_key:full_snapshot";
      return std::nullopt;
    }

    Metadata metadata;
    metadata.digest = values.at("output_cdr_sha256");
    metadata.session_id = values.at("session_id");
    metadata.source_node_fqn = values.at("source_node_fqn");
    metadata.source_writer_gid = values.at("source_writer_gid");
    metadata.map_digest = values.at("map_digest");
    metadata.transform_digest = values.at("transform_digest");
    metadata.serialization_contract = values.at("serialization_contract");
    try {
      std::uint64_t object_count = 0U;
      if (!parse_decimal_exact(values.at("sequence"), metadata.sequence) ||
        !parse_decimal_exact(values.at("output_stamp_ns"), metadata.output_stamp_ns) ||
        !parse_decimal_exact(values.at("output_object_count"), object_count) ||
        object_count > std::numeric_limits<std::size_t>::max())
      {
        throw std::invalid_argument("strict decimal parse failed");
      }
      metadata.object_count = static_cast<std::size_t>(object_count);
    } catch (const std::exception &) {
      reason = "invalid_diagnostic_numeric_value";
      return std::nullopt;
    }
    // HH_260810 - Bound free-form DiagnosticArray fields before they enter source state.
    if (!is_lowercase_hex(metadata.digest, SHA256_DIGEST_LENGTH * 2U) ||
      !is_lowercase_hex(metadata.source_writer_gid, RMW_GID_STORAGE_SIZE * 2U) ||
      metadata.digest.find_first_not_of('0') == std::string::npos ||
      metadata.source_writer_gid.find_first_not_of('0') == std::string::npos ||
      !is_safe_session_id(metadata.session_id) ||
      metadata.source_node_fqn.empty() || metadata.source_node_fqn.front() != '/' ||
      metadata.sequence == 0U || metadata.output_stamp_ns <= 0)
    {
      reason = "invalid_diagnostic_identity_value";
      return std::nullopt;
    }
    if (!parse_bool(values.at("source_ready"), metadata.source_ready) ||
      !parse_bool(values.at("full_snapshot"), metadata.full_snapshot))
    {
      reason = "invalid_diagnostic_boolean_value";
      return std::nullopt;
    }
    metadata.receive_time = SteadyClock::now();
    return metadata;
  }

  void on_diagnostics(
    const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string node_fqn;
    std::string writer_gid;
    std::string rejection_reason;
    if (!contract_approved_) {
      reject("contract_unapproved", "", 0U, 0U, true);
      return;
    }
    if (!validate_endpoint(
        diagnostic_topic_, "diagnostic_msgs/msg/DiagnosticArray",
        approved_diagnostic_node_fqn_, node_fqn, writer_gid, rejection_reason))
    {
      reject(rejection_reason, "", 0U, 0U, true);
      return;
    }
    auto metadata = parse_metadata(*message, rejection_reason);
    if (!metadata) {
      reject(rejection_reason, "", 0U, 0U, true);
      return;
    }
    // HH_260810 - Bind the metadata writer separately and require manual re-arm after restart.
    if (observed_diagnostic_writer_gid_.empty() ||
      observed_diagnostic_writer_gid_ != writer_gid)
    {
      observed_diagnostic_writer_gid_ = writer_gid;
      disarm_and_clear("diagnostic_writer_identity_change_manual_rearm_required", true);
    }
    // HH_260810 - Reject repeated metadata instead of silently replacing a pending contract sample.
    const auto pending = pending_metadata_.find(metadata->digest);
    if (pending != pending_metadata_.end()) {
      if (!same_metadata_contract(pending->second, *metadata)) {
        reject(
          "conflicting_pending_metadata_digest", metadata->digest, metadata->sequence,
          metadata->object_count, true);
      }
      return;
    }
    if (pending_metadata_.size() >= max_pending_samples_) {
      reject(
        "pending_metadata_capacity_exceeded", metadata->digest, metadata->sequence,
        metadata->object_count, true);
      return;
    }
    // HH_260810 - Permit an unchanged DiagnosticArray heartbeat without extending object TTL.
    if (!observed_session_id_.empty() && metadata->session_id == observed_session_id_ &&
      metadata->source_writer_gid == observed_writer_gid_ && metadata->sequence <= last_sequence_)
    {
      if (metadata->sequence == last_sequence_ && metadata->digest == last_reviewed_digest_) {
        return;
      }
      reject(
        "replayed_or_conflicting_metadata", metadata->digest, metadata->sequence,
        metadata->object_count, true);
      return;
    }
    pending_metadata_[metadata->digest] = *metadata;
    try_join(metadata->digest);
  }

  void try_join(const std::string & digest)
  {
    const auto object_it = pending_objects_.find(digest);
    const auto metadata_it = pending_metadata_.find(digest);
    if (object_it == pending_objects_.end() || metadata_it == pending_metadata_.end()) {
      return;
    }
    const auto object = object_it->second;
    const auto metadata = metadata_it->second;
    pending_objects_.erase(object_it);
    pending_metadata_.erase(metadata_it);

    if (object.writer_gid != metadata.source_writer_gid ||
      object.writer_node_fqn != metadata.source_node_fqn)
    {
      reject(
        "metadata_source_identity_mismatch", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    if (metadata.source_node_fqn != approved_object_node_fqn_ ||
      metadata.map_digest != expected_map_digest_ ||
      metadata.transform_digest != expected_transform_digest_ ||
      metadata.serialization_contract != expected_serialization_contract_)
    {
      reject(
        "metadata_contract_mismatch", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    if (!metadata.source_ready || !metadata.full_snapshot) {
      reject(
        "source_not_ready_or_not_full_snapshot", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    if (object.message->objects.empty() && !allow_empty_snapshot_) {
      reject(
        "empty_snapshot_not_approved_for_scenario", digest, metadata.sequence, 0U, true);
      return;
    }
    if (metadata.output_stamp_ns != stamp_to_nanoseconds(object.message->header.stamp) ||
      metadata.object_count != object.message->objects.size())
    {
      reject(
        "metadata_stamp_or_count_mismatch", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    last_reviewed_receive_time_ = SteadyClock::now();

    // HH_260810 - Reject session reuse and require a new session whenever the DDS writer changes.
    if (!observed_session_id_.empty() && observed_session_id_ == metadata.session_id &&
      observed_writer_gid_ != metadata.source_writer_gid)
    {
      reject(
        "writer_change_without_new_session", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    if (!observed_session_id_.empty() && observed_session_id_ != metadata.session_id &&
      seen_session_ids_.find(metadata.session_id) != seen_session_ids_.end())
    {
      reject(
        "reused_session_id", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }

    // HH_260810 - Treat every approved writer or session transition as a manual-rearm boundary.
    if (observed_session_id_.empty() || observed_session_id_ != metadata.session_id ||
      observed_writer_gid_.empty() || observed_writer_gid_ != metadata.source_writer_gid)
    {
      seen_session_ids_.insert(metadata.session_id);
      observed_session_id_ = metadata.session_id;
      observed_writer_gid_ = metadata.source_writer_gid;
      observed_source_node_fqn_ = metadata.source_node_fqn;
      last_sequence_ = metadata.sequence;
      last_reviewed_source_stamp_ns_ = metadata.output_stamp_ns;
      last_reviewed_digest_ = digest;
      disarm_and_clear("source_identity_change_manual_rearm_required", true);
      return;
    }
    if (metadata.sequence <= last_sequence_) {
      reject(
        "duplicate_out_of_order_or_replayed_sequence", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    if (metadata.output_stamp_ns <= last_reviewed_source_stamp_ns_) {
      reject(
        "nonmonotonic_object_stamp", digest, metadata.sequence,
        object.message->objects.size(), true);
      return;
    }
    last_sequence_ = metadata.sequence;
    last_reviewed_source_stamp_ns_ = metadata.output_stamp_ns;
    last_reviewed_digest_ = digest;
    if (!armed_) {
      reject(
        "valid_snapshot_but_source_disarmed", digest, metadata.sequence,
        object.message->objects.size(), false);
      return;
    }

    accepted_virtual_objects_ = *object.message;
    accepted_digest_ = digest;
    accepted_sequence_ = metadata.sequence;
    accepted_receive_time_ = SteadyClock::now();
    accepted_source_stamp_ = object.message->header.stamp;
    accepted_count_ = object.message->objects.size();
    accepted_ = true;
    reason_ = "accepted";
    last_transition_time_ = now();
    if (!append_event("snapshot_accepted", reason_, digest, metadata.sequence, accepted_count_)) {
      ++rejected_total_;
      disarm_and_clear("provenance_evidence_fault", true);
      return;
    }
    ++accepted_total_;

    if (mode_ == SourceMode::VirtualOnlyRequired) {
      publish_output(accepted_virtual_objects_);
    }
  }

  void on_physical_objects(const TrackedObjects::ConstSharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode_ == SourceMode::VirtualOnlyRequired) {
      return;
    }
    // HH_260810 - Never perform time association or selected-output publication from a malformed stamp.
    if (!valid_stamp_fields(message->header.stamp) ||
      stamp_to_nanoseconds(message->header.stamp) <= 0)
    {
      reason_ = "invalid_physical_stamp";
      ++rejected_total_;
      append_event("physical_rejected", reason_, "", 0U, message->objects.size());
      return;
    }
    if (message->header.frame_id != expected_frame_id_) {
      reason_ = "physical_frame_mismatch";
      ++rejected_total_;
      append_event("physical_rejected", reason_, "", 0U, message->objects.size());
      return;
    }
    const auto virtual_available = accepted_snapshot_is_fresh();
    if (!virtual_available) {
      if (mode_ == SourceMode::Shadow || mode_ == SourceMode::HybridOptional) {
        publish_output(*message);
      }
      return;
    }
    const auto fusion_skew = std::abs(
      (rclcpp::Time(message->header.stamp) - rclcpp::Time(accepted_source_stamp_)).seconds());
    if (fusion_skew > max_fusion_skew_sec_) {
      reason_ = "fusion_stamp_skew";
      ++rejected_total_;
      append_event(
        "fusion_skipped", reason_, accepted_digest_, accepted_sequence_,
        accepted_count_);
      if (mode_ == SourceMode::Shadow || mode_ == SourceMode::HybridOptional) {
        publish_output(*message);
      }
      return;
    }
    FusionStats stats;
    auto fused = fuse_physical_main(
      *message, accepted_virtual_objects_, association_distance_m_,
      require_matching_classification_, stats);
    if (fused.objects.size() > limits_.max_objects) {
      ++rejected_total_;
      disarm_and_clear("fused_object_count_out_of_range", true);
      if (mode_ == SourceMode::Shadow || mode_ == SourceMode::HybridOptional) {
        publish_output(*message);
      }
      return;
    }
    reason_ = "accepted";
    if (!append_event(
        "physical_triggered_fusion", "selected", accepted_digest_, accepted_sequence_,
        fused.objects.size()))
    {
      ++rejected_total_;
      disarm_and_clear("provenance_evidence_fault", true);
      return;
    }
    publish_output(fused);
  }

  bool accepted_snapshot_is_fresh()
  {
    if (!accepted_ || !armed_) {
      return false;
    }
    if (SteadyClock::now() - accepted_receive_time_ > virtual_ttl_) {
      disarm_and_clear("virtual_ttl_expired", true);
      return false;
    }
    return true;
  }

  bool safe_vehicle_state(std::string & reason) const
  {
    if (!require_safe_vehicle_state_for_arm_) {
      return true;
    }
    if (!velocity_report_ || !gear_report_ || !control_mode_report_ ||
      !velocity_receive_time_ || !gear_receive_time_ || !control_mode_receive_time_)
    {
      reason = "vehicle_safe_state_missing";
      return false;
    }
    const auto now_steady = SteadyClock::now();
    const auto max_age = std::chrono::duration<double>(safe_state_timeout_sec_);
    if (now_steady - *velocity_receive_time_ > max_age ||
      now_steady - *gear_receive_time_ > max_age ||
      now_steady - *control_mode_receive_time_ > max_age)
    {
      reason = "vehicle_safe_state_stale";
      return false;
    }
    if (!std::isfinite(velocity_report_->longitudinal_velocity) ||
      !std::isfinite(velocity_report_->lateral_velocity) ||
      !std::isfinite(velocity_report_->heading_rate))
    {
      reason = "vehicle_velocity_not_finite";
      return false;
    }
    if (std::abs(velocity_report_->longitudinal_velocity) > stationary_velocity_mps_ ||
      std::abs(velocity_report_->lateral_velocity) > stationary_velocity_mps_ ||
      std::abs(velocity_report_->heading_rate) > stationary_yaw_rate_rps_)
    {
      reason = "vehicle_not_stationary";
      return false;
    }
    if (gear_report_->report != autoware_vehicle_msgs::msg::GearReport::PARK) {
      reason = "vehicle_not_in_park";
      return false;
    }
    if (control_mode_report_->mode != autoware_vehicle_msgs::msg::ControlModeReport::MANUAL) {
      reason = "vehicle_not_in_manual";
      return false;
    }
    return true;
  }

  void on_arm_request(
    const ArmSource::Request::SharedPtr request, ArmSource::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!request->arm) {
      disarm_and_clear("operator_disarm", true);
      response->success = true;
      response->reason = reason_;
      return;
    }
    // HH_260810 - Reject redundant arm requests so accepted cache cannot be half-reset in place.
    if (armed_) {
      response->success = false;
      response->reason = "source_already_armed";
      return;
    }
    if (!contract_approved_) {
      response->success = false;
      response->reason = "contract_unapproved";
      return;
    }
    if (evidence_fault_) {
      response->success = false;
      response->reason = "provenance_evidence_fault";
      return;
    }
    if (observed_session_id_.empty() || observed_writer_gid_.empty()) {
      response->success = false;
      response->reason = "no_reviewable_source_identity_observed";
      return;
    }
    if (!last_reviewed_receive_time_ ||
      SteadyClock::now() - *last_reviewed_receive_time_ >
      std::chrono::duration<double>(identity_timeout_sec_))
    {
      response->success = false;
      response->reason = "source_identity_stale";
      return;
    }
    std::string resolved_node_fqn;
    std::string resolved_writer_gid;
    std::string endpoint_reason;
    if (!validate_endpoint(
        virtual_topic_, "autoware_perception_msgs/msg/TrackedObjects",
        approved_object_node_fqn_, resolved_node_fqn, resolved_writer_gid, endpoint_reason) ||
      resolved_writer_gid != observed_writer_gid_ ||
      !validate_endpoint(
        diagnostic_topic_, "diagnostic_msgs/msg/DiagnosticArray",
        approved_diagnostic_node_fqn_, resolved_node_fqn, resolved_writer_gid,
        endpoint_reason) || resolved_writer_gid != observed_diagnostic_writer_gid_)
    {
      response->success = false;
      response->reason = "source_endpoint_unavailable:" + endpoint_reason;
      return;
    }
    if (request->session_id != observed_session_id_ ||
      request->writer_gid != observed_writer_gid_ ||
      request->diagnostic_writer_gid != observed_diagnostic_writer_gid_ ||
      request->map_digest != expected_map_digest_ ||
      request->transform_digest != expected_transform_digest_)
    {
      response->success = false;
      response->reason = "arm_identity_mismatch";
      return;
    }
    std::string safe_state_reason;
    if (!safe_vehicle_state(safe_state_reason)) {
      response->success = false;
      response->reason = safe_state_reason;
      return;
    }
    armed_ = true;
    accepted_ = false;
    reason_ = "armed_awaiting_fresh_snapshot";
    last_transition_time_ = now();
    if (!append_event("operator_arm", reason_, "", last_sequence_, 0U)) {
      armed_ = false;
      response->success = false;
      response->reason = "provenance_evidence_fault";
      return;
    }
    response->success = true;
    response->reason = reason_;
  }

  void disarm_and_clear(const std::string & reason, const bool clear_pending)
  {
    armed_ = false;
    accepted_ = false;
    accepted_virtual_objects_ = TrackedObjects{};
    accepted_digest_.clear();
    accepted_sequence_ = 0U;
    accepted_count_ = 0U;
    accepted_source_stamp_ = builtin_interfaces::msg::Time{};
    if (clear_pending) {
      pending_objects_.clear();
      pending_metadata_.clear();
    }
    reason_ = reason;
    last_transition_time_ = now();
    append_event("source_disarmed", reason_, "", last_sequence_, 0U);
  }

  void reject(
    const std::string & reason, const std::string & digest, const std::uint64_t sequence,
    const std::size_t object_count, const bool disarm)
  {
    ++rejected_total_;
    reason_ = reason;
    append_event("snapshot_rejected", reason, digest, sequence, object_count);
    if (disarm) {
      disarm_and_clear(reason, true);
    }
  }

  void on_maintenance_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto now_steady = SteadyClock::now();
    bool join_expired = false;
    for (auto iterator = pending_objects_.begin(); iterator != pending_objects_.end(); ) {
      if (now_steady - iterator->second.receive_time > metadata_join_timeout_) {
        iterator = pending_objects_.erase(iterator);
        join_expired = true;
      } else {
        ++iterator;
      }
    }
    for (auto iterator = pending_metadata_.begin(); iterator != pending_metadata_.end(); ) {
      if (now_steady - iterator->second.receive_time > metadata_join_timeout_) {
        iterator = pending_metadata_.erase(iterator);
        join_expired = true;
      } else {
        ++iterator;
      }
    }
    if (join_expired) {
      reject("metadata_join_timeout", "", 0U, 0U, true);
    }
    if (accepted_ && now_steady - accepted_receive_time_ > virtual_ttl_) {
      disarm_and_clear("virtual_ttl_expired", true);
    }
  }

  std::uint8_t status_mode() const
  {
    return static_cast<std::uint8_t>(mode_);
  }

  void publish_status()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    AcceptedStatus status;
    status.header.stamp = now();
    status.header.frame_id = expected_frame_id_;
    status.mode = status_mode();
    // HH_260810 - Evaluate TTL before copying armed state so one status sample cannot report stale arming.
    status.accepted = accepted_snapshot_is_fresh();
    status.armed = armed_;
    status.available = status.accepted;
    status.reason = reason_;
    status.source_node_fqn = observed_source_node_fqn_;
    status.source_writer_gid = observed_writer_gid_;
    status.diagnostic_writer_gid = observed_diagnostic_writer_gid_;
    status.session_id = observed_session_id_;
    status.sequence = accepted_sequence_;
    status.source_stamp = accepted_source_stamp_;
    if (accepted_) {
      const auto elapsed = SteadyClock::now() - accepted_receive_time_;
      status.receive_age_sec = std::chrono::duration<double>(elapsed).count();
      const auto ttl_duration = std::chrono::duration_cast<SteadyClock::duration>(virtual_ttl_);
      const auto remaining = elapsed < ttl_duration ? ttl_duration - elapsed :
        SteadyClock::duration::zero();
      const auto ttl_deadline_ns = now().nanoseconds() +
        std::chrono::duration_cast<std::chrono::nanoseconds>(remaining).count();
      status.ttl_deadline = nanoseconds_to_stamp(ttl_deadline_ns);
    }
    status.map_digest = expected_map_digest_;
    status.transform_digest = expected_transform_digest_;
    status.serialization_contract = expected_serialization_contract_;
    status.object_count = static_cast<std::uint32_t>(accepted_count_);
    status.snapshot_digest = accepted_digest_;
    status.last_transition_time = last_transition_time_;
    status.accepted_count = accepted_total_;
    status.rejected_count = rejected_total_;
    status_publisher_->publish(status);
  }

  SourceMode mode_{SourceMode::RealOnly};
  bool contract_approved_{false};
  bool enable_canonical_selection_{false};
  bool enable_required_modes_{false};
  bool require_matching_classification_{true};
  bool require_safe_vehicle_state_for_arm_{true};

  std::string physical_topic_;
  std::string virtual_topic_;
  std::string diagnostic_topic_;
  std::string output_topic_;
  std::string status_topic_;
  std::string velocity_topic_;
  std::string gear_topic_;
  std::string control_mode_topic_;
  std::string expected_frame_id_;
  std::string expected_map_digest_;
  std::string expected_transform_digest_;
  std::string expected_serialization_contract_;
  std::string approved_object_node_fqn_;
  std::string approved_diagnostic_node_fqn_;
  std::string diagnostic_name_;
  std::string diagnostic_hardware_id_;
  bool allow_empty_snapshot_{false};
  std::string provenance_log_path_;

  ValidationLimits limits_;
  std::chrono::milliseconds virtual_ttl_{0};
  std::chrono::milliseconds metadata_join_timeout_{0};
  double max_source_age_sec_{0.0};
  double max_future_skew_sec_{0.0};
  double max_fusion_skew_sec_{0.0};
  double safe_state_timeout_sec_{1.0};
  double association_distance_m_{0.0};
  double stationary_velocity_mps_{0.05};
  double stationary_yaw_rate_rps_{0.0};
  double identity_timeout_sec_{0.0};

  std::mutex mutex_;
  std::unordered_map<std::string, PendingObject> pending_objects_;
  std::unordered_map<std::string, Metadata> pending_metadata_;
  std::set<std::string> seen_session_ids_;
  TrackedObjects accepted_virtual_objects_;
  std::string accepted_digest_;
  std::string observed_session_id_;
  std::string observed_writer_gid_;
  std::string observed_diagnostic_writer_gid_;
  std::string observed_source_node_fqn_;
  std::string last_reviewed_digest_;
  builtin_interfaces::msg::Time accepted_source_stamp_;
  SteadyClock::time_point accepted_receive_time_{};
  rclcpp::Time last_transition_time_{0, 0, RCL_ROS_TIME};
  std::uint64_t accepted_sequence_{0U};
  std::uint64_t last_sequence_{0U};
  std::uint64_t accepted_total_{0U};
  std::uint64_t rejected_total_{0U};
  std::int64_t last_reviewed_source_stamp_ns_{0};
  std::size_t accepted_count_{0U};
  std::size_t max_pending_samples_{0U};
  bool armed_{false};
  bool accepted_{false};
  bool evidence_fault_{false};
  std::string reason_{"initializing"};

  std::optional<autoware_vehicle_msgs::msg::VelocityReport> velocity_report_;
  std::optional<autoware_vehicle_msgs::msg::GearReport> gear_report_;
  std::optional<autoware_vehicle_msgs::msg::ControlModeReport> control_mode_report_;
  std::optional<SteadyClock::time_point> velocity_receive_time_;
  std::optional<SteadyClock::time_point> gear_receive_time_;
  std::optional<SteadyClock::time_point> control_mode_receive_time_;
  std::optional<SteadyClock::time_point> last_reviewed_receive_time_;

  rclcpp::Subscription<TrackedObjects>::SharedPtr physical_subscription_;
  rclcpp::Subscription<TrackedObjects>::SharedPtr virtual_subscription_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_subscription_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_subscription_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::GearReport>::SharedPtr gear_subscription_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr
    control_mode_subscription_;
  rclcpp::Publisher<TrackedObjects>::SharedPtr output_publisher_;
  rclcpp::Publisher<AcceptedStatus>::SharedPtr status_publisher_;
  rclcpp::Service<ArmSource>::SharedPtr arm_service_;
  rclcpp::TimerBase::SharedPtr maintenance_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  int provenance_fd_{-1};
};

}  // namespace autoware::vils_object_integration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<autoware::vils_object_integration::VilsObjectIntegrationNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("vils_object_integration"), "%s", exception.what());
    rclcpp::shutdown();
    return 2;
  }
  rclcpp::shutdown();
  return 0;
}
