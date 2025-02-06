// Copyright 2021 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef ROS2_SOCKETCAN__SOCKET_CAN_SENDER_NODE_HPP_
#define ROS2_SOCKETCAN__SOCKET_CAN_SENDER_NODE_HPP_

#include <memory>
#include <string>

//HH_230308 // HH_250107 - autoware auto msg -> autoware msg
#include "std_msgs/msg/bool.hpp"
#include <std_msgs//msg/float64.hpp>
#include <std_msgs/msg/int32.hpp> 
#include <std_msgs/msg/int8.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include "ros2_socketcan/visibility_control.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace drivers
{
namespace socketcan
{
/// \brief SocketCanSenderNode class which can pass messages
/// from CAN hardware or virtual channels
class SOCKETCAN_PUBLIC SocketCanSenderNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  explicit SocketCanSenderNode(rclcpp::NodeOptions options);

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback for ros can frame.
  void on_frame(const can_msgs::msg::Frame::SharedPtr msg);

  /// \brief Callback for ros can fd frame.
  void on_fd_frame(const ros2_socketcan_msgs::msg::FdFrame::SharedPtr msg);

  //HH_230308
  void current_cruise_mode_callback(const std_msgs::msg::Bool msg);
  can_msgs::msg::Frame generateFrame_IONIQ_EV_VCU();
  can_msgs::msg::Frame generateFrame_IONIQ_ADS2EPS_ACC();
  //HH_230317 
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> current_cruise_mode;
private:
  std::string interface_;
  std::unique_ptr<SocketCanSender> sender_;
  std::chrono::nanoseconds timeout_ns_;
  bool enable_fd_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr frames_sub_;
  rclcpp::Subscription<ros2_socketcan_msgs::msg::FdFrame>::SharedPtr fd_frames_sub_;

  //HH_230308  
  unsigned char CAN_alive_count;
  static double steering_cmd_;
  static double throttle_cmd__;
  std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> can1_156_frames_pub_;
  std::shared_ptr<lc::LifecyclePublisher<can_msgs::msg::Frame>> can1_157_frames_pub_;
  

};
}  // namespace socketcan
}  // namespace drivers

#endif  // ROS2_SOCKETCAN__SOCKET_CAN_SENDER_NODE_HPP_
