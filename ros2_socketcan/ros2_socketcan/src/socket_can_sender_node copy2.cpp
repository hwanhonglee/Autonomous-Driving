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

#include "ros2_socketcan/socket_can_sender_node.hpp"
#include "ros2_socketcan/socket_can_common.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
  
namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace drivers
{
namespace socketcan
{
SocketCanSenderNode::SocketCanSenderNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("socket_can_sender_node", options)
{
  interface_ = this->declare_parameter("interface", "can0");
  enable_fd_ = this->declare_parameter("enable_can_fd", false);
  double timeout_sec = this->declare_parameter("timeout_sec", 0.01);
  timeout_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(timeout_sec));

  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "can fd enabled: %s", enable_fd_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "timeout(s): %f", timeout_sec);
}

LNI::CallbackReturn SocketCanSenderNode::on_configure(const lc::State & state)
{
  (void)state;

  try {
    sender_ = std::make_unique<SocketCanSender>(interface_, enable_fd_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening CAN sender: %s - %s",
      interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Sender successfully configured.");

  if (!enable_fd_) {
    frames_sub_ = this->create_subscription<can_msgs::msg::Frame>(
      "to_can_bus", 500, std::bind(&SocketCanSenderNode::on_frame, this, std::placeholders::_1)); //HH_230308  //to_can_bus
  } else {
    fd_frames_sub_ = this->create_subscription<ros2_socketcan_msgs::msg::FdFrame>(
      "to_can_bus_fd", 500, std::bind( //HH_230308  //to_can_bus_fd
        &SocketCanSenderNode::on_fd_frame, this,
        std::placeholders::_1));
  }

  //Subscribe
  //HH_230308
  current_cruise_mode = this->create_subscription<std_msgs::msg::Bool>(
      "/current_cruise_mode_status", 
      10, 
      std::bind(&SocketCanSenderNode::current_cruise_mode_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "cruise_mode_state0: %d", current_cruise_mode);
  //Publisher
  //HH_230308
  can1_156_frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 500);
  can1_157_frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("can1_157_data", 500);  

  // auto throttle_cmd = this->create_subscription<autoware_auto_vehicle_msgs::msg::SteeringReport>(
  //   "throttle_command", 10, std::bind(&SocketCanSenderNode::current_throttle_callback, this, std::placeholders::_1));
    //std::cout<< "cruise_mode_report : "<< cruise_mode_report <<std::endl;

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_activate(const lc::State & state)
{
  (void)state;
  //HH_230313
  can1_157_frames_pub_->on_activate();
  can1_156_frames_pub_->on_activate();
  RCLCPP_DEBUG(this->get_logger(), "Sender activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_deactivate(const lc::State & state)
{
  (void)state;
  //HH_230313
  can1_157_frames_pub_->on_deactivate();
  can1_156_frames_pub_->on_deactivate();
  RCLCPP_DEBUG(this->get_logger(), "Sender deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_cleanup(const lc::State & state)
{
  (void)state;

  if (!enable_fd_) {
    frames_sub_.reset();
  } else {
    fd_frames_sub_.reset();
  }

  RCLCPP_DEBUG(this->get_logger(), "Sender cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanSenderNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Sender shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

//HH_230313
void SocketCanSenderNode::current_cruise_mode_callback(const std_msgs::msg::Bool msg)
{
  // std::shared_ptr<std_msgs::msg::Bool> current_cruise_mode;
  std_msgs::msg::Bool current_cruise_mode = msg;
  RCLCPP_INFO(this->get_logger(), "cruise_mode_state2: %d", current_cruise_mode);

  return;
}

//HH_230308
// void SocketCanSenderNode::current_throttle_callback(const autoware_auto_control_msgs::msg::acceleration::SharedPtr msg)
// {
//   throttle_tmp = msg->acceleration;
//   throttle_cmd = throttle_tmp;
//   //HH_230201
//   if (throttle_cmd >= 1) throttle_cmd = 1.0;
//   else if (throttle_cmd == 0) throttle_cmd = 0.1;  
//   else if (throttle_cmd <= -3) throttle_cmd = -3.0;
// }

//HH_230308
// void current_steering_callback(const autoware_auto_vehicle_msgs::msg::SteeringReport::SharedPtr msg)
// {
//   auto steering_cmd = msg->steering_tire_angle; //maybe input -> angle, not torque
//   if (steering_cmd > 438) steering_cmd = 438.;
//   if (steering_cmd < -438) steering_cmd = -438.;
// }

//HH_230308
can_msgs::msg::Frame SocketCanSenderNode::generateFrame_IONIQ_CtrlVCU()
{
  // //HH_230308
  // std_msgs::msg::Bool current_cruise_mode;
  bool current_cruise_mode = true;
  RCLCPP_INFO(this->get_logger(), "cruise_mode_state: %d", current_cruise_mode);
  
  can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  frame_msg.header.frame_id = "IONIQ_CtrlVCU";
  frame_msg.header.stamp = rclcpp::Clock().now();
  frame_msg.id = 342; //0x156
  frame_msg.is_rtr = false;
  frame_msg.is_extended = false;
  frame_msg.is_error = false;
  frame_msg.dlc = 8;

  //cruise_mode on
  frame_msg.data[0] = (current_cruise_mode ? 1 : 0) & 0x01; //EPS_En
  frame_msg.data[1] = 0;  //
  frame_msg.data[2] = (current_cruise_mode ? 1 : 0) & 0x01; // ? 65 : 0) & 0x41로 프로토콜처럼 입력 가능하지만 0100 0000, AEB가 켜지면 후방 충돌음이 계속 들림.// ACC_En, AEB_En //HH_221207
  frame_msg.data[3] = 0;
  frame_msg.data[4] = 0;
  frame_msg.data[5] = 0; // trun_signal, hazard, Gear_cmd
  frame_msg.data[6] = 0;
  frame_msg.data[7] = CAN_alive_count++;             // alive-count  
  //std::cout<< "cruise_mode_report : "<< cruise_mode_report <<std::endl;
  
  return frame_msg;
}

//HH_230308
can_msgs::msg::Frame SocketCanSenderNode::generateFrame_IONIQ_ADS2EPS_ACC() //EPS_REQ ACC_REQ
{
    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

    frame_msg.header.frame_id = "IONIQ_ADS2EPS_ACC";
    frame_msg.header.stamp = rclcpp::Clock().now();
    frame_msg.id = 343; //0x157
    frame_msg.is_rtr = false;
    frame_msg.is_extended = false;
    frame_msg.is_error = false;
    frame_msg.dlc = 8;

    //cruise_mode on
    // frame_msg.data[0] = ((char)(double)(10.0*steering_cmd) & 0x0FF); //steering_cmd
    // frame_msg.data[1] = ((char)(double)(10.0*steering_cmd) >> 8 & 0xFF00); //steering_cmd
    // frame_msg.data[2] = 0;
    // frame_msg.data[3] = ((char)(double)(100.0*((throttle_cmd) + 10.23)) & 0x00FF); //throttle_cmd
    // frame_msg.data[4] = ((char)(double)(100.0*((throttle_cmd) + 10.23)) >> 8 & 0x00FF); //throttle_cmd))
    // throttle_cmd  Acceleration Control    100*((value)+10.23) [-5.0, 5.0]    멈추려면 -값 주면 된다고 함.
    frame_msg.data[5] = 0;
    frame_msg.data[6] = 0;
    frame_msg.data[7] = 0;

    return frame_msg;
}

void SocketCanSenderNode::on_frame(const can_msgs::msg::Frame::SharedPtr msg)
{ 
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    FrameType type;
    if (msg->is_rtr) {
      type = FrameType::REMOTE;
    } else if (msg->is_error) {
      type = FrameType::ERROR;
    } else {
      type = FrameType::DATA;
    }

    CanId send_id = msg->is_extended ? CanId(msg->id, 0, type, ExtendedFrame) :
      CanId(msg->id, 0, type, StandardFrame);
  // RCLCPP_INFO(this->get_logger(), "cruise_mode_state3: %d", current_cruise_mode);
    //HH_230320
    // CAN 메시지 데이터 생성
    can_msgs::msg::Frame send_frame_1(rosidl_runtime_cpp::MessageInitialization::ZERO);
    send_frame_1 = generateFrame_IONIQ_CtrlVCU();
    can1_156_frames_pub_->publish(std::move(send_frame_1));

    //HH_230321
    can_msgs::msg::Frame send_frame_2(rosidl_runtime_cpp::MessageInitialization::ZERO);
    send_frame_2 = generateFrame_IONIQ_ADS2EPS_ACC();
    can1_157_frames_pub_->publish(std::move(send_frame_2));  

    // RCLCPP_INFO(get_logger(), "frame_156: %d %d %d %d %d %d %d %d %d %d %d", 
    //     send_frame.id, send_frame.is_extended, send_frame.dlc,
    //     send_frame.data[0], send_frame.data[1], send_frame.data[2], send_frame.data[3],
    //     send_frame.data[4], send_frame.data[5], send_frame.data[6], send_frame.data[7]);

    try {
      sender_->send(msg->data.data(), msg->dlc, send_id, timeout_ns_);  
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        interface_.c_str(), ex.what());
      return;
    }
  }
}

void SocketCanSenderNode::on_fd_frame(const ros2_socketcan_msgs::msg::FdFrame::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    FrameType type;
    if (msg->is_error) {
      type = FrameType::ERROR;
    } else {
      type = FrameType::DATA;
    }

    CanId send_id = msg->is_extended ? CanId(msg->id, 0, type, ExtendedFrame) :
      CanId(msg->id, 0, type, StandardFrame);
    try {
      sender_->send_fd(msg->data.data<void>(), msg->len, send_id, timeout_ns_);
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Error sending CAN message: %s - %s",
        interface_.c_str(), ex.what());
      return;
    }
  }
}

}  // namespace socketcan
}  // namespace drivers
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanSenderNode)
