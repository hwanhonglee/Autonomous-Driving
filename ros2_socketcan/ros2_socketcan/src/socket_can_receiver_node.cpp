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

#include "ros2_socketcan/socket_can_receiver_node.hpp"
#include "ros2_socketcan/socket_can_common.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

//HH_230314
#define RAD2DEG 57.295779513
#define DEG2RAD 0.0174533 

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;
using namespace std::chrono_literals;

namespace drivers
{
namespace socketcan
{
SocketCanReceiverNode::SocketCanReceiverNode(rclcpp::NodeOptions options)
: lc::LifecycleNode("socket_can_receiver_node", options)
{
  interface_ = this->declare_parameter("interface", "can0");
  use_bus_time_ = this->declare_parameter<bool>("use_bus_time", false);
  enable_fd_ = this->declare_parameter<bool>("enable_can_fd", false);
  double interval_sec = this->declare_parameter("interval_sec", 0.01);
  this->declare_parameter("filters", "0:0");
  interval_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(interval_sec));

  RCLCPP_INFO(this->get_logger(), "interface: %s", interface_.c_str());
  RCLCPP_INFO(this->get_logger(), "use bus time: %d", use_bus_time_);
  RCLCPP_INFO(this->get_logger(), "can fd enabled: %s", enable_fd_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "interval(s): %f", interval_sec);
}

LNI::CallbackReturn SocketCanReceiverNode::on_configure(const lc::State & state)
{
  (void)state;

  try {
    receiver_ = std::make_unique<SocketCanReceiver>(interface_, enable_fd_);
    // apply CAN filters
    auto filters = get_parameter("filters").as_string();
    receiver_->SetCanFilters(SocketCanReceiver::CanFilterList(filters));
    RCLCPP_INFO(get_logger(), "applied filters: %s", filters.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      this->get_logger(), "Error opening CAN receiver: %s - %s",
      interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Receiver successfully configured.");

  if (!enable_fd_) {
    frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 500);
  } else {
    fd_frames_pub_ =
      this->create_publisher<ros2_socketcan_msgs::msg::FdFrame>("from_can_bus_fd", 500);
  }

  receiver_thread_ = std::make_unique<std::thread>(&SocketCanReceiverNode::receive, this);

  //HH_230308
  current_velocity_topic_ = this->create_publisher<std_msgs::msg::Float64>("current_velocity_status", 500);
  current_brake_press_topic_ = this->create_publisher<std_msgs::msg::Float64>("current_brake_press_status", 500);
  current_steering_angle_topic_ = this->create_publisher<std_msgs::msg::Float64>("current_steering_angle_status", 500);
  current_EPS_En_topic_ = this->create_publisher<std_msgs::msg::Bool>("current_EPS_En_status", 500);
  current_ACC_En_topic_ = this->create_publisher<std_msgs::msg::Bool>("current_ACC_En_status", 500);
  current_cruise_mode_topic_ = this->create_publisher<std_msgs::msg::Bool>("current_cruise_mode_status", 500);

  //HH_230309
  current_steering_angle_status_topic_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 500);
  //HH_230314
  current_velocity_status_topic_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 500);
  //HH_230401
  current_steering_angle_status_topic_for_rviz_ = this->create_publisher<autoware_auto_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status_for_rviz", 500);

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanReceiverNode::on_activate(const lc::State & state)
{
  (void)state;

  if (!enable_fd_) {
    frames_pub_->on_activate();
  } else {
    fd_frames_pub_->on_activate();
  }

  //HH_230308
  current_velocity_topic_->on_activate();
  current_brake_press_topic_->on_activate();
  current_steering_angle_topic_->on_activate();
  current_EPS_En_topic_->on_activate();
  current_ACC_En_topic_->on_activate();
  current_cruise_mode_topic_->on_activate();
  //HH_230309
  current_steering_angle_status_topic_->on_activate();
  //HH-230314
  current_velocity_status_topic_->on_activate();
  //HH_230401
  current_steering_angle_status_topic_for_rviz_->on_activate();

  RCLCPP_DEBUG(this->get_logger(), "Receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanReceiverNode::on_deactivate(const lc::State & state)
{
  (void)state;

  if (!enable_fd_) {
    frames_pub_->on_deactivate();
  } else {
    fd_frames_pub_->on_deactivate();
  }

  //HH_230308
  current_velocity_topic_->on_deactivate();
  current_brake_press_topic_->on_deactivate();
  current_steering_angle_topic_->on_deactivate();
  current_EPS_En_topic_->on_deactivate();
  current_ACC_En_topic_->on_deactivate();
  current_cruise_mode_topic_->on_deactivate();
  //HH_230309
  current_steering_angle_status_topic_->on_deactivate();
  //HH-230314
  current_velocity_status_topic_->on_deactivate();
  //HH_230401
  current_steering_angle_status_topic_for_rviz_->on_deactivate();

  RCLCPP_DEBUG(this->get_logger(), "Receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanReceiverNode::on_cleanup(const lc::State & state)
{
  (void)state;

  if (!enable_fd_) {
    frames_pub_.reset();
  } else {
    fd_frames_pub_.reset();
  }

  if (receiver_thread_->joinable()) {
    receiver_thread_->join();
  }

  //HH_230308
  current_velocity_topic_.reset();
  current_brake_press_topic_.reset();
  current_steering_angle_topic_.reset();
  current_EPS_En_topic_.reset();
  current_ACC_En_topic_.reset();
  current_cruise_mode_topic_.reset();
  //HH_230309
  current_steering_angle_status_topic_.reset();
  //HH-230314
  current_velocity_status_topic_.reset();
  //HH_230401
  current_steering_angle_status_topic_for_rviz_.reset();

  RCLCPP_DEBUG(this->get_logger(), "Receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SocketCanReceiverNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(this->get_logger(), "Receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void SocketCanReceiverNode::receive()
{
  CanId receive_id{};

  if (!enable_fd_) {
    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    frame_msg.header.frame_id = "can";

    while (rclcpp::ok()) {
      if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      try {
        receive_id = receiver_->receive(frame_msg.data.data(), interval_ns_);
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Error receiving CAN message: %s - %s",
          interface_.c_str(), ex.what());
        continue;
      }

      if (use_bus_time_) {
        frame_msg.header.stamp =
          rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
      } else {
        frame_msg.header.stamp = this->now();
      }

      frame_msg.id = receive_id.identifier();
      frame_msg.is_rtr = (receive_id.frame_type() == FrameType::REMOTE);
      frame_msg.is_extended = receive_id.is_extended();
      frame_msg.is_error = (receive_id.frame_type() == FrameType::ERROR);
      frame_msg.dlc = receive_id.length();
      frames_pub_->publish(std::move(frame_msg));

      //HH_230308 전체 재 수정
      switch(frame_msg.id)
      {
        // CAN 0 //0x162, 0x240, 0x386, 0x387
        case 544: //0x220 // at every 10 ms //YAW_RATE
        { 
          break; 
        }
        
        case 881: //0x371 // at every 10 ms //brake_press
        { 
          auto curretn_brake_press_report = frame_msg.data[0]/1.6;     //HH_221108 // full brake -> decimal :160, divide 1.6
          
          std_msgs::msg::Float64 curretn_brake_press_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          if (curretn_brake_press_report >= 100) curretn_brake_press_report = 100;     //HH_221108
          curretn_brake_press_msg.data = curretn_brake_press_report;
          current_brake_press_topic_->publish(std::move(curretn_brake_press_msg));
          
          break; 
        }

        // CAN 1
        case 1808: //0x710 // at every 10 ms //steering_angle
        { 
          int current_steering_angle_report = 0;
          bool current_EPS_En_              = 0;        // EPS_En_status Manual/Auto 모드 변환 요청 피드백
          //bool EPS_Fd_Override_           = 0;        // EPS override check
          short current_EPS_Angle_tmp       = 0; //HH_230313
          //HH_230309
          double current_steering_angle_status_report = 0;

          //HH_230321
          // current_EPS_En_ = (frame_msg.data[0] & 0X01); // EPS_En_status //
          // *****In ROS2, an error occurs when receiving CAN data because the data is received starting from index [1] and [0] is not received.
          current_EPS_En_ = (frame_msg.data[0] & 0X01); // EPS_En_status // 
          std::cout << "EPS_En_: " << current_EPS_En_ << std::endl; // cruise on -> 1  //HH_221208

          //EPS_Fd_Override_ = (frame_msg.data[1] & 0x32);

          current_EPS_Angle_tmp = frame_msg.data[3]& 0xff;
          current_EPS_Angle_tmp = current_EPS_Angle_tmp << 8;
          current_EPS_Angle_tmp = frame_msg.data[2]| current_EPS_Angle_tmp;

          current_steering_angle_report = current_EPS_Angle_tmp * 0.1;
          //std::cout<<std::dec<< "핸들 각도 : "<< 0.1*steering_angle_report <<std::endl; // -483 ~ + 483 (+-485 까지 가능하나, 너무 부담됨)
          // for rviz
          std_msgs::msg::Float64 current_steering_angle_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_steering_angle_msg.data = current_steering_angle_report;
          current_steering_angle_topic_->publish(std::move(current_steering_angle_msg));

          //HH_230531 for raw_vehicle_converter cmd
          current_steering_angle_status_report = current_steering_angle_report * DEG2RAD; //HH_230531 
          autoware_auto_vehicle_msgs::msg::SteeringReport current_steering_angle_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_steering_angle_status_msg.steering_tire_angle = (current_steering_angle_status_report); //HH_230314 // rad
          current_steering_angle_status_topic_->publish(std::move(current_steering_angle_status_msg));

          // //HH_230309
          // current_steering_angle_status_report = current_steering_angle_report;
          // autoware_auto_vehicle_msgs::msg::SteeringReport current_steering_angle_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          // current_steering_angle_status_msg.steering_tire_angle = (current_steering_angle_status_report); //HH_230314 // degree
          // current_steering_angle_status_topic_->publish(std::move(current_steering_angle_status_msg));

          //HH_230401
          autoware_auto_vehicle_msgs::msg::SteeringReport current_steering_angle_status_for_rviz_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_steering_angle_status_for_rviz_msg.steering_tire_angle = (current_steering_angle_report / RAD2DEG); //HH_230314 // degree
          current_steering_angle_status_topic_for_rviz_->publish(std::move(current_steering_angle_status_for_rviz_msg));

          std_msgs::msg::Bool current_EPS_En_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_EPS_En_msg.data = current_EPS_En_;
          current_EPS_En_topic_->publish(std::move(current_EPS_En_msg));

          //HH_230201
          bool current_cruise_status = (frame_msg.data[1] & 0x8A) >> 7;// 1000 0000,
          //bool cancel_ = (frame_msg.data[1] & 0x40) >> 6; // 0100 0000, 
          bool current_cruise_mode = false;
          
          if ((current_cruise_status == 1) && (current_cruise_mode == false))
          {
            current_cruise_mode = true;              

            std_msgs::msg::Bool current_cruise_mode_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
            current_cruise_mode_msg.data = current_cruise_mode;
            current_cruise_mode_topic_->publish(std::move(current_cruise_mode_msg));
          }
          else if ((current_cruise_status == 0))
          { 
            current_cruise_mode = false;
            
            std_msgs::msg::Bool current_cruise_mode_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
            current_cruise_mode_msg.data = current_cruise_mode;
            current_cruise_mode_topic_->publish(std::move(current_cruise_mode_msg));    
          }
          
          //std::cout << "cruise_: " << cruise_ << std::endl; 
          //std::cout << "cancel_: " << cancel_ << std::endl; 
          break; 
        }

        case 1809: //0x711 // at every 10 ms //current_vehicle_velocity
        { 
          //HH_221108
          double current_velocity_report = frame_msg.data[2] & 0xff;  // KPH | if mps --> * 0.1; // * KPH2MPS; 
          //HH_221208
          bool current_ACC_En_ = frame_msg.data[0] & 0x01;
          //std::cout << "ACC_En: " << ACC_En_ << std::endl;
          //HH_230314
          double current_velocity_status_report = 0.0;
          
          //HH_221108
          std_msgs::msg::Float64 current_velocity_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_velocity_msg.data = current_velocity_report;
          current_velocity_topic_->publish(std::move(current_velocity_msg));
          // std::cout << "current_velocity_report: " << current_velocity_report << std::endl; 
          //HH_221208
          std_msgs::msg::Bool current_ACC_En_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_ACC_En_msg.data = current_ACC_En_;
          current_ACC_En_topic_->publish(std::move(current_ACC_En_msg));
                    
          //HH_230314
          current_velocity_status_report = current_velocity_report;
          autoware_auto_vehicle_msgs::msg::VelocityReport current_velocity_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_velocity_status_msg.longitudinal_velocity = (current_velocity_status_report / 3.6);
          current_velocity_status_msg.header.frame_id = "base_link";  //HH_230316
          current_velocity_status_topic_->publish(std::move(current_velocity_status_msg));
          // std::cout << "current_velocity_status_report: " << current_velocity_status_report << std::endl;
          break; 
        }
      }
      }
  } else {
    ros2_socketcan_msgs::msg::FdFrame fd_frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    fd_frame_msg.header.frame_id = "can";

    while (rclcpp::ok()) {
      if (this->get_current_state().id() != State::PRIMARY_STATE_ACTIVE) {
        std::this_thread::sleep_for(100ms);
        continue;
      }

      fd_frame_msg.data.resize(64);

      try {
        receive_id = receiver_->receive_fd(fd_frame_msg.data.data<void>(), interval_ns_);
      } catch (const std::exception & ex) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 1000,
          "Error receiving CAN FD message: %s - %s",
          interface_.c_str(), ex.what());
        continue;
      }

      fd_frame_msg.data.resize(receive_id.length());

      if (use_bus_time_) {
        fd_frame_msg.header.stamp =
          rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
      } else {
        fd_frame_msg.header.stamp = this->now();
      }

      fd_frame_msg.id = receive_id.identifier();
      fd_frame_msg.is_extended = receive_id.is_extended();
      fd_frame_msg.is_error = (receive_id.frame_type() == FrameType::ERROR);
      fd_frame_msg.len = receive_id.length();
      fd_frames_pub_->publish(std::move(fd_frame_msg));
    }
  }
}

}  // namespace socketcan
}  // namespace drivers

RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanReceiverNode)
