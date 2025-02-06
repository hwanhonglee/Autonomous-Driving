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
: lc::LifecycleNode("socket_can_receiver_node", options),
  autoware_longitudinal_velocity_(0.0),
  autoware_lateral_velocity_(0.0),
  autoware_heading_rate_(0.0),
  wheel_speed_fr(0.0),
  longitudinal_velocity_received_(false),
  lateral_velocity_received_(false)
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
    auto filters = get_parameter("filters").as_string();
    receiver_->SetCanFilters(SocketCanReceiver::CanFilterList(filters));
    RCLCPP_INFO(get_logger(), "applied filters: %s", filters.c_str());
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(this->get_logger(), "Error opening CAN receiver: %s - %s", interface_.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(this->get_logger(), "Receiver successfully configured.");

  if (!enable_fd_) {
    frames_pub_ = this->create_publisher<can_msgs::msg::Frame>("from_can_bus", 500);
  } else {
    fd_frames_pub_ = this->create_publisher<ros2_socketcan_msgs::msg::FdFrame>("from_can_bus_fd", 500);
  }

  receiver_thread_ = std::make_unique<std::thread>(&SocketCanReceiverNode::receive, this);

  // Custom publishers
  // HH_250107 - autoware auto msg -> autoware msg
  current_velocity_topic_ = this->create_publisher<std_msgs::msg::Float64>("current/velocity_status", 500);
  current_brake_press_topic_ = this->create_publisher<std_msgs::msg::Float64>("current/brake_press_status", 500);
  current_steering_angle_topic_ = this->create_publisher<std_msgs::msg::Float64>("current_steering_angle_status", 500);
  current_cruise_mode_status_topic_ = this->create_publisher<std_msgs::msg::Bool>("current/cruise_mode_status", 500);
  current_cruise_mode_cancel_topic_= this->create_publisher<std_msgs::msg::Bool>("current/cruise_mode_cancel_status", 500);
  vehicle_status_steering_status_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status", 500);
  vehicle_status_velocity_status_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>("/vehicle/status/velocity_status", 500);
  vehicle_status_gear_status_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::GearReport>("/vehicle/status/gear_status", 500);
  vehicle_status_control_mode_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>("/vehicle/status/control_mode", 500);
  vehicle_status_hazard_lights_status_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>("/vehicle/status/hazard_lights_status", 500);
  vehicle_status_turn_indicators_status_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>("/vehicle/status/turn_indicators_status", 500);

  // HH_240814 // HH_250107 - autoware auto msg -> autoware msg
  vehicle_status_steering_status_rviz_topic_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>("/vehicle/status/steering_status_for_rviz", 500); 
  
  // TODO: accel, brake, steer 
  // actuation_status_topic_ = this->create_publisher<tier4_vehicle_msgs::msg::ActuationStatusStamped>("/vehicle/status/actuation_status", 500); 

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

  // Activate custom publishers
  current_velocity_topic_->on_activate();
  current_brake_press_topic_->on_activate();
  current_steering_angle_topic_->on_activate();
  current_cruise_mode_status_topic_->on_activate();
  current_cruise_mode_cancel_topic_->on_activate();
  vehicle_status_steering_status_topic_->on_activate();
  vehicle_status_steering_status_rviz_topic_->on_activate(); // HH_240814
  vehicle_status_velocity_status_topic_->on_activate();
  vehicle_status_gear_status_topic_->on_activate();
  vehicle_status_control_mode_topic_->on_activate();
  vehicle_status_hazard_lights_status_topic_->on_activate();
  vehicle_status_turn_indicators_status_topic_->on_activate();

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

  // Deactivate custom publishers
  current_velocity_topic_->on_deactivate();
  current_brake_press_topic_->on_deactivate();
  current_steering_angle_topic_->on_deactivate();
  current_cruise_mode_status_topic_->on_deactivate();
  current_cruise_mode_cancel_topic_->on_deactivate();
  vehicle_status_steering_status_topic_->on_deactivate();
  vehicle_status_steering_status_rviz_topic_->on_deactivate(); // HH_240814
  vehicle_status_velocity_status_topic_->on_deactivate();
  vehicle_status_gear_status_topic_->on_deactivate();
  vehicle_status_control_mode_topic_->on_deactivate();
  vehicle_status_hazard_lights_status_topic_->on_deactivate();
  vehicle_status_turn_indicators_status_topic_->on_deactivate();

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

  // Reset custom publishers
  current_velocity_topic_.reset();
  current_brake_press_topic_.reset();
  current_steering_angle_topic_.reset();
  current_cruise_mode_status_topic_.reset();
  current_cruise_mode_cancel_topic_.reset();
  vehicle_status_steering_status_topic_.reset();
  vehicle_status_steering_status_rviz_topic_.reset(); // HH_240814
  vehicle_status_velocity_status_topic_.reset();
  vehicle_status_gear_status_topic_.reset();
  vehicle_status_control_mode_topic_.reset();
  vehicle_status_hazard_lights_status_topic_.reset();
  vehicle_status_turn_indicators_status_topic_.reset();

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
        frame_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
      } else {
        frame_msg.header.stamp = this->now();
      }

      frame_msg.id = receive_id.identifier();
      frame_msg.is_rtr = (receive_id.frame_type() == FrameType::REMOTE);
      frame_msg.is_extended = receive_id.is_extended();
      frame_msg.is_error = (receive_id.frame_type() == FrameType::ERROR);
      frame_msg.dlc = receive_id.length();
      frames_pub_->publish(std::move(frame_msg));

      // Process the CAN message based on its ID
      switch(frame_msg.id)
      {
        // CAN 0
        // HH_250108
        case 881: //0x371 // at every 10 ms // brake_press, find by HH
        { 
          auto current_brake_press_report = frame_msg.data[0] / 1.6;
          if (current_brake_press_report >= 100) current_brake_press_report = 100;

          std_msgs::msg::Float64 current_brake_press_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_brake_press_msg.data = current_brake_press_report;
          current_brake_press_topic_->publish(std::move(current_brake_press_msg));
          break; 
        }

        // HH_250108
        case 882: //0x372 //for gear state
        {
          u_int8_t gear_state_raw_data = frame_msg.data[2];
          u_int8_t gear_state_data = 0; // autoware_vehicle_msgs number // 0x20

          if (gear_state_raw_data == 0x10) { gear_state_data = 22;} // P
          else if (gear_state_raw_data == 0x17) { gear_state_data = 20;} // R
          else if (gear_state_raw_data == 0x16) { gear_state_data = 1;} // N
          else if (gear_state_raw_data == 0x15) { gear_state_data = 2;} // D
          else if (gear_state_raw_data == 0x20) { gear_state_data = 3;} // D - eco, sports         

          // HH_250107 - autoware auto msg -> autoware msg
          autoware_vehicle_msgs::msg::GearReport vehicle_status_gear_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          vehicle_status_gear_status_msg.stamp = this->now();
          vehicle_status_gear_status_msg.report = gear_state_data;
          vehicle_status_gear_status_topic_->publish(std::move(vehicle_status_gear_status_msg)); 
          break;
        }

        // HH_250109
        // We don't use it because we can use the indicator light topic in the autoware universe. This topic is used by twistController.
        // case 1345: // 0x541, Turn_indicator, Hazard
        // {
        //     uint8_t data5 = frame_msg.data[5];  // [5] data
        //     uint8_t data2 = frame_msg.data[2];  // [2] data
        //     uint8_t data7 = frame_msg.data[7];  // [7] data

        //     // Hazard Lights Status
        //     int hazard_status = 0;  // Default (정지 상태)
        //     // Hazard
        //     if (data5 == 0x02 && data2 == 0x09 && data7 == 0x40) {
        //         hazard_status = 2;  // Emergency
        //     } 
        //     // Left Turn
        //     else if (data5 == 0x04 && data2 == 0x09 && data7 == 0x00) {
        //         hazard_status = 1;  // Driving
        //     }
        //     // Right Turn
        //     else if (data5 == 0x02 && data7 == 0x40 && data2 == 0x09) {
        //         hazard_status = 1;  // Driving
        //     }

        //     // Vehicle Status Update: Hazard Lights Status send
        //     // autoware_vehicle_msgs::msg::HazardLightsReport vehicle_status_hazard_lights_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
        //     // vehicle_status_hazard_lights_status_msg.stamp = this->now();
        //     // vehicle_status_hazard_lights_status_msg.report = gear_state_data;
        //     // vehicle_status_hazard_lights_status_topic_->publish(std::move(vehicle_status_hazard_lights_status_msg)); 
        //     // vehicle_status.hazard_lights_status = hazard_status;

        //     // // Vehicle Status Update: Turn Indicators Status send
        //     // int turn_indicator_status = 1;  // Default (Straight)

        //     // if (data5 == 0x04 && data2 == 0x09) {
        //     //     turn_indicator_status = 2;  // Left Turn
        //     // } else if (data5 == 0x02 && data7 == 0x40) {
        //     //     turn_indicator_status = 3;  // Right Turn
        //     // }

        //     // // 차량 상태 업데이트: Turn Indicators 상태 전송
        //     // vehicle_status.turn_indicators_status = turn_indicator_status;

        //     break;
        // }

        // HH_250108
        case 897: //0x381 // at every 10 ms // steering_angle
        { 
          short current_EPS_Angle_tmp = (frame_msg.data[4] << 8) | frame_msg.data[3];
          int current_steering_angle_report = current_EPS_Angle_tmp * 0.1;

          std_msgs::msg::Float64 current_steering_angle_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          current_steering_angle_msg.data = current_steering_angle_report;
          current_steering_angle_topic_->publish(std::move(current_steering_angle_msg)); // topic: /current_steering_angle_status

          // HH_250107 - autoware auto msg -> autoware msg
          double current_steering_tire_angle_status_report = current_steering_angle_report * DEG2RAD / 16.16667;
          autoware_vehicle_msgs::msg::SteeringReport vehicle_status_steering_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          vehicle_status_steering_status_msg.stamp = this->now();
          vehicle_status_steering_status_msg.steering_tire_angle = current_steering_tire_angle_status_report;
          vehicle_status_steering_status_topic_->publish(std::move(vehicle_status_steering_status_msg)); // topic: /vehicle/status/steering_status

          // HH_240814 // HH_250107 - autoware auto msg -> autoware msg 
          double current_steering_angle_status_report_for_rviz = current_steering_angle_report * DEG2RAD ;
          autoware_vehicle_msgs::msg::SteeringReport vehicle_status_steering_status_for_rivz_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          vehicle_status_steering_status_for_rivz_msg.stamp = this->now();
          vehicle_status_steering_status_for_rivz_msg.steering_tire_angle = current_steering_angle_status_report_for_rviz;
          vehicle_status_steering_status_rviz_topic_->publish(std::move(vehicle_status_steering_status_for_rivz_msg));

          break; 
        }

        // HH_250108
        case 902: // 0x386 // Vehicle Wheel Speed(FR), 
        {
          int16_t wheel_speed_fr_tmp = 0;
          // wheel_speed_fr_tmp = (frame_msg.data[3] << 8) | frame_msg.data[2]; // Use 16 bit  
          wheel_speed_fr_tmp = ((frame_msg.data[3] << 8) | frame_msg.data[2]) & 0x3FFF; // Use 14 bit  
          wheel_speed_fr = wheel_speed_fr_tmp * 0.03125; // mps
          autoware_longitudinal_velocity_ = wheel_speed_fr;
          longitudinal_velocity_received_ = true;

          break;
        }

        // HH_250108
        case 916: // 0x394 // Vehicle Longitudinal, Lateral Accel, Heading Rate, Brake Press 
        // !!!! 현재 DBC 파일의 아이디는 일치하지만, 내부 인덱스 등 매핑 요소의 불일치로 인해 값이 정확히 출력되지 않는 문제가 발생하고 있습니다. DBC 파일의 매핑 값을 검토하거나 올바른 DBC 파일을 적용해야 합니다.
        {
          int16_t lateral_acceleration_raw = (frame_msg.data[0] | (frame_msg.data[1] << 8)) & 0x07FF; // 11-bit 마스킹
          double lateral_acceleration = 0.01 * lateral_acceleration_raw - 10.23;

          // Using previous values for sudden changes
          static double previous_lateral_acceleration = 0.0;
          if (std::abs(lateral_acceleration - previous_lateral_acceleration) > 0.3) { // Rate Limit 0.3 m/s²
              lateral_acceleration = previous_lateral_acceleration;
          }
          previous_lateral_acceleration = lateral_acceleration;

          int16_t longitudinal_acceleration_raw = (frame_msg.data[4] | (frame_msg.data[5] << 8)) & 0x07FF; // 11-bit 마스킹
          double longitudinal_acceleration = 0.01 * longitudinal_acceleration_raw - 10.23;
          // Using previous values for sudden changes
          static double previous_longitudinal_acceleration = 0.0;
          if (std::abs(longitudinal_acceleration - previous_longitudinal_acceleration) > 0.3) { // Limit 0.3 m/s²
              longitudinal_acceleration = previous_longitudinal_acceleration;
          }
          previous_longitudinal_acceleration = longitudinal_acceleration;

          double lateral_velocity = 0.0;
          // double longitudinal_velocity = 0.0;
          double dt = 0.001; 

          // Lateral ACC to Lateral Velocity -> for autoware_msgs type 
          lateral_velocity += lateral_acceleration * dt; 
          if ( lateral_velocity <= 0.0 ) lateral_velocity = 0.0;
          autoware_lateral_velocity_ = lateral_velocity;
          lateral_velocity_received_ = true;

          // Longitudinal ACC to Longitudinal Velocity -> for autoware_msgs type
          // longitudinal_velocity += longitudinal_acceleration * dt;
          // if ( longitudinal_velocity <= 0.0 ) longitudinal_velocity = 0.0;
          // autoware_longitudinal_velocity_ = longitudinal_velocity;
          // longitudinal_velocity_received_ = true;

          // Heading Rate -> for autoware_msgs type
          // int16_t heading_rate_;
          // if (longitudinal_velocity > 0.01) heading_rate_ = lateral_acceleration / longitudinal_velocity;  // Prevent division when velocity is near zero
          // else { heading_rate_ = 0.0; }  // When velocity is nearly zero, turnover is also considered zero
          // autoware_heading_rate_ = heading_rate_;

          // RCLCPP_INFO(this->get_logger(), "Raw Lateral Acceleration Bits: %d", lateral_acceleration_raw);
          // RCLCPP_INFO(this->get_logger(), "Converted Lateral Acceleration: %.2f m/s²", lateral_acceleration);
          // RCLCPP_INFO(this->get_logger(), "Raw Longitudinal Acceleration Bits: %d", longitudinal_acceleration_raw);
          // RCLCPP_INFO(this->get_logger(), "Converted Longitudinal Acceleration: %.2f m/s²", longitudinal_acceleration);
        
          // Brake Press, find by openDBC, not use, yet 
          bool driver_braking = (frame_msg.data[6] & 0x80) >> 7; // Bitpos 55 -> byte[6] - 7 bit
          if (driver_braking) {
              // Brakes are working
              // std::cout << "Driver is braking." << std::endl;
              // additional logic...
          } else {
              // Brakes not working
              // std::cout << "Driver is not braking." << std::endl;
          }

          break;
        }

        // HH_250108
        case 1265: //0x4F1 // at every 10 ms // CRUISE & CANCEL Swtich 
        {           
          static bool cruise_mode_status = false; // Persistent state
          static bool cruise_mode_cancel = false; // Persistent state
          static bool acc_mode_status = false; // Persistent state
          static bool previous_cruise_pressed = false; // Track previous state for cruise button
          static bool previous_cancel_pressed = false; // Track previous state for cancel button
          static bool previous_acc_pressed = false; // Track previous state for cancel button
          uint8_t autoware_vehicle_status_mode = 4; // HH_250206


          int16_t cruise_status_raw = frame_msg.data[0]; // Extract [0]: 3-bit
          // uint8_t cruise_main_value = frame_msg.data[0] & 0x08; // 3번째 비트를 추출 // inpu cruise button -> return 0x08 

          // 디버깅 출력
          // RCLCPP_INFO(this->get_logger(), "Cruise Main Value: %d", cruise_main_value);
          // RCLCPP_INFO(this->get_logger(), "Cruise status raw: 0x%x", cruise_status_raw);

          // Cruise button logic (toggle)
          bool current_cruise_pressed = (cruise_status_raw == 0x28 || cruise_status_raw == 0x08);
          if (current_cruise_pressed && !previous_cruise_pressed) { cruise_mode_status = !cruise_mode_status;} // Toggle state
          previous_cruise_pressed = current_cruise_pressed; // Update previous state          
          
          // Cruise_Status Topic Publish
          std_msgs::msg::Bool cruise_mode_status_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          cruise_mode_status_msg.data = cruise_mode_status;
          current_cruise_mode_status_topic_->publish(std::move(cruise_mode_status_msg));

          // Cancel button logic (toggle)
          bool current_cancel_pressed = (cruise_status_raw == 0x24 || cruise_status_raw == 0x04);
          if (current_cancel_pressed && !previous_cancel_pressed) {cruise_mode_cancel = !cruise_mode_cancel;} // Toggle state
          previous_cancel_pressed = current_cancel_pressed; // Update previous state

          // Cancel_Status Topic Publish
          std_msgs::msg::Bool cruise_mode_cancel_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          cruise_mode_cancel_msg.data = cruise_mode_cancel;
          current_cruise_mode_cancel_topic_->publish(std::move(cruise_mode_cancel_msg));

          // HH_250206
          // Autoware Control Mode Status
          if (cruise_mode_status == 1) {autoware_vehicle_status_mode = 1;}
          else if (cruise_mode_status == 0) {autoware_vehicle_status_mode = 4;}
          else {autoware_vehicle_status_mode = 0;}
          autoware_vehicle_msgs::msg::ControlModeReport autoware_vehicle_status_control_mode_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
          autoware_vehicle_status_control_mode_msg.stamp = this -> now();
          autoware_vehicle_status_control_mode_msg.mode = autoware_vehicle_status_mode; 
          vehicle_status_control_mode_topic_->publish(std::move(autoware_vehicle_status_control_mode_msg));


          // ACC button logic (toggle)
          bool current_acc_pressed = (cruise_status_raw == 0x23 || cruise_status_raw == 0x03);
          if (current_acc_pressed && !previous_acc_pressed) {
              acc_mode_status = !acc_mode_status; // Toggle state
          }
          previous_cancel_pressed = current_cancel_pressed; // Update previous state

          // ACC_Status Topic Publish (TODO)..
    
          break; 
        }

        // CAN 1
        // {};
      }

      if (lateral_velocity_received_ && longitudinal_velocity_received_) {
        publish_combined_message();
        lateral_velocity_received_ = false;
        longitudinal_velocity_received_ = false;
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
        fd_frame_msg.header.stamp = rclcpp::Time(static_cast<int64_t>(receive_id.get_bus_time() * 1000U));
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

// HH_240722 for convert autoware_msgs topic  // HH_250107 - autoware auto msg -> autoware msg
void SocketCanReceiverNode::publish_combined_message()
{
  autoware_vehicle_msgs::msg::VelocityReport combined_msg;
  combined_msg.header.stamp = this->now();
  combined_msg.header.frame_id = "base_link";
  combined_msg.longitudinal_velocity = autoware_longitudinal_velocity_ / 3.6; // Convert to m/s
  combined_msg.lateral_velocity = autoware_lateral_velocity_;
  combined_msg.heading_rate = autoware_heading_rate_;

  vehicle_status_velocity_status_topic_->publish(combined_msg);
}

}  // namespace socketcan
}  // namespace drivers

RCLCPP_COMPONENTS_REGISTER_NODE(drivers::socketcan::SocketCanReceiverNode)
