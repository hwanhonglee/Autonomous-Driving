// HH_230322 //all create 
#include <memory>
#include <string>

#include "std_msgs/msg/bool.hpp"
#include <std_msgs//msg/float64.hpp>
#include <std_msgs/msg/int32.hpp> 
#include <std_msgs/msg/int8.hpp>

// HH_230323 // HH_250107 - autoware auto msg -> autoware msg, lateral & longitudinal merge to Control 
#include <autoware_control_msgs/msg/control.hpp>
// HH_230328
#include <tier4_vehicle_msgs/msg/actuation_command_stamped.hpp>

#include "ros2_socketcan/visibility_control.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "lifecycle_msgs/msg/state.hpp"

// HH_230323
#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535
#define KPH2MPS 0.27778
#define MPS2KPH 3.6

class twistController2Vcu2Eps2AccNode : public rclcpp::Node
{
public:
  twistController2Vcu2Eps2AccNode()
  : Node("twistController2Eps2Acc_node")
  {
    // Subscribe to the current_cruise_mode_status topic
    crise_mode_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "current/cruise_mode_status", 10, 
      std::bind(&twistController2Vcu2Eps2AccNode::cruise_mode_status_callback, this, std::placeholders::_1));

    // HH_230328 // HH_250107 - autoware auto msg -> autoware msg, lateral & longitudinal merge to Control 
    control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
      "/control/command/control_cmd", 1, 
      std::bind(&twistController2Vcu2Eps2AccNode::control_cmd_callback, this, std::placeholders::_1));

    // Publish CAN data
    pub_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 1);

    // HH_240725
    // Create a timer to publish the throttle command at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30), // ≈33hz 
      std::bind(&twistController2Vcu2Eps2AccNode::ControlCmdPublish2Controller, this));
  }

private:
  // HH_250107 - autoware auto msg -> autoware msg, lateral & longitudinal merge to Control 
  // Subscriber
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr crise_mode_status_sub_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;

  // HH_240725
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialize member variables
  double throttle_cmd_ = -1.0;
  double steering_cmd_ = 0.0;
  bool cruise_mode_status_ = false;
  unsigned int CAN_alive_count_ = 0;

  void cruise_mode_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    cruise_mode_status_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Current cruise mode status: %d", cruise_mode_status_); //HH_230914

    // Generate and publish CAN message
    // can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_CtrlVCU();
    // pub_->publish(frame_msg);
  }

  // HH_230531 for actuation cmd _ to rad // HH_250107 - autoware auto msg -> autoware msg, lateral & longitudinal merge to Control 
  void control_cmd_callback(const autoware_control_msgs::msg::Control::SharedPtr msg)
  {
    // Longitudinal Control 
    throttle_cmd_ = msg->longitudinal.acceleration;

    // HH_230907 for edit limit 
    if (throttle_cmd_ >= 1.5) throttle_cmd_ = 1.5;
    // if (throttle_cmd_ >= 0) throttle_cmd_ = 0.5;
    else if (throttle_cmd_ == 0.0) throttle_cmd_ = -0.01;
    else if (throttle_cmd_ <= -5.0) throttle_cmd_ = -5.0;
  
    RCLCPP_INFO(this->get_logger(), "throttle_cmd_: %f", throttle_cmd_);

    // Lateral Control
    // steering_cmd_ =  std::clamp(msg->lateral.steering_tire_angle * RAD2DEG * (20 / (msg->lateral.steering_tire_angle * RAD2DEG / 1.5)), -438.0, 438.0); // rad2deg * max_steering_wheel_angle / max_wheel_angle
    // steering_cmd_ =  std::clamp(msg->lateral.steering_tire_angle * RAD2DEG * 1.4 , -438.0, 438.0);

    // HH_230615
    steering_cmd_ =  msg->lateral.steering_tire_angle * RAD2DEG * 16.16667; // tire rad to steer deg // 

    if (steering_cmd_ > 438.0) 
    {
      steering_cmd_ = 438.0;
    }
    else if (steering_cmd_ < -438.0) 
    {
      steering_cmd_ = -438.0; 
    }
    else {};
    
    RCLCPP_INFO(this->get_logger(), "steering_cmd_: %f", steering_cmd_);

    // // Generate and publish CAN message
    // can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_EV_VCU2EPS2ACC();
    // pub_->publish(frame_msg);
  }

  // HH_240725
  void ControlCmdPublish2Controller()
  {
    // Generate and publish CAN message
    can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_EV_VCU2EPS2ACC();
    pub_->publish(frame_msg);
  }

  can_msgs::msg::Frame generateFrame_IONIQ_EV_VCU2EPS2ACC() //EPS_REQ ACC_REQ
  {
    // RCLCPP_INFO(this->get_logger(), "Generating CAN message for IONIQ_EV");

    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

    frame_msg.header.frame_id = "IONIQ_EV";
    frame_msg.header.stamp = rclcpp::Clock().now();
    frame_msg.id = 1584; //0x630
    frame_msg.is_rtr = false;
    frame_msg.is_extended = false;
    frame_msg.is_error = false;
    frame_msg.dlc = 8;

    // Populate CAN message data
    // Acceleration command (0~11 bits)
    int accel_cmd = static_cast<int>(std::round((throttle_cmd_ + 10.23) / 0.01)); // 스케일링 및 오프셋 보정
    accel_cmd = std::clamp(accel_cmd, 0, 2047); // 11비트 값 범위에 맞게 클램핑

    // LSB와 MSB 추출
    frame_msg.data[0] = accel_cmd & 0xFF;       // 하위 8비트
    frame_msg.data[1] = (accel_cmd >> 8) & 0x07; // 상위 3비트 (11비트 사용)
    
    // frame_msg.data[0] = (static_cast<int>(100.0*((throttle_cmd_) + 10.23)) & 0x00FF); // LSB
    // frame_msg.data[1] = (static_cast<int>(100.0*((throttle_cmd_) + 10.23)) >> 8 & 0x07); // MSB (11 bits total)

    // CtrlModeReq (Control mode request at bit 24~31)
    frame_msg.data[3] = cruise_mode_status_; // Set control mode to 1 (SWA CMD / Accel CMD)

    // Steering wheel angle command (32~47 bits)
    frame_msg.data[4] = (static_cast<int>(10.0*steering_cmd_) & 0x00FF); // LSB 
    frame_msg.data[5] = (static_cast<int>(10.0*steering_cmd_) >> 8 & 0xFFFF); // MSB
    
    // Target speed (48~55 bits) - not used in this mode
    frame_msg.data[6] = 0; // Set to 0 for now

    // Alive Counter (56~63 bits)
    frame_msg.data[7] = CAN_alive_count_++ & 0xFF; // Alive Counter

    return frame_msg;
  }

};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<twistController2Vcu2Eps2AccNode>());
  rclcpp::shutdown();
  return 0;
}