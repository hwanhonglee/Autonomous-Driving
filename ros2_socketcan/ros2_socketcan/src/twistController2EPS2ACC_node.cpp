// HH_230322 //all create 
#include <memory>
#include <string>

#include "std_msgs/msg/bool.hpp"
#include <std_msgs//msg/float64.hpp>
#include <std_msgs/msg/int32.hpp> 
#include <std_msgs/msg/int8.hpp>

//HH_230323
#include <autoware_auto_control_msgs/msg/ackermann_control_command.hpp>
#include <autoware_auto_control_msgs/msg/ackermann_lateral_command.hpp>
#include <autoware_auto_control_msgs/msg/longitudinal_command.hpp>
//HH_230328
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

//HH_230323
#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535
#define KPH2MPS 0.27778
#define MPS2KPH 3.6

class TwistController2Eps2AccNode : public rclcpp::Node
{
public:
  TwistController2Eps2AccNode()
  : Node("twistController2Eps2Acc_node")
  {
    //HH_230328
    // Subscribe to the current_cruise_mode_status topic
    steering_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1, 
      std::bind(&TwistController2Eps2AccNode::steering_callback, this, std::placeholders::_1));
    
    throttle_sub_ = this->create_subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>(
      "/control/command/control_cmd", 1, 
      std::bind(&TwistController2Eps2AccNode::throttle_callback, this, std::placeholders::_1));

    // Publish CAN data
    pub_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 1);

    // HH_240725
    // Create a timer to publish the throttle command at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30), // ≈33hz 
      std::bind(&TwistController2Eps2AccNode::publish_cmd, this));
  }

private:
  //HH_230328
  // Subscriber
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr steering_sub_;
  rclcpp::Subscription<autoware_auto_control_msgs::msg::AckermannControlCommand>::SharedPtr throttle_sub_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;

  // HH_240725
  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Initialize member variables
  double throttle_cmd = 0.0;
  double steering_cmd = 0.0;

  //HH_230328
  void throttle_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    throttle_cmd = msg->longitudinal.acceleration;

    // HH_230907 for edit limit 
    if (throttle_cmd >= 5.0) throttle_cmd = 5.0;
    // if (throttle_cmd >= 0) throttle_cmd = 0.5;
    else if (throttle_cmd == 0.0) throttle_cmd = -0.01;
    else if (throttle_cmd <= -3.0) throttle_cmd = -3.0;
  
    RCLCPP_INFO(this->get_logger(), "Current throttle_cmd: %f", throttle_cmd);

    // // Generate and publish CAN message
    // can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_ADS2EPS_ACC();
    // pub_->publish(frame_msg);
  }

  //HH_230531 for actuation cmd _ to rad
  void steering_callback(const autoware_auto_control_msgs::msg::AckermannControlCommand::SharedPtr msg)
  {
    // steering_cmd =  std::clamp(msg->lateral.steering_tire_angle * RAD2DEG * (20 / (msg->lateral.steering_tire_angle * RAD2DEG / 1.5)), -438.0, 438.0); // rad2deg * max_steering_wheel_angle / max_wheel_angle
    // steering_cmd =  std::clamp(msg->lateral.steering_tire_angle * RAD2DEG * 1.4 , -438.0, 438.0);

    // HH_230615
    steering_cmd =  msg->lateral.steering_tire_angle * RAD2DEG * 16.16667; // tire rad to steer deg // 

    if (steering_cmd > 438.0) 
    {
      steering_cmd = 438.0;
    }
    else if (steering_cmd < -438.0) 
    {
      steering_cmd = -438.0; 
    }
    else 
    {};
    
    RCLCPP_INFO(this->get_logger(), "Current steering_cmd: %f", steering_cmd);

    // // Generate and publish CAN message
    // can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_ADS2EPS_ACC();
    // pub_->publish(frame_msg);
  }

  //HH_240725
  void publish_cmd()
  {
    // Generate and publish CAN message
    can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_ADS2EPS_ACC();
    pub_->publish(frame_msg);
  }

  can_msgs::msg::Frame generateFrame_IONIQ_ADS2EPS_ACC() //EPS_REQ ACC_REQ
  {
    RCLCPP_INFO(this->get_logger(), "Generating CAN message for IONIQ_ADS2EPS_ACC");

    can_msgs::msg::Frame frame_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

    frame_msg.header.frame_id = "IONIQ_ADS2EPS_ACC";
    frame_msg.header.stamp = rclcpp::Clock().now();
    frame_msg.id = 343; //0x157
    frame_msg.is_rtr = false;
    frame_msg.is_extended = false;
    frame_msg.is_error = false;
    frame_msg.dlc = 8;

    //cruise_mode on
    frame_msg.data[0] = (static_cast<int>(10.0*steering_cmd) & 0x00FF); //steering_cmd
    frame_msg.data[1] = (static_cast<int>(10.0*steering_cmd) >> 8 & 0xFFFF); //steering_cmd
    frame_msg.data[2] = 0;
    frame_msg.data[3] = (static_cast<int>(100.0*((throttle_cmd) + 10.23)) & 0x00FF); //throttle_cmd
    frame_msg.data[4] = (static_cast<int>(100.0*((throttle_cmd) + 10.23)) >> 8 & 0xFFFF); //throttle_cmd))
    // throttle_cmd  Acceleration Control    100*((value)+10.23) [-5.0, 5.0]    멈추려면 -값 주면 된다고 함.
    frame_msg.data[5] = 0;
    frame_msg.data[6] = 0;
    frame_msg.data[7] = 0;

    return frame_msg;
  }

};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistController2Eps2AccNode>());
  rclcpp::shutdown();
  return 0;
}