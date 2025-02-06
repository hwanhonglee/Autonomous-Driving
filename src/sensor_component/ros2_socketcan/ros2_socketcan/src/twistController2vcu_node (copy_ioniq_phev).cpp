//HH_230322
#include <memory>
#include <string>

#include "std_msgs/msg/bool.hpp"
#include <std_msgs//msg/float64.hpp>
#include <std_msgs/msg/int32.hpp> 
#include <std_msgs/msg/int8.hpp>
// HH_250107 - autoware auto msg -> autoware msg
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

class TwistController2VcuNode : public rclcpp::Node
{
public:
  TwistController2VcuNode()
  : Node("twistController2Vcu_node")
  {
    // Subscribe to the current_cruise_mode_status topic
    sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "current_cruise_mode_status", 10, 
      std::bind(&TwistController2VcuNode::current_cruise_mode_callback, this, std::placeholders::_1));

    // Publish CAN data
    pub_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 500);
  }

private:
  // Subscriber
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  // Publisher
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr pub_;

  // Initialize member variables
  bool current_cruise_mode_ = false;
  unsigned int CAN_alive_count_ = 0;

  void current_cruise_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    current_cruise_mode_ = msg->data;
    // RCLCPP_INFO(this->get_logger(), "Current cruise mode status: %d", current_cruise_mode_); //HH_230914

    // Generate and publish CAN message
    can_msgs::msg::Frame frame_msg = generateFrame_IONIQ_EV_VCU();
    pub_->publish(frame_msg);
  }

  can_msgs::msg::Frame generateFrame_IONIQ_EV_VCU()
  {
    //RCLCPP_INFO(this->get_logger(), "Generating CAN message for IONIQ_CtrlVCU"); //HH_230914

    can_msgs::msg::Frame frame_msg;
    frame_msg.header.frame_id = "IONIQ_CtrlVCU";
    frame_msg.header.stamp = this->get_clock()->now();
    frame_msg.id = 342; // 0x156
    frame_msg.is_rtr = false;
    frame_msg.is_extended = false;
    frame_msg.is_error = false;
    frame_msg.dlc = 8;

    // set data bytes based on current_cruise_mode variable
    frame_msg.data[0] = (current_cruise_mode_ ? 1 : 0) & 0x01; // EPS_En
    frame_msg.data[1] = 0;
    frame_msg.data[2] = (current_cruise_mode_ ? 1 : 0) & 0x01; // ACC_En, AEB_En
    frame_msg.data[3] = 0;
    frame_msg.data[4] = 0;
    frame_msg.data[5] = 0; // turn_signal, hazard, Gear_cmd
    frame_msg.data[6] = 0;
    frame_msg.data[7] = CAN_alive_count_++; // alive-count

    return frame_msg;
  }
};
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistController2VcuNode>());
  rclcpp::shutdown();
  return 0;
}