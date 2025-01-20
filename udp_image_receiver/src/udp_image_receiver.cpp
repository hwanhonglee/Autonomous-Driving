#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <boost/asio.hpp>

class UDPImageReceiver : public rclcpp::Node {
public:
  UDPImageReceiver() : Node("udp_image_receiver") {
    // UDP 소켓 설정
    socket_ = std::make_shared<boost::asio::ip::udp::socket>(
        io_context_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 3000)); // 수신 포트 번호: 3000

    // ROS 2 토픽 설정
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("udp_image", 10);

    // UDP 데이터 수신 및 처리
    receiveData();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  boost::asio::io_context io_context_;
  std::shared_ptr<boost::asio::ip::udp::socket> socket_;

  void receiveData() {
    RCLCPP_INFO(this->get_logger(), "receiveData() function started.");

    while (rclcpp::ok()) {
      char buffer[65507];  // UDP 패킷 크기 제한
      boost::asio::ip::udp::endpoint sender_endpoint;
      size_t length = socket_->receive_from(boost::asio::buffer(buffer), sender_endpoint);
      
      // 수신된 데이터를 sensor_msgs::msg::Image 메시지로 변환하여 토픽으로 게시
      auto message = std::make_unique<sensor_msgs::msg::Image>();
      // TODO: 실제로 수신된 데이터를 이미지 메시지에 설정 (이미지 데이터 처리)
      // 예시: 수신된 데이터를 랜덤한 RGB 값으로 설정 (실제 데이터 처리 로직에 맞게 수정 필요)
      for (size_t i = 0; i < length; i += 3) {
        // 랜덤한 RGB 값 설정 (가상의 데이터 예시)
        buffer[i] = rand() % 256; // R 값
        buffer[i + 1] = rand() % 256; // G 값
        buffer[i + 2] = rand() % 256; // B 값
      }

      message->header.stamp = this->now();
      message->header.frame_id = "udp_image_frame";
      message->width = 640;
      message->height = 480;
      message->encoding = "rgb8";
      message->is_bigendian = false;
      message->step = 3 * message->width;
      message->data.resize(length);
      memcpy(message->data.data(), buffer, length);
      
      image_publisher_->publish(std::move(message));

      RCLCPP_INFO(this->get_logger(), "Received image data and published on udp_image topic.");
    }

    RCLCPP_INFO(this->get_logger(), "receiveData() function ended.");

  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UDPImageReceiver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}