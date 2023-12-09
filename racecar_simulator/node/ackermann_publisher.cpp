#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class AckermannPublisher : public rclcpp::Node
{
public:
  AckermannPublisher() : Node("ackermann_publisher")
  {
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&AckermannPublisher::publish_speed, this));
  }

private:
  void publish_speed()
  {
    auto ack_msg = std::make_unique<ackermann_msgs::msg::AckermannDriveStamped>();
    ack_msg->drive.speed = 1.0;
    publisher_->publish(std::move(ack_msg));
    RCLCPP_INFO(this->get_logger(), "Published speed: 1.0");
  }

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AckermannPublisher>());
  rclcpp::shutdown();
  return 0;
}