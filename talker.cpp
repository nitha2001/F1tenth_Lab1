#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
  public:
    Publisher()
    : Node("Publisher")

    {
      this->declare_parameter("steering_angle",50.0);
      this->declare_parameter("speed",7.0);

      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

      timer_ = this->create_wall_timer(
      50ms, std::bind(&Publisher::timer_callback, this));
    }

  private:
    

    void timer_callback()
    {
      

      rclcpp::Parameter speed_param = this->get_parameter("speed");
      rclcpp::Parameter angle_param = this->get_parameter("steering_angle");

      float speed_f = speed_param.as_double();
      float angle_f = angle_param.as_double();

      auto message = ackermann_msgs::msg::AckermannDriveStamped();
      message.drive.steering_angle = angle_f;
      message.drive.speed = speed_f;
      
      RCLCPP_INFO(this->get_logger(), "d: '%f'", angle_f);
      RCLCPP_INFO(this->get_logger(), "v: '%f'", speed_f);
      
      publisher_->publish(message);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
