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

class Relay : public rclcpp::Node
{
  public:
    Relay()
    : Node("Relay")
    {
      //creates publisher
      publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
      timer_ = this->create_wall_timer(
      50ms, std::bind(&Relay::timer_callback, this));

      subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10,
      std::bind(&Relay::topic_callback,this,std::placeholders::_1));

    }

  private:

    float d;
    float v;

    void timer_callback()
    {
      auto message = ackermann_msgs::msg::AckermannDriveStamped();
      float d_new = d*3;
      float v_new = v*3;
      RCLCPP_INFO(this->get_logger(), "d_new: '%f'", d_new);
      RCLCPP_INFO(this->get_logger(), "v_new: '%f'", v_new);

      message.drive.steering_angle = d_new;
      message.drive.speed = v_new;

      publisher_->publish(message);
    }

    void topic_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
    {
       d = msg->drive.steering_angle;
       v = msg->drive.speed;
       RCLCPP_INFO(this->get_logger(), "I heard d: '%f'", d);
       RCLCPP_INFO(this->get_logger(), "I heard v: '%f'", v);
      
    }


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Relay>());
  rclcpp::shutdown();
  return 0;
}
