
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node
{
public:
    VelocityPublisher() : Node("velocity_publisher")
    {
        // Declare configurable ROS 2 parameters
        this->declare_parameter<double>("linear_speed", 0.2);
        this->declare_parameter<double>("angular_speed", 0.0);

        // Publish velocity commands
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10);

        // Publish at 10 Hz
        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&VelocityPublisher::publish_velocity, this));
    }

private:
    void publish_velocity()
    {
        geometry_msgs::msg::Twist msg;

        msg.linear.x =
            this->get_parameter("linear_speed").as_double();

        msg.angular.z =
            this->get_parameter("angular_speed").as_double();

        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "linear=%.2f m/s, angular=%.2f rad/s",
            msg.linear.x,
            msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityPublisher>());
    rclcpp::shutdown();
    return 0;
}
