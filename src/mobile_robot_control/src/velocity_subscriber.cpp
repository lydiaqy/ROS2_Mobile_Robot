
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class VelocitySubscriber : public rclcpp::Node
{
public:
    VelocitySubscriber() : Node("velocity_subscriber")
    {
        subscription_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &VelocitySubscriber::velocity_callback,
                    this,
                    std::placeholders::_1));
    }

private:
    void velocity_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received velocity: linear=%.2f m/s, angular=%.2f rad/s",
            msg->linear.x,
            msg->angular.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<VelocitySubscriber>());

    rclcpp::shutdown();
    return 0;
}
