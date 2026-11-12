#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class DistanceController : public rclcpp::Node
{
public:
    DistanceController() : Node("distance_controller")
    {
        declare_parameter<double>("target_distance", 1.0);
        declare_parameter<double>("max_speed", 0.2);
        declare_parameter<double>("kp", 0.8);

        cmd_pub_ =
            create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10);

        odom_sub_ =
            create_subscription<nav_msgs::msg::Odometry>(
                "/odom",
                10,
                std::bind(
                    &DistanceController::odom_callback,
                    this,
                    std::placeholders::_1));

        timer_ = create_wall_timer(
            100ms,
            std::bind(&DistanceController::control_loop, this));
    }

private:
    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        if (!initialized_) {
            start_x_ = x;
            start_y_ = y;
            initialized_ = true;
        }

        current_x_ = x;
        current_y_ = y;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (!initialized_) {
            cmd_pub_->publish(cmd);
            return;
        }

        double dx = current_x_ - start_x_;
        double dy = current_y_ - start_y_;

        double traveled = std::hypot(dx, dy);

        double target =
            get_parameter("target_distance").as_double();

        double max_speed =
            get_parameter("max_speed").as_double();

        double kp =
            get_parameter("kp").as_double();

        double error = target - traveled;

        if (error <= 0.02) {
            cmd.linear.x = 0.0;
            cmd_pub_->publish(cmd);

            if (!finished_) {
                RCLCPP_INFO(
                    get_logger(),
                    "Target reached! Distance: %.3f m",
                    traveled);
                finished_ = true;
            }
            return;
        }

        cmd.linear.x = std::clamp(
            kp * error, 0.0, max_speed);

        cmd_pub_->publish(cmd);

        RCLCPP_INFO(
            get_logger(),
            "Traveled: %.3f m, Error: %.3f m, Speed: %.3f m/s",
            traveled, error, cmd.linear.x);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double current_x_ = 0.0;
    double current_y_ = 0.0;

    bool initialized_ = false;
    bool finished_ = false;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DistanceController>());
    rclcpp::shutdown();
    return 0;
}
