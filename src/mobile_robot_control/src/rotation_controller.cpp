#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class RotationController : public rclcpp::Node
{
public:
    RotationController() : Node("rotation_controller")
    {
        declare_parameter<double>("target_angle_deg", 90.0);
        declare_parameter<double>("max_angular_speed", 0.5);
        declare_parameter<double>("kp", 1.5);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(
                &RotationController::odom_callback,
                this,
                std::placeholders::_1));

        timer_ = create_wall_timer(
            100ms,
            std::bind(&RotationController::control_loop, this));
    }

private:
    static double normalize_angle(double angle)
    {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &q = msg->pose.pose.orientation;

        double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        if (!initialized_) {
            start_yaw_ = yaw;
            initialized_ = true;
        }

        current_yaw_ = yaw;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        if (!initialized_) {
            cmd_pub_->publish(cmd);
            return;
        }

        constexpr double pi = 3.14159265358979323846;

        double target_angle =
            get_parameter("target_angle_deg").as_double()
            * pi / 180.0;

        double max_speed =
            get_parameter("max_angular_speed").as_double();

        double kp = get_parameter("kp").as_double();

        double rotated =
            normalize_angle(current_yaw_ - start_yaw_);

        double error = normalize_angle(
            target_angle - rotated);

        if (std::abs(error) < 0.005) {
            cmd_pub_->publish(cmd);

            if (!finished_) {
                RCLCPP_INFO(
                    get_logger(),
                    "Rotation complete! Angle: %.2f deg",
                    rotated * 180.0 / pi);
                finished_ = true;
            }
            return;
        }

        cmd.angular.z = std::clamp(
            kp * error, -max_speed, max_speed);

        cmd_pub_->publish(cmd);

        RCLCPP_INFO(
            get_logger(),
            "Rotated: %.2f deg, Error: %.2f deg",
            rotated * 180.0 / pi,
            error * 180.0 / pi);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double start_yaw_ = 0.0;
    double current_yaw_ = 0.0;

    bool initialized_ = false;
    bool finished_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RotationController>());
    rclcpp::shutdown();
    return 0;
}
