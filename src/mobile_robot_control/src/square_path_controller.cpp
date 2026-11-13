#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class SquarePathController : public rclcpp::Node
{
public:
    SquarePathController() : Node("square_path_controller")
    {
        declare_parameter<double>("side_length", 1.0);
        declare_parameter<double>("max_linear_speed", 0.2);
        declare_parameter<double>("max_angular_speed", 0.4);
        declare_parameter<double>("settle_time", 0.5);
        declare_parameter<double>("stop_linear_tolerance", 0.01);
        declare_parameter<double>("stop_angular_tolerance", 0.02);

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&SquarePathController::odom_callback,
                      this, std::placeholders::_1));

        timer_ = create_wall_timer(
            100ms,
            std::bind(&SquarePathController::control_loop, this));
    }

    ~SquarePathController() override
    {
        publish_stop();
    }

private:
    enum class State {
        WAITING,
        MOVING,
        STOPPING_FOR_TURN,
        ROTATING,
        STOPPING_AFTER_TURN,
        FINISHED
    };

    static double normalize_angle(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    void publish_stop()
    {
        geometry_msgs::msg::Twist cmd;
        if (cmd_pub_) {
            cmd_pub_->publish(cmd);
        }
    }

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;

        const auto &q = msg->pose.pose.orientation;

        double yaw = std::atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        if (!have_odom_) {
            initial_x_ = x_;
            initial_y_ = y_;
            yaw_ = yaw;
            start_segment();
            segment_heading_ = normalize_angle(yaw_);
            state_ = State::MOVING;
            have_odom_ = true;

            RCLCPP_INFO(get_logger(), "Starting square path");
        } else {
            // Accumulate yaw changes across +/-180 degrees.
            accumulated_yaw_ += normalize_angle(yaw - yaw_);
            yaw_ = yaw;
        }

        const auto sample_time = now();
        const bool fresh_interval = have_stop_sample_ &&
            (sample_time - last_odom_time_).seconds() >= 0.0 &&
            (sample_time - last_odom_time_).seconds() <= 0.5;
        last_odom_time_ = sample_time;

        if (state_ == State::STOPPING_FOR_TURN ||
            state_ == State::STOPPING_AFTER_TURN) {
            const auto &v = msg->twist.twist;
            const bool stopped =
                std::isfinite(v.linear.x) && std::isfinite(v.linear.y) &&
                std::isfinite(v.angular.z) &&
                std::hypot(v.linear.x, v.linear.y) <=
                    get_parameter("stop_linear_tolerance").as_double() &&
                std::abs(v.angular.z) <=
                    get_parameter("stop_angular_tolerance").as_double();

            if (!stopped) {
                have_stop_sample_ = false;
            } else if (!fresh_interval) {
                stopped_since_ = sample_time;
                have_stop_sample_ = true;
            }
        }
    }

    void begin_stop(State next_state)
    {
        publish_stop();
        state_ = next_state;
        have_stop_sample_ = false;
    }

    bool stop_settled() const
    {
        // Require fresh odometry throughout the dwell, not just elapsed time.
        return have_stop_sample_ &&
            (last_odom_time_ - stopped_since_).seconds() >=
                get_parameter("settle_time").as_double();
    }

    void start_segment()
    {
        start_x_ = x_;
        start_y_ = y_;
        start_yaw_ = accumulated_yaw_;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        // Stop if odometry is unavailable or stale.
        if (!have_odom_ ||
            (now() - last_odom_time_).seconds() > 0.5) {
            publish_stop();
            have_stop_sample_ = false;
            return;
        }

        constexpr double PI = 3.14159265358979323846;

        switch (state_) {

        case State::MOVING: {
            double dx = x_ - start_x_;
            double dy = y_ - start_y_;

            // Progress along the segment's starting heading.
            double traveled =
                dx * std::cos(segment_heading_) +
                dy * std::sin(segment_heading_);

            double target =
                get_parameter("side_length").as_double();

            double error = target - traveled;

            if (error <= 0.02) {
                begin_stop(State::STOPPING_FOR_TURN);

                RCLCPP_INFO(
                    get_logger(),
                    "Side %d complete. Waiting to stop before rotating...",
                    completed_sides_ + 1);
                return;
            }

            double max_speed =
                get_parameter("max_linear_speed").as_double();

            cmd.linear.x = std::clamp(
                0.8 * error, 0.0, max_speed);

            // Heading correction during straight movement.
            double heading_error =
                normalize_angle(segment_heading_ - yaw_);

            cmd.angular.z = std::clamp(
                1.5 * heading_error, -0.3, 0.3);

            break;
        }

        case State::STOPPING_FOR_TURN:
            publish_stop();
            if (stop_settled()) {
                start_segment();
                state_ = State::ROTATING;
            }
            return;

        case State::ROTATING: {
            double rotated =
                accumulated_yaw_ - start_yaw_;

            double error = PI / 2.0 - rotated;

            if (std::abs(error) < 0.02) {
                begin_stop(State::STOPPING_AFTER_TURN);
                return;
            }

            double max_speed =
                get_parameter("max_angular_speed").as_double();

            cmd.angular.z = std::clamp(
                1.5 * error, -max_speed, max_speed);

            break;
        }

        case State::STOPPING_AFTER_TURN:
            publish_stop();
            if (!stop_settled()) {
                return;
            }

            RCLCPP_INFO(
                get_logger(),
                "Turn %d: settled odometry rotation = %.2f deg",
                completed_sides_ + 1,
                (accumulated_yaw_ - start_yaw_) * 180.0 / PI);
            completed_sides_++;

            if (completed_sides_ >= 4) {
                state_ = State::FINISHED;
                RCLCPP_INFO(get_logger(), "Square path completed!");
                RCLCPP_INFO(
                    get_logger(), "Final position error: %.3f m",
                    std::hypot(x_ - initial_x_, y_ - initial_y_));
            } else {
                start_segment();
                segment_heading_ = normalize_angle(yaw_);
                state_ = State::MOVING;
            }
            return;

        case State::FINISHED:
        case State::WAITING:
            publish_stop();
            return;
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING;

    bool have_odom_ = false;
    bool have_stop_sample_ = false;
    int completed_sides_ = 0;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double accumulated_yaw_ = 0.0;

    double initial_x_ = 0.0;
    double initial_y_ = 0.0;
    double start_x_ = 0.0;
    double start_y_ = 0.0;
    double start_yaw_ = 0.0;
    double segment_heading_ = 0.0;

    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time stopped_since_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePathController>());
    rclcpp::shutdown();
    return 0;
}
