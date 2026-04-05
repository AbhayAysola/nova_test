#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <cmath>

class DualPIDController : public rclcpp::Node {
public:
    DualPIDController() : Node("dual_pid_controller") {
        // --- ROS 2 PARAMETERS (Tune these live!) ---
        this->declare_parameter("kp_v", 0.04);
        this->declare_parameter("ki_v", 0.02);
        this->declare_parameter("kd_v", 0.0);
        this->declare_parameter("kp_s", 0.8);
        this->declare_parameter("kd_s", 0.0);
        this->declare_parameter("throttle_smooth", 1.0); // Low-pass filter (0.1 to 1.0)

        // Subscribers
        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/drive", 10, std::bind(&DualPIDController::ack_cb, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/autodrive/roboracer_1/odom", 10, std::bind(&DualPIDController::odom_cb, this, std::placeholders::_1));

        // Publishers
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/steering_command", 10);

        last_time_ = this->get_clock()->now();
    }

private:
    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        float vx = msg->twist.twist.linear.x;
        float vy = msg->twist.twist.linear.y;
        current_vel_ = std::sqrt(vx*vx + vy*vy);
    }

    void ack_cb(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0) return;

        // Fetch current parameters
        double kp_v = this->get_parameter("kp_v").as_double();
        double ki_v = this->get_parameter("ki_v").as_double();
        double kd_v = this->get_parameter("kd_v").as_double();
        double kp_s = this->get_parameter("kp_s").as_double();
        double kd_s = this->get_parameter("kd_s").as_double();
        double alpha = this->get_parameter("throttle_smooth").as_double();

        // --- THROTTLE PID WITH STUTTER FIXES ---
        float v_error = msg->speed - current_vel_;
        
        // Deadzone: If speed is within 0.05m/s of target, stop oscillating
        if (std::abs(v_error) < 0.05f) v_error = 0.0f;

        v_integral_ += v_error * dt;
        v_integral_ = std::clamp(v_integral_, -0.5f, 0.5f); // Anti-windup
        float v_deriv = (v_error - last_v_error_) / dt;
        
        float raw_throttle = (kp_v * v_error) + (ki_v * v_integral_) + (kd_v * v_deriv);

        // Low-Pass Filter: Prevents rapid stuttering by smoothing out changes
        float smooth_throttle = (alpha * raw_throttle) + ((1.0f - alpha) * last_published_throttle_);

        // --- STEERING PD ---
        float s_error = msg->steering_angle;
        float s_deriv = (s_error - last_s_error_) / dt;
        float steer_out = (kp_s * s_error) + (kd_s * s_deriv);

        // --- PUBLISH ---
        auto t_msg = std_msgs::msg::Float32();
        auto s_msg = std_msgs::msg::Float32();
        
        t_msg.data = std::clamp(smooth_throttle, -1.0f, 1.0f);
        s_msg.data = std::clamp(steer_out, -1.0f, 1.0f);

        throttle_pub_->publish(t_msg);
        steering_pub_->publish(s_msg);

        // Update state
        last_v_error_ = v_error;
        last_s_error_ = s_error;
        last_published_throttle_ = t_msg.data;
        last_time_ = now;
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDrive>::SharedPtr ackermann_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_, steering_pub_;

    float current_vel_ = 0.0, last_v_error_ = 0.0, v_integral_ = 0.0;
    float last_s_error_ = 0.0, last_published_throttle_ = 0.0;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualPIDController>());
    rclcpp::shutdown();
    return 0;
}
