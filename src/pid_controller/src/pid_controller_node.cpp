#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>
#include <cmath>


struct WheelState {
    double last_pos = 0.0;
    rclcpp::Time last_time;
    float current_vel = 0.0;
    bool initialized = false;
};

class DualPIDController : public rclcpp::Node {
public:
    WheelState left_val_, right_val_;

    DualPIDController() : Node("dual_pid_controller") {
        // --- ROS 2 PARAMETERS ---
        this->declare_parameter("kp_v", 0.04);
        this->declare_parameter("ki_v", 0.02);
        this->declare_parameter("kd_v", 0.0);
        this->declare_parameter("kp_s", 0.8);
        this->declare_parameter("kd_s", 0.0);
        this->declare_parameter("throttle_smooth", 0.15);
        this->declare_parameter("wheel_radius", 0.05); // Set this to your car's actual wheel radius

        // Subscribers
        ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDrive>(
            "/drive", 10, std::bind(&DualPIDController::ack_cb, this, std::placeholders::_1));
        
        left_enc_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/autodrive/roboracer_1/left_encoder", 10, 
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->update_velocity(msg, left_val_); });

        right_enc_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/autodrive/roboracer_1/right_encoder", 10, 
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) { this->update_velocity(msg, right_val_); });
        // Publishers
        throttle_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/throttle_command", 10);
        steering_pub_ = this->create_publisher<std_msgs::msg::Float32>("/autodrive/roboracer_1/steering_command", 10);

        last_time_ = this->get_clock()->now();
    }

private:
    void update_velocity(const sensor_msgs::msg::JointState::SharedPtr msg, WheelState &wheel) {
        rclcpp::Time now = msg->header.stamp;
        if (wheel.initialized) {
            double dt = (now - wheel.last_time).seconds();
            if (dt > 0.0001) { // Prevent division by zero
                double d_pos = msg->position[0] - wheel.last_pos;
                wheel.current_vel = static_cast<float>(d_pos / dt);
            }
        } else {
            wheel.initialized = true;
        }
        wheel.last_pos = msg->position[0];
        wheel.last_time = now;
    }

    void ack_cb(const ackermann_msgs::msg::AckermannDrive::SharedPtr msg) {
        auto now = this->get_clock()->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0) return;

        double r = this->get_parameter("wheel_radius").as_double();
        
        // Calculate linear velocity from angular wheel velocities: v = omega * r
        // Averaging left and right for the center-line velocity
        float current_vel = ((left_val_.current_vel * r) + (right_val_.current_vel * r)) / 2.0f;
        std::cout << current_vel << '\n';

        // Fetch PID parameters
        double kp_v = this->get_parameter("kp_v").as_double();
        double ki_v = this->get_parameter("ki_v").as_double();
        double kd_v = this->get_parameter("kd_v").as_double();
        double kp_s = this->get_parameter("kp_s").as_double();
        double kd_s = this->get_parameter("kd_s").as_double();
        double alpha = this->get_parameter("throttle_smooth").as_double();

        // --- THROTTLE PID ---
        float v_error = msg->speed - current_vel;
        if (std::abs(v_error) < 0.05f) v_error = 0.0f;

        v_integral_ += v_error * dt;
        v_integral_ = std::clamp(v_integral_, -0.5f, 0.5f);
        float v_deriv = (v_error - last_v_error_) / dt;
        
        float raw_throttle = (kp_v * v_error) + (ki_v * v_integral_) + (kd_v * v_deriv);
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
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_enc_sub_, right_enc_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_, steering_pub_;

    float last_v_error_ = 0.0, v_integral_ = 0.0;
    float last_s_error_ = 0.0, last_published_throttle_ = 0.0;
    rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DualPIDController>());
    rclcpp::shutdown();
    return 0;
}
