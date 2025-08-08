// ================================================================================================
// INCLUDES & NAMESPACE
// ================================================================================================
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>

#include <Eigen/Dense>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

using std::placeholders::_1;

namespace ekf_odom_estimation
{

// ================================================================================================
// EXTENDED KALMAN FILTER CLASS - State Estimation Core
// State Vector: [x, y, theta, imu_bias]
// ================================================================================================
class EKF
{
public:
    EKF();
    void predict(double dt, double v, double w);  // Prediction step with motion model
    void update(double measured_omega, double R_val);  // Update with IMU angular velocity
    void updateHeading(double measured_theta, double R_val, double imu_offset = 0.0, double angular_vel = 0.0);  // Update with IMU heading
    void setState(const Eigen::Vector4d &state);
    Eigen::Vector4d getState();
    double getBias();

private:
    Eigen::Vector4d x_; // State: [x, y, theta, bias]
    Eigen::Matrix4d P_; // Covariance matrix
};

class VescToMyOdom : public rclcpp::Node
{
  public:
    explicit VescToMyOdom(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    // Callback functions
    void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state);
    void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu);
    void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo);

    // Helper functions
    void updateImuAcceleration(const vesc_msgs::msg::VescImuStamped::SharedPtr imu);
    bool detectSlip(double wheel_acceleration, double expected_omega, double measured_omega);
    void updateSlipFactor(bool slip_detected);
    double calculateAckermannOmega(double velocity, double servo_cmd);

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
    rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

    std::string odom_frame_;
    std::string base_frame_;
    double speed_to_erpm_gain_;
    double speed_to_erpm_offset_;
    double wheelbase_;
    double steering_to_servo_gain_;
    double steering_to_servo_offset_;
    double imu_offset_x_; // IMU offset from rear axle (positive = forward)
    bool publish_tf_;
    bool use_servo_cmd_;

    double x_, y_, initial_yaw_;
    double imu_yaw_rate_, real_yaw_rate_;
    std_msgs::msg::Float64::SharedPtr last_servo_cmd_;
    bool ekf_initialized_;
    rclcpp::Time initialization_time_;
    rclcpp::Time last_time_;

    // Slip detection variables
    double last_speed_;
    double slip_factor_;
    std::queue<double> speed_history_;
    std::queue<double> accel_history_;

    // IMU acceleration for slip detection
    std::queue<double> imu_accel_x_history_;
    std::queue<double> imu_accel_y_history_;
    double last_imu_accel_x_, last_imu_accel_y_;

    // IMU outlier detection
    std::queue<double> imu_yaw_history_;
    std::queue<double> imu_rate_history_;
    double last_imu_yaw_;

    EKF ekf_;
    std::queue<vesc_msgs::msg::VescStateStamped::SharedPtr> vesc_buffer_;
};

// ================================================================================================
// EKF IMPLEMENTATION - Extended Kalman Filter Methods
// ================================================================================================
EKF::EKF()
{
    x_ = Eigen::Vector4d::Zero(); // Initialize state: [x, y, theta, bias]
    P_ = Eigen::Matrix4d::Identity() * 0.1;  // Initial covariance
    P_(3, 3) = 0.01; // Lower initial uncertainty for IMU bias
}

// --------------------------------- EKF PREDICTION STEP ---------------------------------
// Uses motion model: x += v*cos(θ)*dt, y += v*sin(θ)*dt, θ += (ω-bias)*dt
void EKF::predict(double dt, double v, double w)
{
    double theta = x_(2);
    double bias = x_(3);

    Eigen::Vector4d x_pred;
    x_pred(0) = x_(0) + v * cos(theta) * dt;
    x_pred(1) = x_(1) + v * sin(theta) * dt;
    x_pred(2) = x_(2) + (w - bias) * dt; // Apply bias correction
    x_pred(3) = x_(3);                   // Bias evolves slowly

    while (x_pred(2) > M_PI)
        x_pred(2) -= 2 * M_PI;
    while (x_pred(2) < -M_PI)
        x_pred(2) += 2 * M_PI;

    Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
    F(0, 2) = -v * sin(theta) * dt;
    F(1, 2) = v * cos(theta) * dt;
    F(2, 3) = -dt; // Bias affects heading prediction

    // Adaptive process noise based on speed and angular velocity
    double speed_factor = std::max(0.001, std::abs(v) * 0.02);
    double angular_factor = std::max(0.001, std::abs(w) * 0.05);
    double dt_factor = std::max(0.1, dt * 10.0);

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = speed_factor * dt_factor;   // x position uncertainty
    Q(1, 1) = speed_factor * dt_factor;   // y position uncertainty
    Q(2, 2) = angular_factor * dt_factor; // heading uncertainty
    Q(3, 3) = 0.0001 * dt;                // Bias random walk

    P_ = F * P_ * F.transpose() + Q;
    x_ = x_pred;
}

// --------------------------------- EKF UPDATE: ANGULAR VELOCITY ---------------------------------
// Updates IMU bias using angular velocity measurement from IMU
void EKF::update(double measured_omega, double R_val)
{
    // Update using angular velocity measurement (IMU)
    double predicted_omega = x_(2); // This should be angular velocity from state, but we're using heading
    // We need to track angular velocity in state or calculate it
    double bias = x_(3);
    double corrected_predicted_omega = predicted_omega - bias; // Apply bias correction

    double innovation = measured_omega - corrected_predicted_omega;

    // Observation model: we observe angular velocity directly
    Eigen::RowVector4d H;
    H << 0, 0, 0, -1; // Observe bias indirectly through angular velocity

    double S = H * P_ * H.transpose() + R_val;
    Eigen::Vector4d K = P_ * H.transpose() / S;

    x_ = x_ + K * innovation;

    // Clamp bias to reasonable range
    x_(3) = std::max(-0.5, std::min(0.5, x_(3)));

    P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
}

// --------------------------------- EKF UPDATE: HEADING ANGLE ---------------------------------
// Updates vehicle heading using IMU yaw measurement (with offset compensation)
void EKF::updateHeading(double measured_theta, double R_val, double imu_offset, double angular_vel)
{
    double theta_pred = x_(2);

    // Compensate for IMU offset during rotation
    // IMU is between front-rear axles, so minimal theta correction needed
    double imu_theta_correction = 0.0;
    if (std::abs(angular_vel) > 0.1 && imu_offset != 0.0)
    {
        // Small correction for IMU being slightly ahead of vehicle center
        double dt_effective = 0.005; // Smaller correction since IMU is closer to center
        imu_theta_correction = angular_vel * dt_effective * (imu_offset * 0.3); // Reduced factor
    }

    double corrected_measured_theta = measured_theta - imu_theta_correction;
    double y = corrected_measured_theta - theta_pred;

    while (y > M_PI)
        y -= 2 * M_PI;
    while (y < -M_PI)
        y += 2 * M_PI;

    Eigen::RowVector4d H;
    H << 0, 0, 1, 0; // Only observe heading, not bias directly

    double S = H * P_ * H.transpose() + R_val;
    Eigen::Vector4d K = P_ * H.transpose() / S;

    x_ = x_ + K * y;

    while (x_(2) > M_PI)
        x_(2) -= 2 * M_PI;
    while (x_(2) < -M_PI)
        x_(2) += 2 * M_PI;

    // Clamp bias to reasonable range
    x_(3) = std::max(-0.5, std::min(0.5, x_(3)));

    P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
}

void EKF::setState(const Eigen::Vector4d &state)
{
    x_ = state;
}

Eigen::Vector4d EKF::getState()
{
    return x_;
}

double EKF::getBias()
{
    return x_(3);
}

// ================================================================================================
// NODE CONSTRUCTOR - Initialize ROS2 Node & Parameters
// ================================================================================================
VescToMyOdom::VescToMyOdom(const rclcpp::NodeOptions &options)
    : Node("vesc_to_my_odom_node", options), x_(0.0), y_(0.0), initial_yaw_(0.0), imu_yaw_rate_(0.0),
      real_yaw_rate_(0.0), last_servo_cmd_(nullptr), ekf_initialized_(false), initialization_time_(rclcpp::Time(0)),
      last_speed_(0.0), slip_factor_(1.0), last_imu_accel_x_(0.0), last_imu_accel_y_(0.0), last_imu_yaw_(0.0)
{
    // --------------------------------- LOAD ROS2 PARAMETERS ---------------------------------
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    
    // Vehicle & sensor parameters (loaded from f1tenth_stack/config/vesc.yaml)
    wheelbase_ = this->declare_parameter<double>("wheelbase", 0.324);
    steering_to_servo_gain_ = this->declare_parameter<double>("steering_angle_to_servo_gain", -1.2135);
    steering_to_servo_offset_ = this->declare_parameter<double>("steering_angle_to_servo_offset", 0.1667);
    speed_to_erpm_gain_ = this->declare_parameter<double>("speed_to_erpm_gain", 4614.0);
    speed_to_erpm_offset_ = this->declare_parameter<double>("speed_to_erpm_offset", 0.0);
    imu_offset_x_ = declare_parameter("imu_offset_x", 0.20); // 200mm forward from rear axle
    // Behavior parameters
    publish_tf_ = declare_parameter("publish_tf", false);
    use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);

    // --------------------------------- SETUP ROS2 PUBLISHERS & SUBSCRIBERS ---------------------------------
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    if (publish_tf_)
    {
        tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
    }

    vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
        "sensors/core", 10, std::bind(&VescToMyOdom::vescStateCallback, this, _1));

    imu_sub_ = create_subscription<vesc_msgs::msg::VescImuStamped>("sensors/imu", 10,
                                                                   std::bind(&VescToMyOdom::imuCallback, this, _1));

    if (use_servo_cmd_)
    {
        servo_sub_ = create_subscription<std_msgs::msg::Float64>("sensors/servo_position_command", 10,
                                                                 std::bind(&VescToMyOdom::servoCmdCallback, this, _1));
    }

    // --------------------------------- INITIALIZE EKF STATE ---------------------------------
    Eigen::Vector4d init_state;
    init_state(0) = 0.0; // x
    init_state(1) = 0.0; // y
    init_state(2) = 0.0; // yaw
    init_state(3) = 0.0; // bias
    ekf_.setState(init_state);

    last_time_ = this->now();
}

// ================================================================================================
// IMU CALLBACK - Process IMU Data & Update EKF
// ================================================================================================
void VescToMyOdom::imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu)
{
    // --------------------------------- EXTRACT IMU DATA ---------------------------------
    double measured_yaw_deg = -imu->imu.ypr.z;
    double measured_yaw_rad = measured_yaw_deg * M_PI / 180.0;

    double imu_angular_velocity_z_deg = -imu->imu.angular_velocity.z;
    double raw_yaw_rate = imu_angular_velocity_z_deg * M_PI / 180.0;

    // --------------------------------- UPDATE IMU ACCELERATION FOR SLIP DETECTION ---------------------------------
    updateImuAcceleration(imu);

    // --------------------------------- IMU OUTLIER DETECTION ---------------------------------
    bool is_outlier = false;

    // Update IMU history
    imu_yaw_history_.push(measured_yaw_rad);
    imu_rate_history_.push(raw_yaw_rate);
    if (imu_yaw_history_.size() > 5)
    {
        imu_yaw_history_.pop();
        imu_rate_history_.pop();
    }

    // Check for large jumps in yaw angle
    if (imu_yaw_history_.size() >= 2)
    {
        double yaw_change = std::abs(measured_yaw_rad - last_imu_yaw_);
        if (yaw_change > 0.5)
        { // More than ~28 degrees jump
            is_outlier = true;
            RCLCPP_WARN(this->get_logger(), "IMU yaw outlier detected: change = %f rad", yaw_change);
        }
    }

    // Check for excessive angular velocity
    if (std::abs(raw_yaw_rate) > 8.0)
    { // More than 8 rad/s
        is_outlier = true;
        RCLCPP_WARN(this->get_logger(), "IMU rate outlier detected: rate = %f rad/s", raw_yaw_rate);
    }

    // Use previous valid measurements if outlier detected
    if (is_outlier && imu_rate_history_.size() >= 3)
    {
        // Use median of recent measurements
        std::vector<double> rates;
        std::queue<double> temp_queue = imu_rate_history_;
        while (!temp_queue.empty())
        {
            rates.push_back(temp_queue.front());
            temp_queue.pop();
        }
        std::sort(rates.begin(), rates.end());
        imu_yaw_rate_ = rates[rates.size() / 2];
        return; // Skip the rest of processing for this outlier
    }
    else
    {
        imu_yaw_rate_ = std::max(-5.0, std::min(5.0, raw_yaw_rate));
        last_imu_yaw_ = measured_yaw_rad;
    }

    real_yaw_rate_ = imu_yaw_rate_;

    if (!ekf_initialized_)
    {
        initial_yaw_ = measured_yaw_rad;
        ekf_initialized_ = true;
        initialization_time_ = imu->header.stamp;
        last_time_ = imu->header.stamp;

        RCLCPP_INFO(this->get_logger(), "EKF initialized at origin with IMU yaw: %f rad (%f deg)", initial_yaw_,
                    measured_yaw_deg);

        // Process any buffered VESC messages
        while (!vesc_buffer_.empty())
        {
            auto buffered_state = vesc_buffer_.front();
            vesc_buffer_.pop();
            rclcpp::Time buffer_time(buffered_state->header.stamp);
            if (buffer_time >= initialization_time_)
            {
                vescStateCallback(buffered_state);
            }
        }
        return;
    }

    double corrected_yaw = measured_yaw_rad - initial_yaw_;

    while (corrected_yaw > M_PI)
        corrected_yaw -= 2 * M_PI;
    while (corrected_yaw < -M_PI)
        corrected_yaw += 2 * M_PI;

    double R_omega = 0.1; // Angular velocity measurement noise
    ekf_.update(imu_yaw_rate_, R_omega);

    // Also update heading for additional constraint
    double R_yaw = 0.05;
    ekf_.updateHeading(corrected_yaw, R_yaw, imu_offset_x_, imu_yaw_rate_);
}

// --------------------------------- SERVO COMMAND CALLBACK ---------------------------------
void VescToMyOdom::servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo)
{
    last_servo_cmd_ = servo;
}

// ================================================================================================
// VESC STATE CALLBACK - Main Odometry Processing & EKF Prediction
// ================================================================================================
void VescToMyOdom::vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state)
{
    if (!ekf_initialized_)
    {
        if (vesc_buffer_.size() > 100)
        {
            vesc_buffer_.pop();
        }
        vesc_buffer_.push(state);
        return;
    }

    double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(current_speed) < 0.05)
    {
        current_speed = 0.0;
    }

    rclcpp::Time current_time = state->header.stamp;
    double dt = (current_time - last_time_).seconds();

    // Slip detection and compensation
    if (dt > 0.001)
    {
        double acceleration = (current_speed - last_speed_) / dt;

        // Update speed and acceleration history
        speed_history_.push(current_speed);
        accel_history_.push(acceleration);
        if (speed_history_.size() > 10)
        {
            speed_history_.pop();
            accel_history_.pop();
        }

        // Calculate expected angular velocity from Ackermann model
        double expected_omega = 0.0;
        if (last_servo_cmd_ && use_servo_cmd_)
        {
            expected_omega = calculateAckermannOmega(current_speed, last_servo_cmd_->data);
        }

        // Enhanced slip detection using IMU acceleration and model comparison
        bool wheel_slip_detected = detectSlip(acceleration, expected_omega, real_yaw_rate_);
        updateSlipFactor(wheel_slip_detected);

        // Apply slip compensation to speed
        current_speed *= slip_factor_;
        last_speed_ = current_speed;
    }

    if (dt > 1.0 || dt <= 0.0)
    {
        last_time_ = current_time;
        return;
    }

    last_time_ = current_time;

    // Use Ackermann model for prediction if available, otherwise use IMU
    double prediction_omega = real_yaw_rate_; // Default to IMU
    if (last_servo_cmd_ && use_servo_cmd_)
    {
        prediction_omega = calculateAckermannOmega(current_speed, last_servo_cmd_->data);
    }

    ekf_.predict(dt, current_speed, prediction_omega);

    Eigen::Vector4d state_est = ekf_.getState();

    if (std::isnan(state_est(0)) || std::isnan(state_est(1)) || std::isnan(state_est(2)) ||
        std::abs(state_est(0)) > 1000 || std::abs(state_est(1)) > 1000)
    {
        RCLCPP_ERROR(this->get_logger(), "EKF state invalid, resetting");
        ekf_initialized_ = false;
        return;
    }

    x_ = state_est(0);
    y_ = state_est(1);
    double theta_est = state_est(2);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = state->header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta_est / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta_est / 2.0);

    odom_msg.pose.covariance[0] = 0.1;
    odom_msg.pose.covariance[7] = 0.1;
    odom_msg.pose.covariance[35] = 0.2;

    odom_msg.twist.twist.linear.x = current_speed;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = real_yaw_rate_;

    if (publish_tf_)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.frame_id = odom_frame_;
        tf.child_frame_id = base_frame_;
        tf.header.stamp = current_time;
        tf.transform.translation.x = x_;
        tf.transform.translation.y = y_;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation = odom_msg.pose.pose.orientation;

        if (rclcpp::ok())
        {
            tf_pub_->sendTransform(tf);
        }
    }

    odom_pub_->publish(odom_msg);
}

// ================================================================================================
// HELPER FUNCTIONS - Slip Detection, Ackermann Model, etc.
// ================================================================================================

// --------------------------------- UPDATE IMU ACCELERATION DATA ---------------------------------
void VescToMyOdom::updateImuAcceleration(const vesc_msgs::msg::VescImuStamped::SharedPtr imu)
{
    double imu_accel_x = imu->imu.linear_acceleration.x; // Forward acceleration
    double imu_accel_y = imu->imu.linear_acceleration.y; // Lateral acceleration

    // Update IMU acceleration history
    imu_accel_x_history_.push(imu_accel_x);
    imu_accel_y_history_.push(imu_accel_y);

    if (imu_accel_x_history_.size() > 10)
    {
        imu_accel_x_history_.pop();
        imu_accel_y_history_.pop();
    }

    last_imu_accel_x_ = imu_accel_x;
    last_imu_accel_y_ = imu_accel_y;
}

// --------------------------------- ENHANCED SLIP DETECTION (5 METHODS) ---------------------------------\nbool VescToMyOdom::detectSlip(double wheel_acceleration, double expected_omega, double measured_omega)
{
    // Slip detection thresholds
    constexpr double WHEEL_ACCEL_THRESHOLD = 2.5;       // m/s^2 from wheel speed
    constexpr double HIGH_ANGULAR_THRESHOLD = 2.0;      // rad/s
    constexpr double LATERAL_ACCEL_THRESHOLD = 3.0;     // m/s^2 for cornering slip
    constexpr double ACCEL_DISCREPANCY_THRESHOLD = 1.5; // m/s^2
    constexpr double TOTAL_ACCEL_THRESHOLD = 4.0;       // m/s^2
    constexpr double OMEGA_DISCREPANCY_THRESHOLD = 0.8; // rad/s for angular velocity mismatch

    // Compare wheel-derived acceleration with IMU acceleration
    double accel_discrepancy = std::abs(wheel_acceleration - last_imu_accel_x_);
    double total_imu_accel = sqrt(last_imu_accel_x_ * last_imu_accel_x_ + last_imu_accel_y_ * last_imu_accel_y_);

    // NEW: Compare expected vs measured angular velocity (Ackermann model vs IMU)
    double omega_discrepancy = std::abs(expected_omega - measured_omega);

    // Multiple slip detection methods
    bool longitudinal_slip =
        (accel_discrepancy > ACCEL_DISCREPANCY_THRESHOLD && std::abs(wheel_acceleration) > WHEEL_ACCEL_THRESHOLD);
    bool lateral_slip = (std::abs(last_imu_accel_y_) > LATERAL_ACCEL_THRESHOLD);
    bool combined_slip = (total_imu_accel > TOTAL_ACCEL_THRESHOLD);
    bool traditional_slip =
        (std::abs(wheel_acceleration) > WHEEL_ACCEL_THRESHOLD || std::abs(measured_omega) > HIGH_ANGULAR_THRESHOLD);

    // NEW: Model-based slip detection (Ackermann vs IMU)
    bool model_slip = (omega_discrepancy > OMEGA_DISCREPANCY_THRESHOLD &&
                       std::abs(expected_omega) > 0.2); // Only check when significant steering expected

    bool slip_detected = longitudinal_slip || lateral_slip || combined_slip || traditional_slip || model_slip;

    if (slip_detected)
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "Slip detected: accel_diff=%.2f, lat_accel=%.2f, total_accel=%.2f, omega_diff=%.2f",
                     accel_discrepancy, last_imu_accel_y_, total_imu_accel, omega_discrepancy);
    }

    return slip_detected;
}

// --------------------------------- UPDATE SLIP COMPENSATION FACTOR ---------------------------------
void VescToMyOdom::updateSlipFactor(bool slip_detected)
{
    constexpr double MIN_SLIP_FACTOR = 0.6;      // Minimum trust in wheel odometry
    constexpr double SLIP_REDUCTION_RATE = 0.92; // Aggressive reduction during slip
    constexpr double SLIP_RECOVERY_RATE = 1.03;  // Gradual recovery when no slip

    if (slip_detected)
    {
        // Reduce trust in wheel odometry during slip
        slip_factor_ = std::max(MIN_SLIP_FACTOR, slip_factor_ * SLIP_REDUCTION_RATE);
    }
    else
    {
        // Gradually restore trust when no slip detected
        slip_factor_ = std::min(1.0, slip_factor_ * SLIP_RECOVERY_RATE);
    }
}

// --------------------------------- ACKERMANN STEERING MODEL ---------------------------------
// Calculate expected angular velocity from servo command using bicycle model
double VescToMyOdom::calculateAckermannOmega(double velocity, double servo_cmd)
{
    // Convert servo command to steering angle (radians)
    double steering_angle = (servo_cmd - steering_to_servo_offset_) / steering_to_servo_gain_;

    // Clamp steering angle to reasonable range (approximately +/- 30 degrees)
    constexpr double MAX_STEERING_ANGLE = 0.52; // ~30 degrees in radians
    steering_angle = std::max(-MAX_STEERING_ANGLE, std::min(MAX_STEERING_ANGLE, steering_angle));

    // Calculate angular velocity using bicycle model: ω = v * tan(δ) / L
    double angular_velocity = 0.0;
    if (std::abs(velocity) > 0.05 && wheelbase_ > 0.0)
    {
        angular_velocity = velocity * tan(steering_angle) / wheelbase_;
    }

    // Clamp angular velocity to reasonable range
    constexpr double MAX_ANGULAR_VELOCITY = 5.0; // rad/s
    angular_velocity = std::max(-MAX_ANGULAR_VELOCITY, std::min(MAX_ANGULAR_VELOCITY, angular_velocity));

    return angular_velocity;
}

} // namespace ekf_odom_estimation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ekf_odom_estimation::VescToMyOdom)
