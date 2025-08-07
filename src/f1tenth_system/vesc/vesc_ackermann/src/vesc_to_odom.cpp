#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <queue>
#include <limits>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

namespace ekf_odom_estimation {

class EKF {
public:
  EKF();
  void predict(double dt, double v, double w);
  void update(double measured_theta, double R_val);
  void setState(const Eigen::Vector4d& state);
  Eigen::Vector4d getState();
  double getBias();

private:
  Eigen::Vector4d x_;  // [x, y, theta, bias]
  Eigen::Matrix4d P_;
};

class VescToMyOdom : public rclcpp::Node {
public:
  explicit VescToMyOdom(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state);
  void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu);
  void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo);

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
  
  // IMU outlier detection
  std::queue<double> imu_yaw_history_;
  std::queue<double> imu_rate_history_;
  double last_imu_yaw_;

  EKF ekf_;
  std::queue<vesc_msgs::msg::VescStateStamped::SharedPtr> vesc_buffer_;
};

EKF::EKF() {
  x_ = Eigen::Vector4d::Zero();  // [x, y, theta, bias]
  P_ = Eigen::Matrix4d::Identity() * 0.1;
  P_(3,3) = 0.01; // Lower initial uncertainty for bias
}

void EKF::predict(double dt, double v, double w) {
  double theta = x_(2);
  double bias = x_(3);
  
  Eigen::Vector4d x_pred;
  x_pred(0) = x_(0) + v * cos(theta) * dt;
  x_pred(1) = x_(1) + v * sin(theta) * dt;
  x_pred(2) = x_(2) + (w - bias) * dt;  // Apply bias correction
  x_pred(3) = x_(3);  // Bias evolves slowly

  while (x_pred(2) > M_PI) x_pred(2) -= 2 * M_PI;
  while (x_pred(2) < -M_PI) x_pred(2) += 2 * M_PI;

  Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
  F(0,2) = -v * sin(theta) * dt;
  F(1,2) =  v * cos(theta) * dt;
  F(2,3) = -dt;  // Bias affects heading prediction

  // Adaptive process noise based on speed and angular velocity
  double speed_factor = std::max(0.001, std::abs(v) * 0.02);
  double angular_factor = std::max(0.001, std::abs(w) * 0.05);
  double dt_factor = std::max(0.1, dt * 10.0);
  
  Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
  Q(0,0) = speed_factor * dt_factor;  // x position uncertainty
  Q(1,1) = speed_factor * dt_factor;  // y position uncertainty  
  Q(2,2) = angular_factor * dt_factor; // heading uncertainty
  Q(3,3) = 0.0001 * dt;  // Bias random walk

  P_ = F * P_ * F.transpose() + Q;
  x_ = x_pred;
}

void EKF::update(double measured_theta, double R_val) {
  double theta_pred = x_(2);
  double y = measured_theta - theta_pred;

  while (y > M_PI) y -= 2 * M_PI;
  while (y < -M_PI) y += 2 * M_PI;

  Eigen::RowVector4d H;
  H << 0, 0, 1, 0;  // Only observe heading, not bias directly

  double S = H * P_ * H.transpose() + R_val;
  Eigen::Vector4d K = P_ * H.transpose() / S;

  x_ = x_ + K * y;
  
  while (x_(2) > M_PI) x_(2) -= 2 * M_PI;
  while (x_(2) < -M_PI) x_(2) += 2 * M_PI;
  
  // Clamp bias to reasonable range
  x_(3) = std::max(-0.5, std::min(0.5, x_(3)));
  
  P_ = (Eigen::Matrix4d::Identity() - K * H) * P_;
}

void EKF::setState(const Eigen::Vector4d& state) {
  x_ = state;
}

Eigen::Vector4d EKF::getState() {
  return x_;
}

double EKF::getBias() {
  return x_(3);
}

VescToMyOdom::VescToMyOdom(const rclcpp::NodeOptions & options)
: Node("vesc_to_my_odom_node", options),
  x_(0.0), y_(0.0), initial_yaw_(0.0),
  imu_yaw_rate_(0.0), real_yaw_rate_(0.0), last_servo_cmd_(nullptr),
  ekf_initialized_(false), initialization_time_(rclcpp::Time(0)),
  last_speed_(0.0), slip_factor_(1.0), last_imu_yaw_(0.0)
{
  odom_frame_ = declare_parameter("odom_frame", "odom");
  base_frame_ = declare_parameter("base_frame", "base_link");
  speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 4200.0);
  speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
  wheelbase_ = declare_parameter("wheelbase", 0.25);
  steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", -1.2135);
  steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset", 0.570526);
  publish_tf_ = declare_parameter("publish_tf", false);
  use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  
  if (publish_tf_) {
    tf_pub_.reset(new tf2_ros::TransformBroadcaster(this));
  }
  
  vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
    "sensors/core", 10, std::bind(&VescToMyOdom::vescStateCallback, this, _1));
  
  imu_sub_ = create_subscription<vesc_msgs::msg::VescImuStamped>(
    "sensors/imu", 10, std::bind(&VescToMyOdom::imuCallback, this, _1));
  
  if (use_servo_cmd_) {
    servo_sub_ = create_subscription<std_msgs::msg::Float64>(
      "sensors/servo_position_command", 10, std::bind(&VescToMyOdom::servoCmdCallback, this, _1));
  }
  

  // Initialize EKF at origin
  Eigen::Vector4d init_state;
  init_state(0) = 0.0;  // x
  init_state(1) = 0.0;  // y
  init_state(2) = 0.0;  // yaw
  init_state(3) = 0.0;  // bias
  ekf_.setState(init_state);
  
  last_time_ = this->now();
}


void VescToMyOdom::imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu) {
  double measured_yaw_deg = -imu->imu.ypr.z;
  double measured_yaw_rad = measured_yaw_deg * M_PI / 180.0;
  
  double imu_angular_velocity_z_deg = -imu->imu.angular_velocity.z;
  double raw_yaw_rate = imu_angular_velocity_z_deg * M_PI / 180.0;
  
  // Outlier detection for IMU measurements
  bool is_outlier = false;
  
  // Update IMU history
  imu_yaw_history_.push(measured_yaw_rad);
  imu_rate_history_.push(raw_yaw_rate);
  if (imu_yaw_history_.size() > 5) {
    imu_yaw_history_.pop();
    imu_rate_history_.pop();
  }
  
  // Check for large jumps in yaw angle
  if (imu_yaw_history_.size() >= 2) {
    double yaw_change = std::abs(measured_yaw_rad - last_imu_yaw_);
    if (yaw_change > 0.5) {  // More than ~28 degrees jump
      is_outlier = true;
      RCLCPP_WARN(this->get_logger(), "IMU yaw outlier detected: change = %f rad", yaw_change);
    }
  }
  
  // Check for excessive angular velocity
  if (std::abs(raw_yaw_rate) > 8.0) {  // More than 8 rad/s
    is_outlier = true;
    RCLCPP_WARN(this->get_logger(), "IMU rate outlier detected: rate = %f rad/s", raw_yaw_rate);
  }
  
  // Use previous valid measurements if outlier detected
  if (is_outlier && imu_rate_history_.size() >= 3) {
    // Use median of recent measurements
    std::vector<double> rates;
    std::queue<double> temp_queue = imu_rate_history_;
    while (!temp_queue.empty()) {
      rates.push_back(temp_queue.front());
      temp_queue.pop();
    }
    std::sort(rates.begin(), rates.end());
    imu_yaw_rate_ = rates[rates.size()/2];
    return;  // Skip the rest of processing for this outlier
  } else {
    imu_yaw_rate_ = std::max(-5.0, std::min(5.0, raw_yaw_rate));
    last_imu_yaw_ = measured_yaw_rad;
  }
  
  real_yaw_rate_ = imu_yaw_rate_;

  if (!ekf_initialized_) {
    initial_yaw_ = measured_yaw_rad;
    ekf_initialized_ = true;
    initialization_time_ = imu->header.stamp;
    last_time_ = imu->header.stamp;
    
    RCLCPP_INFO(this->get_logger(), "EKF initialized at origin with IMU yaw: %f rad (%f deg)", 
                initial_yaw_, measured_yaw_deg);
    
    // Process any buffered VESC messages
    while (!vesc_buffer_.empty()) {
      auto buffered_state = vesc_buffer_.front();
      vesc_buffer_.pop();
      rclcpp::Time buffer_time(buffered_state->header.stamp);
      if (buffer_time >= initialization_time_) {
        vescStateCallback(buffered_state);
      }
    }
    return;
  }

  double corrected_yaw = measured_yaw_rad - initial_yaw_;
  
  while (corrected_yaw > M_PI) corrected_yaw -= 2 * M_PI;
  while (corrected_yaw < -M_PI) corrected_yaw += 2 * M_PI;
  
  double R_val = 0.05;
  ekf_.update(corrected_yaw, R_val);
}

void VescToMyOdom::servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo) {
  last_servo_cmd_ = servo;
}

void VescToMyOdom::vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state) {
  if (!ekf_initialized_) {
    if (vesc_buffer_.size() > 100) {
      vesc_buffer_.pop();
    }
    vesc_buffer_.push(state);
    return;
  }
  
  double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05) {
    current_speed = 0.0;
  }

  rclcpp::Time current_time = state->header.stamp;
  double dt = (current_time - last_time_).seconds();
  
  // Slip detection and compensation
  if (dt > 0.001) {
    double acceleration = (current_speed - last_speed_) / dt;
    
    // Update speed and acceleration history
    speed_history_.push(current_speed);
    accel_history_.push(acceleration);
    if (speed_history_.size() > 10) {
      speed_history_.pop();
      accel_history_.pop();
    }
    
    // Detect slip based on high acceleration and angular velocity
    double high_accel_threshold = 3.0;  // m/s^2
    double high_angular_threshold = 2.0; // rad/s
    
    if (std::abs(acceleration) > high_accel_threshold || 
        std::abs(imu_yaw_rate_) > high_angular_threshold) {
      slip_factor_ = std::max(0.7, slip_factor_ * 0.95);  // Reduce trust in wheel odometry
    } else {
      slip_factor_ = std::min(1.0, slip_factor_ * 1.02);  // Gradually restore trust
    }
    
    // Apply slip compensation to speed
    current_speed *= slip_factor_;
    last_speed_ = current_speed;
  }
  
  if (dt > 1.0 || dt <= 0.0) {
    last_time_ = current_time;
    return;
  }
  
  last_time_ = current_time;

  ekf_.predict(dt, current_speed, real_yaw_rate_);
  
  Eigen::Vector4d state_est = ekf_.getState();
  
  if (std::isnan(state_est(0)) || std::isnan(state_est(1)) || std::isnan(state_est(2)) ||
      std::abs(state_est(0)) > 1000 || std::abs(state_est(1)) > 1000) {
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
  
  if (publish_tf_) {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = current_time;
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation = odom_msg.pose.pose.orientation;
    
    if (rclcpp::ok()) {
      tf_pub_->sendTransform(tf);
    }
  }
  
  odom_pub_->publish(odom_msg);
}

} // namespace ekf_odom_estimation

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(ekf_odom_estimation::VescToMyOdom)