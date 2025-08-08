#include <cmath>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vesc_msgs/msg/vesc_imu_stamped.hpp"
#include "vesc_msgs/msg/vesc_state_stamped.hpp"

using std::placeholders::_1;

namespace ekf_odom_estimation {

// Simple EKF class to estimate [x, y, theta]
class EKF {
public:
  EKF() {
    x_ = Eigen::Vector3d::Zero(); // [x, y, theta]
    P_ = Eigen::Matrix3d::Identity() * 0.1;
  }

  void predict(double dt, double v, double w) {
    double theta = x_(2);
    
    Eigen::Vector3d x_pred;
    x_pred(0) = x_(0) + v * cos(theta) * dt;
    x_pred(1) = x_(1) + v * sin(theta) * dt;
    x_pred(2) = x_(2) + w * dt;
    
    // Normalize theta
    while (x_pred(2) > M_PI) x_pred(2) -= 2 * M_PI;
    while (x_pred(2) < -M_PI) x_pred(2) += 2 * M_PI;

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0,2) = -v * sin(theta) * dt;
    F(1,2) = v * cos(theta) * dt;

    Eigen::Matrix3d Q = Eigen::Matrix3d::Identity();
    Q(0,0) = 0.01;  // x position noise
    Q(1,1) = 0.01;  // y position noise  
    Q(2,2) = 0.05;  // theta noise (higher for angle)

    P_ = F * P_ * F.transpose() + Q;
    x_ = x_pred;
  }

  void update(double measured_theta, double R_val) {
    double theta_pred = x_(2);
    double y = measured_theta - theta_pred;

    // Normalize angle difference
    while (y > M_PI) y -= 2 * M_PI;
    while (y < -M_PI) y += 2 * M_PI;

    Eigen::RowVector3d H;
    H << 0, 0, 1; // Only observe heading

    double S = H * P_ * H.transpose() + R_val;
    Eigen::Vector3d K = P_ * H.transpose() / S;

    x_ = x_ + K * y;
    
    // Normalize theta after update
    while (x_(2) > M_PI) x_(2) -= 2 * M_PI;
    while (x_(2) < -M_PI) x_(2) += 2 * M_PI;

    P_ = (Eigen::Matrix3d::Identity() - K * H) * P_;
  }

  Eigen::Vector3d getState() {
    return x_;
  }

private:
  Eigen::Vector3d x_;
  Eigen::Matrix3d P_;
};

class VescToMyOdom : public rclcpp::Node {
public:
  VescToMyOdom(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vesc_to_my_odom_node", options),
    x_(0.0), y_(0.0), initial_yaw_(0.0), initialized_(false), compensated_yaw_rate_(0.0)
  {
    // Parameters
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base_link");
    speed_to_erpm_gain_ = declare_parameter("speed_to_erpm_gain", 4614.0);
    speed_to_erpm_offset_ = declare_parameter("speed_to_erpm_offset", 0.0);
    wheelbase_ = declare_parameter("wheelbase", 0.324);
    steering_to_servo_gain_ = declare_parameter("steering_angle_to_servo_gain", -1.2135);
    steering_to_servo_offset_ = declare_parameter("steering_angle_to_servo_offset", 0.1667);
    imu_offset_x_ = this->declare_parameter<double>("imu_offset_x", 0.20); // IMU offset from rear axle
    publish_tf_ = declare_parameter("publish_tf", false);
    use_servo_cmd_ = declare_parameter("use_servo_cmd_to_calc_angular_velocity", true);

    // Publishers and Subscribers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    if (publish_tf_) {
      tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    vesc_state_sub_ = create_subscription<vesc_msgs::msg::VescStateStamped>(
      "sensors/core", 10, std::bind(&VescToMyOdom::vescStateCallback, this, _1));
    
    imu_sub_ = create_subscription<vesc_msgs::msg::VescImuStamped>(
      "sensors/imu", 10, std::bind(&VescToMyOdom::imuCallback, this, _1));

    if (use_servo_cmd_) {
      servo_sub_ = create_subscription<std_msgs::msg::Float64>(
        "sensors/servo_position_command", 10, 
        std::bind(&VescToMyOdom::servoCmdCallback, this, _1));
    }

    last_time_ = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Simple VESC odometry node initialized");
  }

private:
  void vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state) {
    if (!initialized_) return;

    // Get current speed
    double current_speed = (state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
    if (std::fabs(current_speed) < 0.05) {
      current_speed = 0.0;
    }

    // Calculate angular velocity from servo command (Ackermann model)
    double angular_velocity = 0.0;
    if (use_servo_cmd_ && last_servo_cmd_) {
      double steering_angle = (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
      if (std::abs(current_speed) > 0.05 && wheelbase_ > 0.0) {
        angular_velocity = current_speed * tan(steering_angle) / wheelbase_;
      }
    }

    // Time step
    rclcpp::Time current_time = state->header.stamp;
    double dt = (current_time - last_time_).seconds();
    
    if (dt > 0.001 && dt < 1.0) {
      // Use compensated IMU yaw rate for prediction (more accurate than Ackermann model)
      double prediction_omega = compensated_yaw_rate_;
      
      // EKF predict step
      ekf_.predict(dt, current_speed, prediction_omega);
      
      // Get state estimate
      Eigen::Vector3d state_est = ekf_.getState();
      x_ = state_est(0);
      y_ = state_est(1);
      double theta_est = state_est(2);

      // Publish odometry (use Ackermann angular velocity for twist message)
      publishOdometry(state->header.stamp, current_speed, angular_velocity, theta_est);
    }

    last_time_ = current_time;
  }

  void imuCallback(const vesc_msgs::msg::VescImuStamped::SharedPtr imu) {
    double measured_yaw_deg = -imu->imu.ypr.z;
    double measured_yaw_rad = measured_yaw_deg * M_PI / 180.0;

    // Get IMU angular velocity and apply offset compensation
    double imu_yaw_rate = -imu->imu.angular_velocity.z * M_PI / 180.0;
    // Compensate for IMU position offset - more conservative approach
    double compensated_yaw_rate = imu_yaw_rate;

    if (!initialized_) {
      initial_yaw_ = measured_yaw_rad;
      initialized_ = true;
      RCLCPP_INFO(this->get_logger(), "Initial yaw set: %.3f rad (%.1f deg)", 
                  initial_yaw_, measured_yaw_deg);
      return;
    }

    // Correct yaw relative to initial
    double corrected_yaw = measured_yaw_rad - initial_yaw_;
    
    // Normalize
    while (corrected_yaw > M_PI) corrected_yaw -= 2 * M_PI;
    while (corrected_yaw < -M_PI) corrected_yaw += 2 * M_PI;

    // EKF update step
    double R_yaw = 0.01;  // Lower value = higher trust in IMU
    ekf_.update(corrected_yaw, R_yaw);
    
    // Store compensated yaw rate for use in prediction
    compensated_yaw_rate_ = compensated_yaw_rate;
  }

  void servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo) {
    last_servo_cmd_ = servo;
  }

  void publishOdometry(const rclcpp::Time& stamp, double linear_vel, double angular_vel, double theta) {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = base_frame_;

    // Position
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    
    // Orientation (quaternion)
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(theta / 2.0);
    odom_msg.pose.pose.orientation.w = cos(theta / 2.0);

    // Velocity
    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_vel;

    // Simple covariance
    odom_msg.pose.covariance[0] = 0.1;   // x
    odom_msg.pose.covariance[7] = 0.1;   // y
    odom_msg.pose.covariance[35] = 0.2;  // theta

    odom_pub_->publish(odom_msg);

    // Publish TF if enabled
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = x_;
      tf.transform.translation.y = y_;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation = odom_msg.pose.pose.orientation;
      
      tf_pub_->sendTransform(tf);
    }
  }

  // ROS components
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<vesc_msgs::msg::VescStateStamped>::SharedPtr vesc_state_sub_;
  rclcpp::Subscription<vesc_msgs::msg::VescImuStamped>::SharedPtr imu_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;

  // Parameters
  std::string odom_frame_;
  std::string base_frame_;
  double speed_to_erpm_gain_;
  double speed_to_erpm_offset_;
  double wheelbase_;
  double steering_to_servo_gain_;
  double steering_to_servo_offset_;
  double imu_offset_x_;  // IMU offset from rear axle (positive = forward)
  bool publish_tf_;
  bool use_servo_cmd_;

  // State variables
  double x_, y_;
  double initial_yaw_;
  bool initialized_;
  double compensated_yaw_rate_;  // IMU yaw rate compensated for position offset
  std::shared_ptr<std_msgs::msg::Float64> last_servo_cmd_;
  rclcpp::Time last_time_;

  // EKF
  EKF ekf_;
};

} // namespace ekf_odom_estimation

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ekf_odom_estimation::VescToMyOdom)