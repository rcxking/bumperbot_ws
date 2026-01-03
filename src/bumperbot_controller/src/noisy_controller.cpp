#include "bumperbot_controller/noisy_controller.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <random>

using std::placeholders::_1;

NoisyController::NoisyController(const std::string &name) : Node(name),
        left_wheel_prev_pos_(0.0),
        right_wheel_prev_pos_(0.0),
        x_(0.0),
        y_(0.0),
        theta_(0.0) {
  declare_parameter("wheel_radius", 0.033);
  declare_parameter("wheel_separation", 0.17);

  wheel_radius_ = get_parameter("wheel_radius").as_double();
  wheel_separation_ = get_parameter("wheel_separation").as_double();

  RCLCPP_INFO_STREAM(get_logger(), "Using wheel_radius " << wheel_radius_);
  RCLCPP_INFO_STREAM(get_logger(), "Using wheel_separation " << wheel_separation_);

  prev_time_ = get_clock()->now();

  // Gazebo publishes this topic
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&NoisyController::jointCallback, this, _1));

  // Odometry publisher with noise
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom_noisy", 10);

  // Initialize constant odometry message fields
  odom_msg_.header.frame_id = "odom"; // Frame odometry is w.r.t. to
  odom_msg_.child_frame_id = "base_footprint_ekf";

  // Initial orientation needs to be normalized (magnitude = 1)
  odom_msg_.pose.pose.orientation.x = 0.0;
  odom_msg_.pose.pose.orientation.y = 0.0;
  odom_msg_.pose.pose.orientation.z = 0.0;
  odom_msg_.pose.pose.orientation.w = 1.0;

  // TF variables
  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(
      *this);
  transform_stamped_.header.frame_id = "odom";
  transform_stamped_.child_frame_id = "base_footprint_noisy";
}

void NoisyController::jointCallback(const sensor_msgs::msg::JointState &msg) {
  // Gaussian noise to encoders
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine noise_generator(seed);
  std::normal_distribution<double> left_encoder_noise(0.0, 0.005);
  std::normal_distribution<double> right_encoder_noise(0.0, 0.005);

  double wheel_encoder_left = msg.position.at(0) + left_encoder_noise(noise_generator);
  double wheel_encoder_right = msg.position.at(1) + right_encoder_noise(noise_generator);

  // Changes in each wheel's position
  double dp_left = wheel_encoder_left - left_wheel_prev_pos_;
  double dp_right = wheel_encoder_right - right_wheel_prev_pos_;

  rclcpp::Time msg_time = msg.header.stamp;

  // Change in time
  rclcpp::Duration dt = msg_time - prev_time_;

  // Don't update odometry if no time change
  if (dt.seconds() == 0) {
    return;
  }

  // Update last seen variables
  left_wheel_prev_pos_ = msg.position.at(0);
  right_wheel_prev_pos_ = msg.position.at(1);
  prev_time_ = msg_time;

  // Compute velocity of each wheel (rad/s)
  double fi_left  = dp_left / dt.seconds();
  double fi_right = dp_right / dt.seconds();

  // Compute robot's linear/angular velocities
  double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2;
  double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_;

  // Compute incremental robot's position update
  double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2;

  // Compute incremental robot's orientation update
  double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_;

  // Update robot's pose
  theta_ += d_theta;
  x_ += d_s * cos(theta_);
  y_ += d_s * sin(theta_);

  // Update odometry message
  tf2::Quaternion q;
  q.setRPY(0, 0, theta_);
  odom_msg_.pose.pose.orientation.x = q.x();
  odom_msg_.pose.pose.orientation.y = q.y();
  odom_msg_.pose.pose.orientation.z = q.z();
  odom_msg_.pose.pose.orientation.w = q.w();

  odom_msg_.header.stamp = get_clock() ->now();

  odom_msg_.pose.pose.position.x = x_;
  odom_msg_.pose.pose.position.y = y_;

  odom_msg_.twist.twist.linear.x = linear;
  odom_msg_.twist.twist.angular.z = angular;

  // Populate transform fields
  transform_stamped_.transform.translation.x = x_;
  transform_stamped_.transform.translation.y = y_;
  transform_stamped_.transform.rotation.x = q.x();
  transform_stamped_.transform.rotation.y = q.y();
  transform_stamped_.transform.rotation.z = q.z();
  transform_stamped_.transform.rotation.w = q.w();
  transform_stamped_.header.stamp = get_clock()->now();

  // Publish odometry message
  odom_pub_->publish(odom_msg_);

  // Publish transform
  transform_broadcaster_->sendTransform(transform_stamped_);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<NoisyController>("noisy_controller");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
