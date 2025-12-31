#ifndef __SIMPLE_CONTROLLER_HPP__
#define __SIMPLE_CONTROLLER_HPP__

#include <Eigen/Core>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

class SimpleController : public rclcpp::Node {
public:
  SimpleController(const std::string &name);

private:
  void velCallback(const geometry_msgs::msg::TwistStamped &msg);

  void jointCallback(const sensor_msgs::msg::JointState &msg);

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_;

  // Subscriber to the robot's joint states
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  // Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  double wheel_radius_;
  double wheel_separation_;

  Eigen::Matrix2d speed_conversion_;

  // Previous logged time
  rclcpp::Time prev_time_;

  // Previous wheel positions at prev_time_
  double left_wheel_prev_pos_, right_wheel_prev_pos_;

  // Robot's current pose
  double x_, y_, theta_;

  // Robot's current odometry message
  nav_msgs::msg::Odometry odom_msg_;

  // Transform Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  // Tracks robot's transform between odom and base_footprint frames
  geometry_msgs::msg::TransformStamped transform_stamped_;
};

#endif
