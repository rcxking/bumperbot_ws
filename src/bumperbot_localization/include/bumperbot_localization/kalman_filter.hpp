/*
 * kalman_filter.hpp
 *
 * Class declaration of a mono-dimension Kalman Filter.
 */
#ifndef __KALMAN_FILTER_HPP__
#define __KALMAN_FILTER_HPP__

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class KalmanFilter : public rclcpp::Node {
public:
  /*
   * Constructor.
   *
   * Parameters:
   */
  KalmanFilter(const std::string &name);

private:
  // Subscriber to robot's odometry
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Subscriber to the IMU data
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Publisher of fused/filtered odometry
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  // Current robot's angular velocity mean/variance
  double mean_, variance_;

  // Last measured IMU's Z angular velocity
  double imu_angular_z_;

  // Is this the wheel odometry message received?
  bool is_first_odom_;

  // Last estimation of the angular velocity from wheel encoders (odom)
  double last_angular_z_;

  // Difference between 2 consecutive angular velocity moments
  double motion_;

  // Filtered odometry message (output from this Kalman Filter)
  nav_msgs::msg::Odometry kalman_odom_;

  // Variance of the robot's motion
  double motion_variance_;

  // Variance of the IMU sensor measurement
  double measurement_variance_;

  // Update the robot's odometry with new sensor data
  void measurementUpdate();

  void statePrediction();

  /*
   * New odometry data callback.
   *
   * Parameters:
   *   odom: Next odom data
   */
  void odomCallback(const nav_msgs::msg::Odometry &odom);

  /*
   * New IMU data callback.
   *
   * Parameters:
   */
  void imuCallback(const sensor_msgs::msg::Imu & imu);
};

#endif
