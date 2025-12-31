/*
 * trajectory_drawer.cpp
 *
 * ROS 2 node that subscribes to a provided odometry topic and
 * maintains/publishes a path using the odometry data.
 */
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

using std::placeholders::_1;

class TrajectoryDrawer : public rclcpp::Node {
public:
  /*
   * Constructor
   *
   * Parameters:
   *   name (std::string): Name of this node
   */
  TrajectoryDrawer(const std::string &name) : Node(name) {
    // Get the odometry topic
    declare_parameter("odom_topic", "/bumperbot_controller/odom");
    const std::string odom_topic(get_parameter("odom_topic").as_string());
    RCLCPP_INFO_STREAM(get_logger(), "Using odom topic: " << odom_topic);

    // Initialize path publisher
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/bumperbot_controller/trajectory", 10);

    // Initialize odom subscriber
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, 10,
        std::bind(&TrajectoryDrawer::odomCallback, this, _1));

    // Path waypoints are w.r.t. odom frame
    path_.header.frame_id = "odom";
  }

private:
  // Subscriber to the odometry topic
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Path publisher
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // Current path
  nav_msgs::msg::Path path_;

  /*
   * Callback to add the next odometry data to the path.
   *
   * Parameters:
   *  msg (nav_msgs::msg::Odometry): Incoming odometry message
   */
  void odomCallback(const nav_msgs::msg::Odometry &msg) {
    // Add the next waypoint to the path
    geometry_msgs::msg::PoseStamped next_pose;
    next_pose.header.stamp = msg.header.stamp;
    next_pose.header.frame_id = msg.header.frame_id;
    next_pose.pose.position.x = msg.pose.pose.position.x;
    next_pose.pose.position.y = msg.pose.pose.position.y;
    next_pose.pose.orientation.x = msg.pose.pose.orientation.x;
    next_pose.pose.orientation.y = msg.pose.pose.orientation.y;
    next_pose.pose.orientation.z = msg.pose.pose.orientation.z;
    next_pose.pose.orientation.w = msg.pose.pose.orientation.w;
    path_.poses.push_back(next_pose);

    // Update path timestamp
    path_.header.stamp = get_clock()->now();

    // Publish next point
    path_pub_->publish(path_);
  }
};

int main(int argc, char *argv[]) {
  // Initialize ROS 2 and start TrajectoryDrawer node
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TrajectoryDrawer>("trajectory_drawer");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
