#pragma once

#include <GeographicLib/LocalCartesian.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace gnss_pose_publisher
{
class GNSSPosePublisher : public rclcpp::Node
{
public:
  explicit GNSSPosePublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  GNSSPosePublisher(
    const std::string & node_name, const std::string & ns,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

  void publishTF();

  void initializeLocalCartesian(const std::string & origin_grid);

  std::optional<tf2::Transform> getTransform(
    const std::string & target_frame, const std::string & source_frame,
    const rclcpp::Time & stamp) const;

  std::string enu_frame_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string gnss_frame_;
  int8_t status_threshold_;
  double transform_tolerance_;
  double tf_publish_rate_;

  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose_with_covariance_;

  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_with_covariance_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;

  rclcpp::TimerBase::SharedPtr publish_tf_timer_;
};
}  // namespace gnss_pose_publisher
