#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "llh_converter/llh_converter.hpp"

namespace gnss_pose_publisher
{
class GNSSPosePublisher : public rclcpp::Node
{
public:
  explicit GNSSPosePublisher(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  GNSSPosePublisher(const std::string& node_name, const std::string& ns,
                    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  void headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_covariance_pub_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr heading_sub_;

  llh_converter::LLHConverter llh_converter_;

  llh_converter::LLHParam llh_param_;

  geometry_msgs::msg::PoseWithCovarianceStamped pose_with_covariance_;
};
}  // namespace gnss_pose_publisher