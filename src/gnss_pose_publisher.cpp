#include "gnss_pose_publisher/gnss_pose_publisher.hpp"

#include <rclcpp_components/register_node_macro.hpp>

namespace gnss_pose_publisher
{
GNSSPosePublisher::GNSSPosePublisher(const rclcpp::NodeOptions& options)
  : GNSSPosePublisher("gnss_pose_publisher", "", options)
{
}

GNSSPosePublisher::GNSSPosePublisher(const std::string& node_name, const std::string& ns,
                                     const rclcpp::NodeOptions& options)
  : Node(node_name, ns, options)
{
  pose_with_covariance_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss_pose_with_covariance", 1);

  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  llh_param_.use_mgrs = true;
  // llh_param_.mgrs_code = "54SVF";  // Tochigi / Ibaraki (Not required for converting llh to xyz)
  llh_param_.height_convert_type = llh_converter::ConvertType::ELLIPS2ORTHO;
  llh_param_.geoid_type = llh_converter::GeoidType::GSIGEO2011;

  fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "fix", 10, std::bind(&GNSSPosePublisher::fixCallback, this, std::placeholders::_1));

  heading_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "heading", 10, std::bind(&GNSSPosePublisher::headingCallback, this, std::placeholders::_1));

  initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10, std::bind(&GNSSPosePublisher::initialposeCallback, this, std::placeholders::_1));
}

void GNSSPosePublisher::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  pose_with_covariance_.header = msg->header;

  llh_converter_.convertDeg2XYZ(msg->latitude, msg->longitude, msg->altitude,
                                pose_with_covariance_.pose.pose.position.x, pose_with_covariance_.pose.pose.position.y,
                                pose_with_covariance_.pose.pose.position.z, llh_param_);

  for (auto i = 0; i < 3; i++)
  {
    pose_with_covariance_.pose.covariance[i * 6 + i] = msg->position_covariance[i * 3 + i];
  }

  pose_with_covariance_pub_->publish(pose_with_covariance_);
}

void GNSSPosePublisher::headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  pose_with_covariance_.header = msg->header;

  pose_with_covariance_.pose.pose.orientation = msg->quaternion;

  pose_with_covariance_pub_->publish(pose_with_covariance_);
}

void GNSSPosePublisher::initialposeCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped map_to_gnss_tf;
  map_to_gnss_tf.header.stamp = msg->header.stamp;
  map_to_gnss_tf.header.frame_id = msg->header.frame_id;
  map_to_gnss_tf.child_frame_id = pose_with_covariance_.header.frame_id;

  map_to_gnss_tf.transform.translation.x = -pose_with_covariance_.pose.pose.position.x;
  map_to_gnss_tf.transform.translation.y = -pose_with_covariance_.pose.pose.position.y;
  map_to_gnss_tf.transform.translation.z = -pose_with_covariance_.pose.pose.position.z;
  map_to_gnss_tf.transform.rotation.x = -pose_with_covariance_.pose.pose.orientation.x;
  map_to_gnss_tf.transform.rotation.y = -pose_with_covariance_.pose.pose.orientation.y;
  map_to_gnss_tf.transform.rotation.z = -pose_with_covariance_.pose.pose.orientation.z;
  map_to_gnss_tf.transform.rotation.w = -pose_with_covariance_.pose.pose.orientation.w;

  static_tf_broadcaster_->sendTransform(map_to_gnss_tf);
}
}  // namespace gnss_pose_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_pose_publisher::GNSSPosePublisher)