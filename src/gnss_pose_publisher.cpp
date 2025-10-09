#include "gnss_pose_publisher/gnss_pose_publisher.hpp"

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace gnss_pose_publisher
{
GNSSPosePublisher::GNSSPosePublisher(const rclcpp::NodeOptions & options)
: GNSSPosePublisher("gnss_pose_publisher", "", options)
{
}

GNSSPosePublisher::GNSSPosePublisher(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
{
  mgrs_frame_ = declare_parameter<std::string>("mgrs_frame");
  map_frame_ = declare_parameter<std::string>("map_frame");
  odom_frame_ = declare_parameter<std::string>("odom_frame");
  gnss_frame_ = declare_parameter<std::string>("gnss_frame");
  gnss_enu_frame_ = declare_parameter<std::string>("gnss_enu_frame");
  status_threshold_ = declare_parameter<int>("status_threshold");
  transform_tolerance_ = declare_parameter<double>("transform_tolerance");

  initializeLocalCartesian(declare_parameter<std::string>("local_cartesian_origin_grid"));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_with_covariance_pub_ =
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss/pose_with_covariance", 1);

  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "gnss/fix", 1, std::bind(&GNSSPosePublisher::fixCallback, this, std::placeholders::_1));

  heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
    "gnss/heading", 1, std::bind(&GNSSPosePublisher::headingCallback, this, std::placeholders::_1));
}

void GNSSPosePublisher::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!local_cartesian_) {
    return;
  }

  if (msg->status.status < status_threshold_) {
    return;
  }

  if (!pose_with_covariance_) {
    pose_with_covariance_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pose_with_covariance_->header.frame_id = mgrs_frame_;
  }

  pose_with_covariance_->header.stamp = msg->header.stamp;

  auto & pose = pose_with_covariance_->pose;
  try {
    local_cartesian_->Forward(
      msg->latitude, msg->longitude, msg->altitude, pose.pose.position.x, pose.pose.position.y,
      pose.pose.position.z);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to convert LLA to XYZ: %s", e.what());
    return;
  }

  for (auto i = 0; i < 3; i++) {
    pose.covariance[i * 6 + i] = msg->position_covariance[i * 3 + i];
  }

  publishPose();
}

void GNSSPosePublisher::headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  if (!pose_with_covariance_) {
    return;
  }

  if (
    std::isnan(msg->quaternion.x) || std::isnan(msg->quaternion.y) ||
    std::isnan(msg->quaternion.z) || std::isnan(msg->quaternion.w)) {
    return;
  }

  tf2::Quaternion heading_quat;
  tf2::fromMsg(msg->quaternion, heading_quat);
  heading_quat = tf2::Quaternion({0, 0, 1}, M_PI_2) * heading_quat.inverse();

  pose_with_covariance_->header.stamp = msg->header.stamp;
  pose_with_covariance_->pose.pose.orientation = tf2::toMsg(heading_quat);

  publishPose();
  sendTransform(tf2::Transform(heading_quat), msg->header.stamp, gnss_frame_, gnss_enu_frame_);
}

void GNSSPosePublisher::publishPose()
{
  if (!pose_with_covariance_) {
    return;
  }

  pose_with_covariance_pub_->publish(*pose_with_covariance_);

  tf2::Transform gnss_to_mgrs_tf;
  tf2::fromMsg(pose_with_covariance_->pose.pose, gnss_to_mgrs_tf);

  const auto odom_to_gnss_tf =
    getTransform(gnss_frame_, odom_frame_, pose_with_covariance_->header.stamp);
  if (!odom_to_gnss_tf) {
    return;
  }
  const auto odom_to_mgrs_tf = gnss_to_mgrs_tf * *odom_to_gnss_tf;

  const auto mgrs_to_map_tf =
    getTransform(map_frame_, mgrs_frame_, pose_with_covariance_->header.stamp);
  if (!mgrs_to_map_tf) {
    return;
  }
  const auto odom_to_map_tf = *mgrs_to_map_tf * odom_to_mgrs_tf;

  sendTransform(odom_to_map_tf, pose_with_covariance_->header.stamp, map_frame_, odom_frame_);
}

void GNSSPosePublisher::initializeLocalCartesian(const std::string & origin_grid)
{
  try {
    int zone;
    bool northp;
    double x, y;
    int prec;
    GeographicLib::MGRS::Reverse(origin_grid, zone, northp, x, y, prec);

    double lat, lon;
    double gamma, k;
    GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon, gamma, k);

    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(lat, lon);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize local cartesian origin: %s", e.what());
    rclcpp::shutdown();
  }
}

void GNSSPosePublisher::sendTransform(
  const tf2::Transform & tf, const rclcpp::Time & stamp, const std::string & frame_id,
  const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = frame_id;
  transform.child_frame_id = child_frame_id;
  transform.transform = tf2::toMsg(tf);

  tf_broadcaster_->sendTransform(transform);
}

std::optional<tf2::Transform> GNSSPosePublisher::getTransform(
  const std::string & source_frame, const std::string & target_frame,
  const rclcpp::Time & stamp) const
{
  tf2::Transform tf;

  try {
    tf2::fromMsg(
      tf_buffer_
        ->lookupTransform(
          source_frame, target_frame, stamp, rclcpp::Duration::from_seconds(transform_tolerance_))
        .transform,
      tf);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 10000, ex.what());
    return std::nullopt;
  }

  return tf;
}
}  // namespace gnss_pose_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_pose_publisher::GNSSPosePublisher)
