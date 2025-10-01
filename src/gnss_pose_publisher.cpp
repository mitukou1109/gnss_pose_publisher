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
  earth_frame_ = declare_parameter<std::string>("earth_frame");
  map_frame_ = declare_parameter<std::string>("map_frame");
  odom_frame_ = declare_parameter<std::string>("odom_frame");
  base_frame_ = declare_parameter<std::string>("base_frame");
  status_threshold_ = declare_parameter<int>("status_threshold");
  transform_tolerance_ = declare_parameter<double>("transform_tolerance");
  tf_publish_rate_ = declare_parameter<double>("tf_publish_rate");

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

  publish_tf_timer_ = create_wall_timer(
    rclcpp::Rate(tf_publish_rate_).period(), std::bind(&GNSSPosePublisher::publishTF, this));
}

void GNSSPosePublisher::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!local_cartesian_) {
    return;
  }

  if (msg->status.status < status_threshold_) {
    return;
  }

  pose_with_covariance_.header = msg->header;

  auto & pose = pose_with_covariance_.pose;
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

  pose_with_covariance_pub_->publish(pose_with_covariance_);
}

void GNSSPosePublisher::headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  pose_with_covariance_.header = msg->header;
  pose_with_covariance_.pose.pose.orientation = msg->quaternion;
  pose_with_covariance_pub_->publish(pose_with_covariance_);
}

void GNSSPosePublisher::publishTF()
{
  const auto stamp = now();
  const auto send_transform = [this, stamp](
                                const tf2::Transform & tf, const std::string & frame_id,
                                const std::string & child_frame_id) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    transform.transform = tf2::toMsg(tf);

    tf_broadcaster_->sendTransform(transform);
  };

  tf2::Transform base_to_earth_tf;
  tf2::fromMsg(pose_with_covariance_.pose.pose, base_to_earth_tf);

  if (odom_frame_.empty()) {
    send_transform(base_to_earth_tf, earth_frame_, base_frame_);
    return;
  }

  const auto odom_to_base_tf = getTransform(base_frame_, odom_frame_, stamp);
  if (!odom_to_base_tf) {
    return;
  }
  const auto odom_to_earth_tf = base_to_earth_tf * *odom_to_base_tf;

  if (map_frame_.empty()) {
    send_transform(odom_to_earth_tf, earth_frame_, odom_frame_);
    return;
  }

  const auto earth_to_map_tf = getTransform(map_frame_, earth_frame_, stamp);
  if (!earth_to_map_tf) {
    return;
  }
  const auto odom_to_map_tf = *earth_to_map_tf * odom_to_earth_tf;
  send_transform(odom_to_map_tf, map_frame_, odom_frame_);
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
      get_logger(), *std::const_pointer_cast<rclcpp::Clock>(get_clock()), 10000, "%s", ex.what());
    return std::nullopt;
  }

  return tf;
}
}  // namespace gnss_pose_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_pose_publisher::GNSSPosePublisher)
