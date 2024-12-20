#include "gnss_pose_publisher/gnss_pose_publisher.hpp"

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>

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
  global_frame_ = declare_parameter<std::string>("global_frame");
  base_frame_ = declare_parameter<std::string>("base_frame");
  const auto publish_rate = declare_parameter<double>("publish_rate");

  initializeLocalCartesian(declare_parameter<std::string>("local_cartesian_origin_grid"));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  pose_with_covariance_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("gnss/pose_with_covariance", 1);

  set_origin_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/set_origin", std::bind(&GNSSPosePublisher::setOrigin, this, std::placeholders::_1, std::placeholders::_2));

  fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "gnss/fix", 10, std::bind(&GNSSPosePublisher::fixCallback, this, std::placeholders::_1));

  heading_sub_ = create_subscription<geometry_msgs::msg::QuaternionStamped>(
      "gnss/heading", 10, std::bind(&GNSSPosePublisher::headingCallback, this, std::placeholders::_1));

  publish_tf_timer_ =
      create_wall_timer(rclcpp::Rate(publish_rate).period(), std::bind(&GNSSPosePublisher::publishTF, this));
}

void GNSSPosePublisher::fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (!local_cartesian_)
  {
    return;
  }

  pose_with_covariance_.header = msg->header;
  auto& pose = pose_with_covariance_.pose;

  try
  {
    local_cartesian_->Forward(msg->latitude, msg->longitude, msg->altitude, pose.pose.position.x, pose.pose.position.y,
                              pose.pose.position.z);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert LLA to XYZ: %s", e.what());
    return;
  }

  for (auto i = 0; i < 3; i++)
  {
    pose.covariance[i * 6 + i] = msg->position_covariance[i * 3 + i];
  }

  position_initialized_ = true;
}

void GNSSPosePublisher::headingCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
{
  pose_with_covariance_.pose.pose.orientation = msg->quaternion;
  orientation_initialized_ = true;
}

void GNSSPosePublisher::setOrigin(const std_srvs::srv::Trigger::Request::SharedPtr /* request */,
                                  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (!position_initialized_ || !orientation_initialized_)
  {
    response->success = false;
    response->message = "Position or orientation is not initialized yet.";
    return;
  }

  tf2::Transform base_to_global_tf;
  try
  {
    tf2::fromMsg(tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero).transform,
                 base_to_global_tf);
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(this->get_logger(), ex.what());
    return;
  }

  if (!gnss_to_global_tf_)
  {
    gnss_to_global_tf_.emplace();
    gnss_to_global_tf_->header.frame_id = global_frame_;
  }

  tf2::Transform base_to_gnss_tf;
  tf2::fromMsg(pose_with_covariance_.pose.pose, base_to_gnss_tf);

  gnss_to_global_tf_->child_frame_id = pose_with_covariance_.header.frame_id;
  tf2::toMsg(base_to_global_tf * base_to_gnss_tf.inverse(), gnss_to_global_tf_->transform);
}

void GNSSPosePublisher::publishTF()
{
  if (gnss_to_global_tf_)
  {
    gnss_to_global_tf_->header.stamp = now();
    tf_broadcaster_->sendTransform(*gnss_to_global_tf_);
  }

  if (position_initialized_ && orientation_initialized_)
  {
    pose_with_covariance_.header.stamp = now();
    pose_with_covariance_pub_->publish(pose_with_covariance_);
  }
}

void GNSSPosePublisher::initializeLocalCartesian(const std::string& origin_grid)
{
  try
  {
    int zone;
    bool northp;
    double x, y;
    int prec;
    GeographicLib::MGRS::Reverse(origin_grid, zone, northp, x, y, prec);

    double lat, lon;
    double gamma, k;
    GeographicLib::UTMUPS::Reverse(zone, northp, x, y, lat, lon, gamma, k);

    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(lat, lon);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize local cartesian origin: %s", e.what());
  }
}
}  // namespace gnss_pose_publisher

RCLCPP_COMPONENTS_REGISTER_NODE(gnss_pose_publisher::GNSSPosePublisher)