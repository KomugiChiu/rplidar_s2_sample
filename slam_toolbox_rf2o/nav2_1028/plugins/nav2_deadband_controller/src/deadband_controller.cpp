#include "nav2_deadband_controller/deadband_controller.hpp"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nav2_deadband_controller
{

void DeadbandController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  parent_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("DeadbandController: parent node is expired");
  }

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".primary_controller",
    rclcpp::ParameterValue(std::string("nav2_mppi_controller::MPPIController")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vx_floor", rclcpp::ParameterValue(0.16));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".vy_floor", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".wz_floor", rclcpp::ParameterValue(0.52));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".deadband", rclcpp::ParameterValue(0.005));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".stop_distance", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".stop_half_width", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sonar_topic", rclcpp::ParameterValue(std::string("/ultrasonic")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sonar_stop_distance", rclcpp::ParameterValue(0.35));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".sonar_hold_time", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_start_angle", rclcpp::ParameterValue(0.4));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_stop_angle", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".lookahead_distance", rclcpp::ParameterValue(0.3));

  node->get_parameter(name_ + ".primary_controller", primary_controller_name_);
  node->get_parameter(name_ + ".vx_floor", vx_floor_);
  node->get_parameter(name_ + ".vy_floor", vy_floor_);
  node->get_parameter(name_ + ".wz_floor", wz_floor_);
  node->get_parameter(name_ + ".deadband", deadband_);
  node->get_parameter(name_ + ".stop_distance", stop_distance_);
  node->get_parameter(name_ + ".stop_half_width", stop_half_width_);
  node->get_parameter(name_ + ".sonar_topic", sonar_topic_);
  node->get_parameter(name_ + ".sonar_stop_distance", sonar_stop_distance_);
  node->get_parameter(name_ + ".sonar_hold_time", sonar_hold_time_);
  node->get_parameter(name_ + ".rotate_start_angle", rotate_start_angle_);
  node->get_parameter(name_ + ".rotate_stop_angle", rotate_stop_angle_);
  node->get_parameter(name_ + ".lookahead_distance", lookahead_distance_);

  sonar_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    sonar_topic_, rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {onSonar(msg);});

  loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
    "nav2_core", "nav2_core::Controller");
  primary_controller_ = loader_->createSharedInstance(primary_controller_name_);
  primary_controller_->configure(parent, name_, tf_, costmap_ros_);

  RCLCPP_INFO(
    node->get_logger(),
    "DeadbandController configured: primary=%s vx_floor=%.3f wz_floor=%.3f "
    "stop=%.2fm half_width=%.2fm sonar=%s <%.2fm",
    primary_controller_name_.c_str(), vx_floor_, wz_floor_,
    stop_distance_, stop_half_width_, sonar_topic_.c_str(), sonar_stop_distance_);
}

void DeadbandController::cleanup()
{
  if (primary_controller_) {
    primary_controller_->cleanup();
  }
  sonar_sub_.reset();
  primary_controller_.reset();
  loader_.reset();
}

void DeadbandController::activate()
{
  if (primary_controller_) {
    primary_controller_->activate();
  }
}

void DeadbandController::deactivate()
{
  if (primary_controller_) {
    primary_controller_->deactivate();
  }
}

void DeadbandController::setPlan(const nav_msgs::msg::Path & path)
{
  path_ = path;
  if (primary_controller_) {
    primary_controller_->setPlan(path);
  }
}

double DeadbandController::floorAxis(double value, double floor) const
{
  if (floor <= 0.0) {
    return value;
  }
  if (std::abs(value) <= deadband_) {
    return 0.0;
  }
  if (std::abs(value) < floor) {
    return std::copysign(floor, value);
  }
  return value;
}

void DeadbandController::onSonar(const std_msgs::msg::Float32::SharedPtr msg)
{
  const double d = static_cast<double>(msg->data) * 0.01;
  if (d < 0.02 || d > 3.0) {
    return;
  }
  sonar_dist_ = d;
  if (d < sonar_stop_distance_) {
    sonar_blocked_until_ = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(static_cast<int>(sonar_hold_time_ * 1000.0));
    const auto now = std::chrono::steady_clock::now();
    if (now - last_sonar_log_ > std::chrono::seconds(1)) {
      last_sonar_log_ = now;
      auto node = parent_.lock();
      if (node) {
        RCLCPP_WARN(
          node->get_logger(),
          "ultrasonic stop: %.2f m < %.2f m (forward only)",
          d, sonar_stop_distance_);
      }
    }
  }
}

bool DeadbandController::forwardBlocked(const geometry_msgs::msg::PoseStamped & pose)
{
  if (std::chrono::steady_clock::now() < sonar_blocked_until_) {
    return true;
  }
  if (!costmap_ros_) {
    return false;
  }
  auto * costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    return false;
  }

  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  for (double dx = 0.05; dx <= stop_distance_ + 1e-6; dx += 0.05) {
    for (double dy = -stop_half_width_; dy <= stop_half_width_ + 1e-6; dy += 0.05) {
      const double wx = pose.pose.position.x + dx * cy - dy * sy;
      const double wy = pose.pose.position.y + dx * sy + dy * cy;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(wx, wy, mx, my)) {
        continue;
      }
      const unsigned char cost = costmap->getCost(mx, my);
      if (cost == nav2_costmap_2d::LETHAL_OBSTACLE ||
        cost == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        return true;
      }
    }
  }
  return false;
}

double DeadbandController::headingError(const geometry_msgs::msg::PoseStamped & pose)
{
  if (path_.poses.empty()) {
    return 0.0;
  }
  const auto & rp = pose.pose.position;

  size_t closest = 0;
  double best = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path_.poses.size(); ++i) {
    const auto & p = path_.poses[i].pose.position;
    const double dx = p.x - rp.x;
    const double dy = p.y - rp.y;
    const double d = dx * dx + dy * dy;
    if (d < best) {
      best = d;
      closest = i;
    }
  }

  size_t idx = closest;
  double accum = 0.0;
  while (idx + 1 < path_.poses.size() && accum < lookahead_distance_) {
    const auto & a = path_.poses[idx].pose.position;
    const auto & b = path_.poses[idx + 1].pose.position;
    accum += std::hypot(b.x - a.x, b.y - a.y);
    ++idx;
  }

  geometry_msgs::msg::PoseStamped target = path_.poses[idx];
  const std::string target_frame = pose.header.frame_id;
  if (tf_ && !path_.header.frame_id.empty() && !target_frame.empty() &&
    path_.header.frame_id != target_frame)
  {
    try {
      geometry_msgs::msg::PoseStamped out;
      tf_->transform(target, out, target_frame, tf2::durationFromSec(0.1));
      target = out;
    } catch (const std::exception &) {
      return 0.0;
    }
  }

  const double dx = target.pose.position.x - rp.x;
  const double dy = target.pose.position.y - rp.y;
  if (std::abs(dx) + std::abs(dy) < 1e-6) {
    return 0.0;
  }
  const double bearing = std::atan2(dy, dx);
  const double yaw = tf2::getYaw(pose.pose.orientation);
  return std::atan2(std::sin(bearing - yaw), std::cos(bearing - yaw));
}

geometry_msgs::msg::TwistStamped DeadbandController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  if (!primary_controller_) {
    throw std::runtime_error("DeadbandController: primary controller is not configured");
  }

  auto cmd = primary_controller_->computeVelocityCommands(pose, velocity, goal_checker);

  // Directional stop: only forward motion is blocked, so the robot can still
  // rotate or reverse out of a tight spot.
  if (cmd.twist.linear.x > 0.0 && forwardBlocked(pose)) {
    cmd.twist.linear.x = 0.0;
  }

  cmd.twist.linear.x = floorAxis(cmd.twist.linear.x, vx_floor_);
  cmd.twist.linear.y = floorAxis(cmd.twist.linear.y, vy_floor_);

  // Turn only when the heading error is large; once aligned, stop turning.
  // This avoids the wz floor amplifying every small MPPI correction.
  const double err = headingError(pose);
  if (!rotating_ && std::abs(err) > rotate_start_angle_) {
    rotating_ = true;
  }
  if (rotating_ && std::abs(err) < rotate_stop_angle_) {
    rotating_ = false;
  }
  if (rotating_) {
    cmd.twist.linear.x = 0.0;
    cmd.twist.linear.y = 0.0;
    cmd.twist.angular.z = std::copysign(wz_floor_, err);
  } else {
    cmd.twist.angular.z = 0.0;
  }

  return cmd;
}

bool DeadbandController::cancel()
{
  if (primary_controller_) {
    return primary_controller_->cancel();
  }
  return true;
}

void DeadbandController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (primary_controller_) {
    primary_controller_->setSpeedLimit(speed_limit, percentage);
  }
}

void DeadbandController::reset()
{
  if (primary_controller_) {
    primary_controller_->reset();
  }
}

}  // namespace nav2_deadband_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_deadband_controller::DeadbandController,
  nav2_core::Controller)
