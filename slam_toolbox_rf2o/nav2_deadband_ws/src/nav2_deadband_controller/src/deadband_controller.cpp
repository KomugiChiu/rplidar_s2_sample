#include "nav2_deadband_controller/deadband_controller.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

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
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".goal_rotate_distance", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".path_min_distance", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cmd_vel_timeout", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".mode", rclcpp::ParameterValue(std::string("simple")));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".drive_speed", rclcpp::ParameterValue(0.14));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steer_gain", rclcpp::ParameterValue(0.8));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".steer_max", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".align_angle", rclcpp::ParameterValue(0.6));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".reverse_distance", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".brake_distance", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".xy_retry_distance", rclcpp::ParameterValue(0.12));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".yaw_tolerance", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rotate_pulse", rclcpp::ParameterValue(0.45));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".settle_time", rclcpp::ParameterValue(0.70));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".ff_max_distance", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".ff_dead_time", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".ff_speed", rclcpp::ParameterValue(0.40));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".cruise_stop_distance", rclcpp::ParameterValue(0.20));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".near_goal_distance", rclcpp::ParameterValue(0.15));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".adapt_gain", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rot_dead_time", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".rot_rate", rclcpp::ParameterValue(2.0));

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
  node->get_parameter(name_ + ".goal_rotate_distance", goal_rotate_distance_);
  node->get_parameter(name_ + ".path_min_distance", path_min_distance_);
  node->get_parameter(name_ + ".cmd_vel_timeout", cmd_vel_timeout_);
  node->get_parameter(name_ + ".mode", mode_);
  node->get_parameter(name_ + ".drive_speed", drive_speed_);
  node->get_parameter(name_ + ".steer_gain", steer_gain_);
  node->get_parameter(name_ + ".steer_max", steer_max_);
  node->get_parameter(name_ + ".align_angle", align_angle_);
  node->get_parameter(name_ + ".reverse_distance", reverse_distance_);
  node->get_parameter(name_ + ".brake_distance", brake_distance_);
  node->get_parameter(name_ + ".xy_retry_distance", xy_retry_distance_);
  node->get_parameter(name_ + ".yaw_tolerance", yaw_tolerance_);
  node->get_parameter(name_ + ".rotate_pulse", rotate_pulse_);
  node->get_parameter(name_ + ".settle_time", settle_time_);
  node->get_parameter(name_ + ".ff_max_distance", ff_max_distance_);
  node->get_parameter(name_ + ".ff_dead_time", ff_dead_time_);
  node->get_parameter(name_ + ".ff_speed", ff_speed_);
  node->get_parameter(name_ + ".cruise_stop_distance", cruise_stop_distance_);
  node->get_parameter(name_ + ".near_goal_distance", near_goal_distance_);
  node->get_parameter(name_ + ".adapt_gain", adapt_gain_);
  node->get_parameter(name_ + ".rot_dead_time", rot_dead_time_);
  node->get_parameter(name_ + ".rot_rate", rot_rate_);
  if (mode_ != "simple" && mode_ != "mppi") {
    throw std::runtime_error("DeadbandController: mode must be 'simple' or 'mppi'");
  }

  sonar_sub_ = node->create_subscription<std_msgs::msg::Float32>(
    sonar_topic_, rclcpp::QoS(10),
    [this](const std_msgs::msg::Float32::SharedPtr msg) {onSonar(msg);});

  // The chassis MCU latches the last motor command.  Publish zero when the
  // controller is no longer being called (goal finished, Nav2 stopped, ...).
  last_cmd_time_ = std::chrono::steady_clock::now();
  stop_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::QoS(1));
  stop_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(100), [this]() {stopTimer();});

  if (mode_ == "mppi") {
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    primary_controller_ = loader_->createSharedInstance(primary_controller_name_);
    primary_controller_->configure(parent, name_, tf_, costmap_ros_);
  }

  RCLCPP_INFO(
    node->get_logger(),
    "DeadbandController configured: mode=%s primary=%s vx_floor=%.3f wz_floor=%.3f "
    "drive=%.2f brake=%.2fm yaw_tol=%.2f stop=%.2fm half_width=%.2fm sonar=%s <%.2fm",
    mode_.c_str(), mode_ == "mppi" ? primary_controller_name_.c_str() : "-",
    vx_floor_, wz_floor_, drive_speed_, brake_distance_, yaw_tolerance_,
    stop_distance_, stop_half_width_, sonar_topic_.c_str(), sonar_stop_distance_);
}

void DeadbandController::cleanup()
{
  if (primary_controller_) {
    primary_controller_->cleanup();
  }
  sonar_sub_.reset();
  stop_timer_.reset();
  stop_pub_.reset();
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
  // Downsample the planner path (0.025 m spacing by default) before handing
  // it to MPPI.  Keep the first and last pose.
  nav_msgs::msg::Path filtered;
  filtered.header = path.header;
  if (path.poses.empty()) {
    path_ = filtered;
  } else if (path_min_distance_ <= 0.0) {
    path_ = path;
  } else {
    filtered.poses.push_back(path.poses.front());
    double accum = 0.0;
    for (size_t i = 1; i < path.poses.size(); ++i) {
      const auto & a = path.poses[i - 1].pose.position;
      const auto & b = path.poses[i].pose.position;
      accum += std::hypot(b.x - a.x, b.y - a.y);
      if (accum >= path_min_distance_ || i == path.poses.size() - 1) {
        filtered.poses.push_back(path.poses[i]);
        accum = 0.0;
      }
    }
    path_ = filtered;
  }

  if (primary_controller_) {
    primary_controller_->setPlan(path_);
  }

  // The BT replans at 1 Hz; only a new goal may clear the ARRIVED latch.
  if (!path_.poses.empty()) {
    const auto & g = path_.poses.back();
    bool new_goal = !have_goal_ || g.header.frame_id != last_goal_.header.frame_id;
    if (!new_goal) {
      const double d = std::hypot(
        g.pose.position.x - last_goal_.pose.position.x,
        g.pose.position.y - last_goal_.pose.position.y);
      const double a = std::abs(std::remainder(
        tf2::getYaw(g.pose.orientation) - tf2::getYaw(last_goal_.pose.orientation), 2.0 * M_PI));
      new_goal = d > 0.02 || a > 0.05;
    }
    if (new_goal) {
      phase_ = Phase::DRIVE;
      pulse_until_ = {};
      pulse_kind_ = PulseKind::NONE;
      settle_until_ = {};
      was_moving_ = false;
    }
    last_goal_ = g;
    last_goal_.header = path_.header;
    have_goal_ = true;
  }
  auto node = parent_.lock();
  if (node) {
    RCLCPP_INFO(
      node->get_logger(), "path downsampled %zu -> %zu (min_dist=%.2f m)",
      path.poses.size(), path_.poses.size(), path_min_distance_);
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

  // Only real (lethal) obstacle cells inside the swept footprint ahead.
  // Counting INSCRIBED cells with a box wider than the robot stopped it on
  // paths the planner considers free (e.g. 10-15 cm from a wall).
  const double step = costmap->getResolution() * 0.5;
  for (double dx = 0.05; dx <= stop_distance_ + 1e-6; dx += step) {
    for (double dy = -stop_half_width_; dy <= stop_half_width_ + 1e-6; dy += step) {
      const double wx = pose.pose.position.x + dx * cy - dy * sy;
      const double wy = pose.pose.position.y + dx * sy + dy * cy;
      unsigned int mx = 0;
      unsigned int my = 0;
      if (!costmap->worldToMap(wx, wy, mx, my)) {
        continue;
      }
      if (costmap->getCost(mx, my) == nav2_costmap_2d::LETHAL_OBSTACLE) {
        auto node = parent_.lock();
        if (node) {
          RCLCPP_WARN_THROTTLE(
            node->get_logger(), *node->get_clock(), 1000,
            "costmap stop: obstacle %.2f m ahead, %+.2f m lateral (forward only)", dx, dy);
        }
        return true;
      }
    }
  }
  return false;
}

double DeadbandController::goalDistance(const geometry_msgs::msg::PoseStamped & pose)
{
  if (path_.poses.empty()) {
    return 0.0;
  }
  geometry_msgs::msg::PoseStamped target = path_.poses.back();
  const std::string target_frame = pose.header.frame_id;
  if (tf_ && !path_.header.frame_id.empty() && !target_frame.empty() &&
    path_.header.frame_id != target_frame)
  {
    try {
      geometry_msgs::msg::PoseStamped out;
      tf_->transform(target, out, target_frame, tf2::durationFromSec(0.1));
      target = out;
    } catch (const std::exception &) {
      // Do not treat a TF failure as "at the goal"; keep translating.
      return std::numeric_limits<double>::max();
    }
  }
  const double dx = target.pose.position.x - pose.pose.position.x;
  const double dy = target.pose.position.y - pose.pose.position.y;
  return std::hypot(dx, dy);
}

double DeadbandController::goalYawError(const geometry_msgs::msg::PoseStamped & pose)
{
  if (path_.poses.empty()) {
    return 0.0;
  }
  geometry_msgs::msg::PoseStamped target = path_.poses.back();
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
  const double goal_yaw = tf2::getYaw(target.pose.orientation);
  const double yaw = tf2::getYaw(pose.pose.orientation);
  return std::atan2(std::sin(goal_yaw - yaw), std::cos(goal_yaw - yaw));
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
  last_cmd_time_ = std::chrono::steady_clock::now();

  if (mode_ == "simple") {
    auto cmd = computeSimple(pose);
    if (cmd.twist.linear.x > 0.0 && forwardBlocked(pose)) {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;
      pulse_until_ = {};
      pulse_kind_ = PulseKind::NONE;
      was_moving_ = false;
    }
    return cmd;
  }

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

  // Translation-only until close to the goal; rotate to the goal heading
  // only inside goal_rotate_distance (default 0.30 m).
  const double goal_dist = goalDistance(pose);
  if (goal_dist > goal_rotate_distance_) {
    rotating_ = false;
    cmd.twist.angular.z = 0.0;
  } else {
    const double err = goalYawError(pose);
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
  }

  return cmd;
}

bool DeadbandController::transformToPoseFrame(
  const geometry_msgs::msg::PoseStamped & in, const std::string & frame,
  geometry_msgs::msg::PoseStamped & out)
{
  out = in;
  if (in.header.frame_id.empty() || frame.empty() || in.header.frame_id == frame) {
    return true;
  }
  try {
    // Use the latest transform: the plan stamp may be older than the TF buffer.
    geometry_msgs::msg::PoseStamped src = in;
    src.header.stamp = rclcpp::Time(0);
    tf_->transform(src, out, frame, tf2::durationFromSec(0.1));
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

geometry_msgs::msg::TwistStamped DeadbandController::computeSimple(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = pose.header.stamp;
  cmd.header.frame_id = costmap_ros_ ? costmap_ros_->getBaseFrameID() : "base_link";

  auto node = parent_.lock();
  const auto now = std::chrono::steady_clock::now();
  const auto secs = [](double s) {
      return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(s));
    };
  const auto wrap = [](double a) {return std::atan2(std::sin(a), std::cos(a));};

  // Pulses (timed drive / turn-in-place) are open loop: finish, then settle.
  if (now < pulse_until_) {
    cmd.twist.linear.x = pulse_vx_;
    cmd.twist.angular.z = pulse_wz_;
    return cmd;
  }
  if (pulse_until_ != std::chrono::steady_clock::time_point{}) {
    pulse_until_ = {};
    settle_until_ = now + secs(settle_time_);
  }
  // After any stop, wait for the chassis and the (lagging) odometry to settle
  // before deciding again; this is what prevents stop/go oscillation.
  if (now < settle_until_) {
    return cmd;
  }
  if (pulse_kind_ != PulseKind::NONE) {
    // Learn from the pulse that just settled.  Stalls (no motion) and
    // outliers (rf2o mis-tracks during fast spins, e.g. "0.38 m in 0.4 s")
    // are not speed measurements and are ignored.
    const double y0 = tf2::getYaw(pulse_start_.pose.orientation);
    if (adapt_gain_ > 0.0 && pose.header.frame_id == pulse_start_.header.frame_id) {
      if (pulse_kind_ == PulseKind::DRIVE) {
        const double dx = pose.pose.position.x - pulse_start_.pose.position.x;
        const double dy = pose.pose.position.y - pulse_start_.pose.position.y;
        const double moved = std::abs(dx * std::cos(y0) + dy * std::sin(y0));
        const double t = pulse_dur_ - ff_dead_time_;
        const double obs = t > 0.05 ? moved / t : 0.0;
        if (moved > 0.02 && obs > 0.6 * ff_speed_ && obs < 1.7 * ff_speed_) {
          const double prev = ff_speed_;
          ff_speed_ = std::clamp((1.0 - adapt_gain_) * ff_speed_ + adapt_gain_ * obs, 0.2, 1.5);
          if (node) {
            RCLCPP_INFO(
              node->get_logger(), "simple: drive pulse %.2f s moved %.3f m -> ff_speed %.3f -> %.3f",
              pulse_dur_, moved, prev, ff_speed_);
          }
        }
      } else {
        const double turned = std::abs(wrap(tf2::getYaw(pose.pose.orientation) - y0));
        const double t = pulse_dur_ - rot_dead_time_;
        const double obs = t > 0.05 ? turned / t : 0.0;
        if (turned > 0.03 && obs > 0.6 * rot_rate_ && obs < 1.7 * rot_rate_) {
          const double prev = rot_rate_;
          rot_rate_ = std::clamp((1.0 - adapt_gain_) * rot_rate_ + adapt_gain_ * obs, 0.5, 6.0);
          if (node) {
            RCLCPP_INFO(
              node->get_logger(), "simple: turn pulse %.2f s turned %.3f rad -> rot_rate %.3f -> %.3f",
              pulse_dur_, turned, prev, rot_rate_);
          }
        }
      }
    }
    pulse_kind_ = PulseKind::NONE;
  }
  if (path_.poses.empty()) {
    return cmd;
  }

  const std::string frame = pose.header.frame_id;
  geometry_msgs::msg::PoseStamped goal;
  if (!transformToPoseFrame(path_.poses.back(), frame, goal)) {
    if (node) {
      RCLCPP_WARN_THROTTLE(
        node->get_logger(), *node->get_clock(), 1000,
        "simple: cannot transform goal %s -> %s", path_.header.frame_id.c_str(), frame.c_str());
    }
    return cmd;
  }
  const double yaw = tf2::getYaw(pose.pose.orientation);
  const double gx = goal.pose.position.x - pose.pose.position.x;
  const double gy = goal.pose.position.y - pose.pose.position.y;
  const double dist = std::hypot(gx, gy);

  const auto stop_and_settle = [&]() {
      if (was_moving_) {
        settle_until_ = now + secs(settle_time_);
      }
      was_moving_ = false;
    };
  const auto start_pulse = [&](double vx, double wz, double dur, const char * what) {
      pulse_vx_ = vx;
      pulse_wz_ = wz;
      pulse_until_ = now + secs(dur);
      pulse_kind_ = vx != 0.0 ? PulseKind::DRIVE : PulseKind::TURN;
      pulse_start_ = pose;
      pulse_dur_ = dur;
      cmd.twist.linear.x = vx;
      cmd.twist.angular.z = wz;
      if (node) {
        RCLCPP_INFO(
          node->get_logger(), "simple: %s pulse vx=%.2f wz=%.2f for %.2f s (dist=%.3f m)",
          what, vx, wz, dur, dist);
      }
    };
  const auto rotate_pulse = [&](double err) {
      const double dur = std::clamp(
        rot_dead_time_ + std::abs(err) / rot_rate_, 0.12, rotate_pulse_);
      start_pulse(0.0, std::copysign(wz_floor_, err), dur, "turn");
    };

  if (phase_ != Phase::DRIVE && dist > xy_retry_distance_) {
    phase_ = Phase::DRIVE;
  }
  if (phase_ == Phase::DRIVE && dist < brake_distance_) {
    phase_ = Phase::ROTATE;
    if (node) {
      RCLCPP_INFO(node->get_logger(), "simple: position reached (%.3f m), checking heading", dist);
    }
    stop_and_settle();
    return cmd;
  }

  if (phase_ == Phase::DRIVE) {
    // Lookahead point on the plan, or the goal itself when close.
    geometry_msgs::msg::PoseStamped target = goal;
    if (dist > lookahead_distance_) {
      size_t closest = 0;
      double best = std::numeric_limits<double>::max();
      std::vector<geometry_msgs::msg::PoseStamped> pts;
      pts.reserve(path_.poses.size());
      for (const auto & p : path_.poses) {
        geometry_msgs::msg::PoseStamped q;
        if (!transformToPoseFrame(p, frame, q)) {
          return cmd;
        }
        const double d = std::hypot(
          q.pose.position.x - pose.pose.position.x, q.pose.position.y - pose.pose.position.y);
        if (d < best) {
          best = d;
          closest = pts.size();
        }
        pts.push_back(q);
      }
      for (size_t i = closest; i < pts.size(); ++i) {
        target = pts[i];
        if (std::hypot(
            pts[i].pose.position.x - pose.pose.position.x,
            pts[i].pose.position.y - pose.pose.position.y) >= lookahead_distance_)
        {
          break;
        }
      }
    }
    const double tx = target.pose.position.x - pose.pose.position.x;
    const double ty = target.pose.position.y - pose.pose.position.y;
    double err = wrap(std::atan2(ty, tx) - yaw);
    double dir = 1.0;
    // Small overshoot / goal just behind: back up instead of turning around.
    if (std::abs(wrap(std::atan2(gy, gx) - yaw)) > M_PI_2 && dist < reverse_distance_) {
      dir = -1.0;
      err = wrap(err + M_PI);
    }
    const double steer = std::clamp(steer_gain_ * err, -steer_max_, steer_max_);
    if (was_moving_) {
      // Cruising closed loop: stop early, the chassis + odometry lag carries
      // it most of the remaining way; the rest is a timed pulse.
      if (std::abs(err) > align_angle_ || dist < cruise_stop_distance_) {
        stop_and_settle();
        return cmd;
      }
      cmd.twist.linear.x = dir * drive_speed_;
      cmd.twist.angular.z = steer;
      return cmd;
    }
    // At rest (after settling).
    // Near the goal the bearing to it swings wildly with a few cm of offset;
    // turning in place there only costs heading.  If the sideways offset is
    // already within tolerance, just pulse straight forward/back.
    if (dist < near_goal_distance_) {
      const double fwd = gx * std::cos(yaw) + gy * std::sin(yaw);
      const double lat = -gx * std::sin(yaw) + gy * std::cos(yaw);
      if (std::abs(lat) < 0.9 * xy_retry_distance_) {
        if (std::abs(fwd) < brake_distance_) {
          phase_ = Phase::ROTATE;
          if (node) {
            RCLCPP_INFO(
              node->get_logger(), "simple: position reached (fwd %.3f lat %.3f m), checking heading",
              fwd, lat);
          }
          return cmd;
        }
        start_pulse(
          std::copysign(drive_speed_, fwd), 0.0,
          ff_dead_time_ + std::abs(fwd) / ff_speed_, "near-goal drive");
        return cmd;
      }
    }
    if (std::abs(err) > align_angle_) {
      rotate_pulse(err);
      return cmd;
    }
    if (dist <= ff_max_distance_) {
      start_pulse(dir * drive_speed_, steer, ff_dead_time_ + dist / ff_speed_, "drive");
      return cmd;
    }
    cmd.twist.linear.x = dir * drive_speed_;
    cmd.twist.angular.z = steer;
    was_moving_ = true;
    return cmd;
  }

  if (phase_ == Phase::ROTATE) {
    const double yerr = goalYawError(pose);
    if (std::abs(yerr) <= yaw_tolerance_) {
      phase_ = Phase::ARRIVED;
      if (node) {
        RCLCPP_INFO(
          node->get_logger(), "simple: arrived dist=%.3f m yaw_err=%.3f rad", dist, yerr);
      }
      return cmd;
    }
    rotate_pulse(yerr);
    return cmd;
  }

  // ARRIVED: hold still until a new goal or a push of > xy_retry_distance.
  return cmd;
}

void DeadbandController::stopTimer()
{
  auto node = parent_.lock();
  if (!node || !stop_pub_) {
    return;
  }
  const auto idle = std::chrono::steady_clock::now() - last_cmd_time_;
  if (idle < std::chrono::duration<double>(cmd_vel_timeout_)) {
    return;
  }

  geometry_msgs::msg::Twist cmd;
  stop_pub_->publish(cmd);
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
  phase_ = Phase::DRIVE;
  pulse_until_ = {};
  settle_until_ = {};
  was_moving_ = false;
  have_goal_ = false;
  pulse_kind_ = PulseKind::NONE;
  if (primary_controller_) {
    primary_controller_->reset();
  }
}

}  // namespace nav2_deadband_controller

PLUGINLIB_EXPORT_CLASS(
  nav2_deadband_controller::DeadbandController,
  nav2_core::Controller)
