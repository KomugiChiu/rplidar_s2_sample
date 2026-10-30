#ifndef NAV2_DEADBAND_CONTROLLER__DEADBAND_CONTROLLER_HPP_
#define NAV2_DEADBAND_CONTROLLER__DEADBAND_CONTROLLER_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_deadband_controller
{

class DeadbandController : public nav2_core::Controller
{
public:
  DeadbandController() = default;
  ~DeadbandController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  bool cancel() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  bool forwardBlocked(const geometry_msgs::msg::PoseStamped & pose);
  double floorAxis(double value, double floor) const;
  double headingError(const geometry_msgs::msg::PoseStamped & pose);
  void onSonar(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  nav2_core::Controller::Ptr primary_controller_;
  std::string primary_controller_name_;

  double vx_floor_{0.16};
  double vy_floor_{0.0};
  double wz_floor_{0.52};
  double deadband_{0.005};
  double stop_distance_{0.30};
  double stop_half_width_{0.15};

  // "turn in place only when the heading error is large" state machine
  nav_msgs::msg::Path path_;
  bool rotating_{false};
  double rotate_start_angle_{0.4};
  double rotate_stop_angle_{0.1};
  double lookahead_distance_{0.3};

  // ultrasonic (single forward range, cm on /ultrasonic)
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sonar_sub_;
  std::string sonar_topic_;
  double sonar_stop_distance_{0.35};
  double sonar_hold_time_{0.3};
  double sonar_dist_{-1.0};
  std::chrono::steady_clock::time_point sonar_blocked_until_{};
  std::chrono::steady_clock::time_point last_sonar_log_{};
};

}  // namespace nav2_deadband_controller

#endif  // NAV2_DEADBAND_CONTROLLER__DEADBAND_CONTROLLER_HPP_
