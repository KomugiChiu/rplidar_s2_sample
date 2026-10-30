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
  double goalDistance(const geometry_msgs::msg::PoseStamped & pose);
  double goalYawError(const geometry_msgs::msg::PoseStamped & pose);
  void onSonar(const std_msgs::msg::Float32::SharedPtr msg);
  void stopTimer();
  bool transformToPoseFrame(
    const geometry_msgs::msg::PoseStamped & in, const std::string & frame,
    geometry_msgs::msg::PoseStamped & out);
  geometry_msgs::msg::TwistStamped computeSimple(
    const geometry_msgs::msg::PoseStamped & pose);

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
  double rotate_stop_angle_{0.35};
  double lookahead_distance_{0.3};
  double goal_rotate_distance_{0.30};
  double path_min_distance_{0.10};

  // mode "simple": deterministic pursuit tuned for the open-loop Yahboom
  // chassis (PWM deadband, ~0.2 s actuation lag, no usable strafe).
  // mode "mppi": primary_controller + floors (old behaviour).
  enum class Phase { DRIVE, ROTATE, ARRIVED };
  std::string mode_{"simple"};
  Phase phase_{Phase::DRIVE};
  double drive_speed_{0.14};       // lowest vx that reliably moves the base
  double steer_gain_{0.8};         // wz = gain * heading error while driving
  double steer_max_{0.30};         // |wz| cap while driving (differential trim)
  double align_angle_{0.6};        // heading error that triggers turn-in-place
  double reverse_distance_{0.25};  // goal behind & closer than this -> reverse
  double brake_distance_{0.05};    // stop driving this far before the goal
  double xy_retry_distance_{0.12}; // re-drive only if pushed this far off
  double yaw_tolerance_{0.30};     // final heading tolerance (rad)
  double rotate_pulse_{0.45};      // max duration of one turn-in-place pulse (s)
  double settle_time_{0.70};       // hold zero after every stop (s)
  // Short moves are open-loop timed pulses; the whole loop (smoother,
  // collision_monitor, MCU, rf2o) lags ~0.45 s, too slow for feedback at 10 cm.
  double ff_max_distance_{0.30};   // goals closer than this use one timed pulse
  double ff_dead_time_{0.25};      // pulse T = dead_time + d / ff_speed
  double ff_speed_{0.40};
  double cruise_stop_distance_{0.20};  // closed-loop: stop this far out, then pulse
  double near_goal_distance_{0.15};    // inside this: no turn-in-place for position
  // Online pulse calibration: after each pulse settles, compare the commanded
  // pulse with the measured motion and blend the result into ff_speed /
  // rot_rate.  Open-loop PWM speed follows battery voltage and differs per
  // robot, so fixed values drift.  0 disables.
  double adapt_gain_{0.3};
  enum class PulseKind { NONE, DRIVE, TURN };
  PulseKind pulse_kind_{PulseKind::NONE};
  geometry_msgs::msg::PoseStamped pulse_start_;
  double pulse_dur_{0.0};
  double rot_dead_time_{0.05};     // rotate pulse T = dead_time + |err| / rot_rate
  double rot_rate_{2.0};
  std::chrono::steady_clock::time_point settle_until_{};
  std::chrono::steady_clock::time_point pulse_until_{};
  bool was_moving_{false};
  double pulse_vx_{0.0};
  double pulse_wz_{0.0};
  bool have_goal_{false};
  geometry_msgs::msg::PoseStamped last_goal_;

  // Safety heartbeat: when the controller stops being called (goal done,
  // Nav2 crash, ...), keep publishing zero so the chassis does not latch the
  // last non-zero command.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr stop_pub_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  std::chrono::steady_clock::time_point last_cmd_time_{};
  double cmd_vel_timeout_{0.25};

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
