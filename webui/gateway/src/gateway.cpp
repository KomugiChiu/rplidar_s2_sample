// robot_console_gateway: ROS 2 side of the robot web console.
//
// Started and restarted by robot-console (Go, ../console); never talks to a
// browser itself.
//
//   stdin : one JSON command per line   {"cmd": "...", "id": 7, ...}
//   stdout: frames [u32 hdr_len][u32 payload_len][hdr JSON][payload]
//           hdr = {"ch": <channel>, "d": <data>}, payload = JPEG or empty
//
// Written in C++ because rclpy costs ~2 ms CPU per callback on this SoC
// (measured 2026-11-03); the gateway watches ~450 msgs/s (gyro 200 Hz, /tf,
// odometry, cmd_vel chain).  Monitored topics use generic subscriptions and
// are only deserialized when somebody needs the content.  Heavy outputs
// (scan, map, costmaps, camera, topic inspector) are produced only while the
// Go side reports a viewer ("demand").
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <iostream>
#include <map>
#include <mutex>
#include <optional>
#include <queue>
#include <regex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>

#include <action_msgs/msg/goal_status_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/msg/behavior_tree_log.hpp>
#include <nav2_msgs/msg/collision_monitor_state.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "introspect.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------
// Manual drive.  No obstacle check by design: the override exists to get the
// robot out of places Nav2 refuses to leave.
constexpr double TELEOP_MAX_LIN = 0.35;    // m/s
constexpr double TELEOP_MAX_ANG = 1.5;     // rad/s
constexpr double TELEOP_DEADMAN_S = 0.5;   // stop when keepalives stop
constexpr const char * TELEOP_TOPIC = "/cmd_vel";  // driver input; collision_monitor is silent when idle
constexpr double ESTOP_ZERO_S = 1.0;

constexpr double NONE = std::numeric_limits<double>::quiet_NaN();

// Owners = who fixes it.  NAV/SLAM team: LIDAR ODOM EKF SLAM TF NAV2 CHASSIS;
// AI team: AI DEMO; CAMERA is the shared RealSense node (started by ekf/).
struct MonDef
{
  const char * topic, * type, * qos, * owner, * label;
  double expect, warn, err;
  bool optional;
};
static const MonDef MONITORS[] = {
  {"/scan", "sensor_msgs/msg/LaserScan", "sensor", "LIDAR", "Lidar scan", 10, 8, 5, false},
  {"/odom_rf2o", "nav_msgs/msg/Odometry", "sensor", "ODOM", "rf2o odometry", 10, 8, 3, false},
  {"/odometry/filtered", "nav_msgs/msg/Odometry", "sensor", "EKF", "EKF odometry", 20, 15, 8, false},
  {"/camera/camera/gyro/sample", "sensor_msgs/msg/Imu", "sensor", "CAMERA", "D455 gyro (EKF input)",
    200, 100, 1, false},
  {"/camera/camera/color/camera_info", "sensor_msgs/msg/CameraInfo", "sensor", "CAMERA", "Color stream",
    30, 20, 5, true},
  {"/camera/camera/aligned_depth_to_color/camera_info", "sensor_msgs/msg/CameraInfo", "sensor", "CAMERA",
    "Aligned depth stream", 30, 20, 5, true},
  {"/ultrasonic", "std_msgs/msg/Float32", "sensor", "CHASSIS", "Chassis driver (sonar)", 10, 8, 3, false},
  {"/person_infos", "ai_human_detection/msg/PersonInfoArray", "sensor", "AI", "Person positions",
    NONE, 2, 0.5, true},
  {"/human_detections", "vision_msgs/msg/Detection2DArray", "sensor", "AI", "2D detections",
    NONE, 2, 0.5, true},
  {"/demo/person_markers", "visualization_msgs/msg/MarkerArray", "sensor", "DEMO", "Person markers",
    NONE, NONE, NONE, true},
  {"/map", "nav_msgs/msg/OccupancyGrid", "latched", "SLAM", "Map", NONE, NONE, NONE, false},
  {"/plan", "nav_msgs/msg/Path", "sensor", "NAV2", "Global plan", NONE, NONE, NONE, true},
  {"/global_costmap/costmap", "nav_msgs/msg/OccupancyGrid", "sensor", "NAV2", "Global costmap",
    NONE, NONE, NONE, true},
  {"/local_costmap/costmap", "nav_msgs/msg/OccupancyGrid", "sensor", "NAV2", "Local costmap",
    NONE, NONE, NONE, true},
  {"/collision_monitor_state", "nav2_msgs/msg/CollisionMonitorState", "sensor", "NAV2",
    "Collision monitor state", NONE, NONE, NONE, true},
};
// Counted by typed callbacks (every message is needed).
static const MonDef TYPED[] = {
  {"/tf", "", "", "TF", "TF", NONE, NONE, NONE, false},
  {"/cmd_vel_nav", "", "", "NAV2", "Controller output", NONE, NONE, NONE, false},
  {"/cmd_vel_smoothed", "", "", "NAV2", "Velocity smoother output", NONE, NONE, NONE, false},
  {"/cmd_vel", "", "", "CHASSIS", "Driver input", NONE, NONE, NONE, false},
  {"/rosout", "", "", "", "rosout", NONE, NONE, NONE, false},
  {"/debug_log", "", "", "AI", "AI debug log", NONE, NONE, NONE, false},
  {"/behavior_tree_log", "", "", "NAV2", "BT log", NONE, NONE, NONE, false},
  {"/demo/state", "", "", "DEMO", "Demo state", NONE, NONE, NONE, false},
};

struct TfCheck
{
  const char * parent, * child, * owner, * label;
  double warn, err;
  bool is_static;
};
static const TfCheck TF_CHECKS[] = {
  {"map", "odom", "SLAM", "map→odom (slam_toolbox)", 0.3, 1.0, false},
  {"odom", "base_footprint", "EKF", "odom→base_footprint (EKF)", 0.15, 0.5, false},
  {"base_footprint", "base_link", "TF", "base_footprint→base_link", 0, 0, true},
  {"base_link", "laser", "TF", "base_link→laser (yaw 180°)", 0, 0, true},
  {"base_footprint", "camera_link", "TF", "base_footprint→camera_link", 0, 0, true},
  {"base_link", "ai_human_detection", "AI", "base_link→ai_human_detection", 0, 0, true},
};

struct LcDef
{
  const char * node, * owner;
  bool required;
};
static const LcDef LIFECYCLE[] = {
  {"controller_server", "NAV2", true}, {"planner_server", "NAV2", true},
  {"smoother_server", "NAV2", true}, {"behavior_server", "NAV2", true},
  {"bt_navigator", "NAV2", true}, {"waypoint_follower", "NAV2", true},
  {"velocity_smoother", "NAV2", true}, {"collision_monitor", "NAV2", true},
  {"global_costmap/global_costmap", "NAV2", true}, {"local_costmap/local_costmap", "NAV2", true},
  {"route_server", "NAV2", false}, {"docking_server", "NAV2", false},
  {"slam_toolbox", "SLAM", false},
};

static const std::map<std::string, std::string> CAMERAS = {
  {"color", "/camera/camera/color/image_raw/compressed"},
  {"debug", "/debug_image/compressed"},
};

// rosout text that explains a stop; shown in the motion pipeline for 5 s.
struct BlockerDef
{
  const char * pattern, * owner, * label, * clear;
};
static const BlockerDef BLOCKERS[] = {
  {"costmap stop", "NAV2", "Controller: costmap stop (obstacle ahead)", nullptr},
  {"ultrasonic stop", "NAV2", "Controller: ultrasonic stop", nullptr},
  {"escape:", "NAV2", "Controller: escape manoeuvre", nullptr},
  {"Robot to stop due to invalid source", "NAV2", "collision_monitor: invalid source (TF/scan timing)",
    "Robot to continue normal operation"},
  {"Failed to make progress", "NAV2", "Progress checker: no progress", nullptr},
  {"Collision Ahead", "NAV2", "Recovery refused: collision ahead", nullptr},
  {"Transform data too old|xtrapolation", "TF", "TF timing error (data too old / extrapolation)", nullptr},
  {"Message Filter dropping", "TF", "Consumer dropping scans (TF lag / queue full)", nullptr},
  {"Goal (failed|was aborted)|Aborting handle", "NAV2", "Nav2 aborted the goal", nullptr},
};

static const char * STATUS_NAMES[] = {"UNKNOWN", "ACCEPTED", "EXECUTING", "CANCELING",
  "SUCCEEDED", "CANCELED", "ABORTED"};
static const char * COLLISION_ACTIONS[] = {"DO_NOTHING", "STOP", "SLOWDOWN", "APPROACH", "LIMIT"};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static int g_out_fd = -1;
static std::mutex g_out_mu;

static void write_all(const char * p, size_t n)
{
  while (n > 0) {
    ssize_t w = ::write(g_out_fd, p, n);
    if (w < 0) {
      if (errno == EINTR) {continue;}
      _exit(0);  // Go went away
    }
    p += w;
    n -= static_cast<size_t>(w);
  }
}

static void emit(const std::string & ch, const json & d, const std::string & payload = {})
{
  json h = {{"ch", ch}, {"d", d}};
  std::string hs = h.dump(-1, ' ', false, json::error_handler_t::replace);
  uint8_t len[8];
  uint32_t a = static_cast<uint32_t>(hs.size()), b = static_cast<uint32_t>(payload.size());
  for (int i = 0; i < 4; ++i) {
    len[i] = static_cast<uint8_t>(a >> (24 - 8 * i));
    len[4 + i] = static_cast<uint8_t>(b >> (24 - 8 * i));
  }
  std::lock_guard<std::mutex> lk(g_out_mu);
  write_all(reinterpret_cast<const char *>(len), 8);
  write_all(hs.data(), hs.size());
  if (!payload.empty()) {write_all(payload.data(), payload.size());}
}

static double mono()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
static double wall()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
// rounded number, NaN -> null
static json num(double x, int nd = 3)
{
  if (!std::isfinite(x)) {return nullptr;}
  double p = std::pow(10.0, nd);
  return std::round(x * p) / p;
}
static double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
static double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
static double stamp_s(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}
static double dur_s(const builtin_interfaces::msg::Duration & t) {return t.sec + t.nanosec * 1e-9;}
static std::string strip(const std::string & f) {return (!f.empty() && f[0] == '/') ? f.substr(1) : f;}

struct Pose2 {double x{0}, y{0}, yaw{0};};
static Pose2 compose(const Pose2 & a, const Pose2 & b)
{
  double c = std::cos(a.yaw), s = std::sin(a.yaw);
  return {a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, a.yaw + b.yaw};
}

static std::string b64(const uint8_t * d, size_t n)
{
  static const char * t = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string o;
  o.reserve((n + 2) / 3 * 4);
  size_t i = 0;
  for (; i + 2 < n; i += 3) {
    uint32_t v = (d[i] << 16) | (d[i + 1] << 8) | d[i + 2];
    o += t[v >> 18]; o += t[(v >> 12) & 63]; o += t[(v >> 6) & 63]; o += t[v & 63];
  }
  if (i + 1 == n) {
    uint32_t v = d[i] << 16;
    o += t[v >> 18]; o += t[(v >> 12) & 63]; o += "==";
  } else if (i + 2 == n) {
    uint32_t v = (d[i] << 16) | (d[i + 1] << 8);
    o += t[v >> 18]; o += t[(v >> 12) & 63]; o += t[(v >> 6) & 63]; o += '=';
  }
  return o;
}

static rclcpp::QoS make_qos(const std::string & kind, size_t depth = 10)
{
  if (kind == "latched") {return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();}
  if (kind == "reliable") {return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();}
  // best effort + volatile matches every publisher
  return rclcpp::QoS(rclcpp::KeepLast(depth)).best_effort().durability_volatile();
}

static json qos_json(const rclcpp::QoS & q)
{
  auto p = q.get_rmw_qos_profile();
  auto rel = p.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE ? "RELIABLE" :
    p.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT ? "BEST_EFFORT" : "OTHER";
  auto dur = p.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL ? "TRANSIENT_LOCAL" :
    p.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE ? "VOLATILE" : "OTHER";
  return {{"reliability", rel}, {"durability", dur}, {"depth", p.depth}};
}

struct Mon
{
  std::string topic, type, owner, label;
  double expect{NONE}, warn{NONE}, err{NONE};
  bool optional{false};
  bool type_ok{true};
  uint64_t count{0}, bytes{0}, seq{0};
  double last{-1};
  std::deque<std::array<double, 3>> hist;
  double hz{0}, bw{0};
  size_t pubs{0};
  std::shared_ptr<const rclcpp::SerializedMessage> raw;
  rclcpp::SubscriptionBase::SharedPtr sub;
  // inspector extras
  std::shared_ptr<TypeCodec> codec;
  double info_t{0};
  json pubs_info = json::array(), subs_info = json::array();

  void tick(size_t n) {count++; bytes += n; last = mono();}
  void sample(double now)
  {
    hist.push_back({now, static_cast<double>(count), static_cast<double>(bytes)});
    while (hist.size() > 6) {hist.pop_front();}
    auto & h0 = hist.front();
    if (now - h0[0] > 0.5) {
      hz = (count - h0[1]) / (now - h0[0]);
      bw = (bytes - h0[2]) / (now - h0[0]);
    }
  }
  json age(double now) const {return last < 0 ? json(nullptr) : num(now - last, 2);}
  template<class T>
  bool get(T & out) const
  {
    if (!raw) {return false;}
    try {
      static rclcpp::Serialization<T> s;
      s.deserialize_message(raw.get(), &out);
      return true;
    } catch (...) {return false;}
  }
};

struct Edge
{
  std::string parent, child;
  bool is_static{false};
  uint64_t count{0};
  double last{-1}, lag{NONE}, hz{0}, max_gap{0};
  std::deque<std::pair<double, double>> hist;
  int gaps{0}, jumps{0};
  std::optional<Pose2> pose;
};

class Health
{
public:
  json items = json::array();
  void begin() {items = json::array();}
  // ERROR only after 3 bad samples in a row unless immediate
  void add(const std::string & id, const std::string & owner, const std::string & name, std::string level,
    const std::string & value = "", const std::string & detail = "", bool immediate = false)
  {
    if (level == "ERROR" && !immediate) {
      if (++bad_[id] < 3) {level = "WARN";}
    } else if (level != "ERROR") {
      bad_[id] = 0;
    }
    items.push_back({{"id", id}, {"owner", owner}, {"name", name}, {"level", level},
      {"value", value}, {"detail", detail}});
  }

private:
  std::map<std::string, int> bad_;
};

static std::string fmt(const char * f, double a)
{
  char b[64];
  snprintf(b, sizeof b, f, a);
  return b;
}
static std::string fmt2(const char * f, double a, double b2)
{
  char b[96];
  snprintf(b, sizeof b, f, a, b2);
  return b;
}

// ---------------------------------------------------------------------------
// Gateway node
// ---------------------------------------------------------------------------
using NavToPose = nav2_msgs::action::NavigateToPose;
using FollowWp = nav2_msgs::action::FollowWaypoints;
using NavThrough = nav2_msgs::action::NavigateThroughPoses;

struct NavState
{
  std::map<std::string, int8_t> goals;
  std::set<std::string> done;
  json fb;
  double fb_t{-1};
};

class Gateway : public rclcpp::Node
{
public:
  Gateway()
  : Node("robot_console")
  {
    t0_ = mono();
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(10.0));

    for (const auto & d : MONITORS) {
      auto m = std::make_shared<Mon>();
      m->topic = d.topic; m->type = d.type; m->owner = d.owner; m->label = d.label;
      m->expect = d.expect; m->warn = d.warn; m->err = d.err; m->optional = d.optional;
      mons_[d.topic] = m;
      try {
        m->sub = create_generic_subscription(d.topic, d.type, make_qos(d.qos),
          [m](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
            m->tick(msg->size());
            m->raw = msg;
            m->seq++;
          });
      } catch (const std::exception & e) {
        m->type_ok = false;
        RCLCPP_WARN(get_logger(), "%s: cannot subscribe as %s: %s", d.topic, d.type, e.what());
      }
    }
    for (const auto & d : TYPED) {
      auto m = std::make_shared<Mon>();
      m->topic = d.topic; m->owner = d.owner; m->label = d.label;
      mons_[d.topic] = m;
    }
    try {
      person_codec_ = std::make_shared<TypeCodec>("ai_human_detection/msg/PersonInfoArray");
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "ai_human_detection messages unavailable: %s", e.what());
    }

    subs_.push_back(create_subscription<tf2_msgs::msg::TFMessage>("/tf", make_qos("sensor", 100),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {on_tf(*m, false);}));
    subs_.push_back(create_subscription<tf2_msgs::msg::TFMessage>("/tf_static",
      rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local(),
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m) {on_tf(*m, true);}));
    for (const char * t : {"/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel"}) {
      std::string topic = t;
      cmd_[topic] = {};
      subs_.push_back(create_subscription<geometry_msgs::msg::Twist>(topic, make_qos("sensor"),
        [this, topic](geometry_msgs::msg::Twist::ConstSharedPtr m) {on_twist(topic, *m);}));
    }
    subs_.push_back(create_subscription<rcl_interfaces::msg::Log>("/rosout", make_qos("reliable", 200),
      [this](rcl_interfaces::msg::Log::ConstSharedPtr m) {on_rosout(*m);}));
    subs_.push_back(create_subscription<std_msgs::msg::String>("/debug_log", make_qos("reliable", 20),
      [this](std_msgs::msg::String::ConstSharedPtr m) {on_ai_debug(*m);}));
    subs_.push_back(create_subscription<nav2_msgs::msg::BehaviorTreeLog>("/behavior_tree_log",
      make_qos("reliable", 20), [this](nav2_msgs::msg::BehaviorTreeLog::ConstSharedPtr m) {on_bt(*m);}));
    subs_.push_back(create_subscription<std_msgs::msg::String>("/demo/state", make_qos("latched"),
      [this](std_msgs::msg::String::ConstSharedPtr m) {on_demo_state(m->data);}));
    subs_.push_back(create_subscription<std_msgs::msg::Int32>("/demo/selected_tracking_id",
      make_qos("latched"), [this](std_msgs::msg::Int32::ConstSharedPtr m) {demo_selected_ = m->data;}));
    subs_.push_back(create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
      make_qos("reliable"), [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr m) {on_diag(*m);}));
    subs_.push_back(create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", make_qos("reliable", 5),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        ext_goal_ = Pose2{m->pose.position.x, m->pose.position.y, yaw_of(m->pose.orientation)};
        ext_goal_t_ = mono();
      }));

    // everybody's Nav2 goals (RViz, demo_manager, us)
    auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();
    for (const char * a : {"navigate_to_pose", "follow_waypoints", "navigate_through_poses"}) {
      std::string name = a;
      nav_[name] = {};
      subs_.push_back(create_subscription<action_msgs::msg::GoalStatusArray>(
        "/" + name + "/_action/status", status_qos,
        [this, name](action_msgs::msg::GoalStatusArray::ConstSharedPtr m) {on_nav_status(name, *m);}));
    }
    subs_.push_back(create_subscription<NavToPose::Impl::FeedbackMessage>(
      "/navigate_to_pose/_action/feedback", make_qos("reliable"),
      [this](NavToPose::Impl::FeedbackMessage::ConstSharedPtr m) {
        auto & f = m->feedback;
        nav_["navigate_to_pose"].fb = {{"action", "navigate_to_pose"}, {"dist", num(f.distance_remaining, 2)},
          {"rec", f.number_of_recoveries}, {"elapsed", num(dur_s(f.navigation_time), 1)},
          {"eta", num(dur_s(f.estimated_time_remaining), 0)}};
        nav_["navigate_to_pose"].fb_t = mono();
      }));
    subs_.push_back(create_subscription<FollowWp::Impl::FeedbackMessage>(
      "/follow_waypoints/_action/feedback", make_qos("reliable"),
      [this](FollowWp::Impl::FeedbackMessage::ConstSharedPtr m) {
        nav_["follow_waypoints"].fb = {{"action", "follow_waypoints"}, {"wp", m->feedback.current_waypoint}};
        nav_["follow_waypoints"].fb_t = mono();
      }));
    subs_.push_back(create_subscription<NavThrough::Impl::FeedbackMessage>(
      "/navigate_through_poses/_action/feedback", make_qos("reliable"),
      [this](NavThrough::Impl::FeedbackMessage::ConstSharedPtr m) {
        auto & f = m->feedback;
        nav_["navigate_through_poses"].fb = {{"action", "navigate_through_poses"},
          {"dist", num(f.distance_remaining, 2)}, {"rec", f.number_of_recoveries},
          {"remaining", f.number_of_poses_remaining}};
        nav_["navigate_through_poses"].fb_t = mono();
      }));
    ac_nav_ = rclcpp_action::create_client<NavToPose>(this, "navigate_to_pose");
    ac_wp_ = rclcpp_action::create_client<FollowWp>(this, "follow_waypoints");
    ac_through_ = rclcpp_action::create_client<NavThrough>(this, "navigate_through_poses");
    demo_stop_ = create_client<std_srvs::srv::Trigger>("/demo/stop");
    clear_global_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
    clear_local_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap");
    for (const auto & l : LIFECYCLE) {
      lc_cli_[l.node] = create_client<lifecycle_msgs::srv::GetState>(std::string("/") + l.node + "/get_state");
    }
    for (const auto & b : BLOCKERS) {
      blocker_re_.push_back({std::regex(b.pattern), b.clear ? std::optional<std::regex>(std::regex(b.clear)) :
        std::nullopt});
    }

    teleop_pub_ = create_publisher<geometry_msgs::msg::Twist>(TELEOP_TOPIC, 10);
    init_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    timer_ = create_wall_timer(50ms, [this]() {tick();});
    event("CONSOLE", "INFO", "gateway started");
  }

  void push_command(json c)
  {
    std::lock_guard<std::mutex> lk(cmd_mu_);
    cmds_.push(std::move(c));
  }

private:
  // ------------------------------------------------------------------ util
  double ros_now() {return get_clock()->now().seconds();}
  void event(const std::string & owner, const std::string & level, const std::string & text)
  {
    emit("events", {{"items", json::array({{{"t", wall()}, {"owner", owner}, {"level", level}, {"text", text}}})}});
  }
  std::optional<Pose2> lookup2d(const std::string & target, const std::string & source)
  {
    try {
      auto t = tf_buffer_->lookupTransform(target, source, tf2::TimePointZero);
      return Pose2{t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
    } catch (...) {return std::nullopt;}
  }

  // ------------------------------------------------------------------- tf
  void on_tf(const tf2_msgs::msg::TFMessage & msg, bool is_static)
  {
    double now = mono();
    double rnow = is_static ? 0 : ros_now();
    if (!is_static) {mons_["/tf"]->tick(0);}
    for (auto t : msg.transforms) {
      t.header.frame_id = strip(t.header.frame_id);
      t.child_frame_id = strip(t.child_frame_id);
      try {tf_buffer_->setTransform(t, "console", is_static);} catch (...) {}
      auto & e = edges_[t.header.frame_id + ">" + t.child_frame_id];
      e.parent = t.header.frame_id;
      e.child = t.child_frame_id;
      e.is_static = is_static;
      Pose2 p{t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)};
      if (!is_static) {
        if (e.last >= 0) {
          double gap = now - e.last;
          if (gap > 0.5) {
            e.gaps++;
            e.max_gap = std::max(e.max_gap, gap);
            event(e.child == "odom" ? "SLAM" : (e.child == "base_footprint" ? "EKF" : "TF"),
              gap > 1.0 ? "ERROR" : "WARN",
              "TF " + e.parent + "→" + e.child + " gap " + fmt("%.2f s", gap));
          }
        }
        if (e.parent == "map" && e.child == "odom" && e.pose) {
          double d = std::hypot(p.x - e.pose->x, p.y - e.pose->y);
          double dy = std::abs(wrap(p.yaw - e.pose->yaw));
          if (d > 0.15 || dy > 0.10) {
            e.jumps++;
            event("SLAM", (d > 0.5 || dy > 0.3) ? "ERROR" : "WARN",
              "SLAM correction: map→odom jumped " + fmt2("%.2f m / %.1f°", d, dy * 180 / M_PI));
          }
        }
        if (e.parent == "odom") {
          odom_hist_.push_back({now, p.x, p.y, p.yaw});
          while (odom_hist_.size() > 300) {odom_hist_.pop_front();}
        }
        e.lag = rnow - stamp_s(t.header.stamp);
      }
      e.pose = p;
      e.count++;
      e.last = now;
    }
  }

  // ------------------------------------------------------------ callbacks
  struct CmdVal {double v[3]{0, 0, 0}; double nz{-1};};
  void on_twist(const std::string & topic, const geometry_msgs::msg::Twist & m)
  {
    mons_[topic]->tick(0);
    auto & c = cmd_[topic];
    c.v[0] = m.linear.x; c.v[1] = m.linear.y; c.v[2] = m.angular.z;
    if (std::abs(c.v[0]) > 1e-3 || std::abs(c.v[1]) > 1e-3 || std::abs(c.v[2]) > 1e-3) {
      c.nz = mono();
      if (topic == "/cmd_vel") {last_nonzero_cmd_ = c.nz;}
    }
  }

  void on_rosout(const rcl_interfaces::msg::Log & m)
  {
    mons_["/rosout"]->tick(m.msg.size());
    if (m.name == "robot_console") {return;}
    double now = mono();
    for (size_t i = 0; i < blocker_re_.size(); ++i) {
      const auto & def = BLOCKERS[i];
      if (std::regex_search(m.msg, blocker_re_[i].first)) {
        auto & b = blockers_[def.label];
        b.owner = def.owner;
        b.t = now;
        b.text = m.msg.substr(0, 200);
        b.n++;
      } else if (blocker_re_[i].second && blockers_.count(def.label) &&
        std::regex_search(m.msg, *blocker_re_[i].second))
      {
        blockers_[def.label].t = 0;
      }
    }
    static const std::map<uint8_t, const char *> lv = {{10, "D"}, {20, "I"}, {30, "W"}, {40, "E"}, {50, "F"}};
    if (log_buf_.size() < 2000) {
      auto it = lv.find(m.level);
      log_buf_.push_back({{"t", stamp_s(m.stamp)}, {"lvl", it == lv.end() ? "I" : it->second},
        {"node", m.name}, {"msg", m.msg}, {"src", "rosout"}});
    }
  }

  void on_ai_debug(const std_msgs::msg::String & m)
  {
    mons_["/debug_log"]->tick(m.data.size());
    static const std::regex err_re("error|fail", std::regex::icase), warn_re("warn", std::regex::icase);
    const char * lvl = std::regex_search(m.data, err_re) ? "E" : (std::regex_search(m.data, warn_re) ? "W" : "I");
    if (log_buf_.size() < 2000) {
      log_buf_.push_back({{"t", wall()}, {"lvl", lvl}, {"node", "ai_human_detection"}, {"msg", m.data},
        {"src", "ai_debug"}});
    }
  }

  void on_bt(const nav2_msgs::msg::BehaviorTreeLog & m)
  {
    mons_["/behavior_tree_log"]->tick(0);
    static const std::set<std::string> quiet = {"WouldAControllerRecoveryHelp", "WouldAPlannerRecoveryHelp",
      "GoalUpdated"};
    static const std::set<std::string> recov = {"BackUp", "Wait", "Spin", "ClearLocalCostmap-Subtree",
      "ClearGlobalCostmap-Subtree", "DriveOnHeading"};
    for (const auto & ev : m.event_log) {
      bt_status_[ev.node_name] = ev.current_status;
      if (ev.current_status == "FAILURE" && !quiet.count(ev.node_name)) {
        event("NAV2", "WARN", "BT " + ev.node_name + ": " + ev.previous_status + " → FAILURE");
      } else if (ev.current_status == "RUNNING" && recov.count(ev.node_name)) {
        event("NAV2", "INFO", "BT recovery: " + ev.node_name);
      }
    }
  }

  void on_demo_state(const std::string & s)
  {
    mons_["/demo/state"]->tick(0);
    if (s != demo_state_) {
      bool bad = s == "TARGET_LOST" || s == "NAVIGATION_FAILED";
      event("DEMO", bad ? "WARN" : "INFO",
        "demo_manager state " + (demo_state_.empty() ? std::string("?") : demo_state_) + " → " + s);
    }
    demo_state_ = s;
  }

  void on_diag(const diagnostic_msgs::msg::DiagnosticArray & m)
  {
    for (const auto & s : m.status) {
      diag_[s.name] = {{"name", s.name}, {"level", static_cast<int>(s.level)}, {"msg", s.message},
        {"hw", s.hardware_id}, {"t", mono()}};
    }
  }

  void on_nav_status(const std::string & a, const action_msgs::msg::GoalStatusArray & m)
  {
    auto & st = nav_[a];
    std::map<std::string, int8_t> goals;
    for (const auto & s : m.status_list) {
      char hex[33];
      for (int i = 0; i < 16; ++i) {snprintf(hex + 2 * i, 3, "%02x", s.goal_info.goal_id.uuid[i]);}
      std::string gid(hex);
      goals[gid] = s.status;
      bool active = s.status >= 1 && s.status <= 3;
      auto prev = st.goals.find(gid);
      if (!active && !st.done.count(gid)) {
        if (prev != st.goals.end() && prev->second >= 1 && prev->second <= 3) {
          event("NAV2", s.status == 4 ? "INFO" : "WARN",
            a + " goal " + gid.substr(0, 6) + " " + STATUS_NAMES[std::clamp<int>(s.status, 0, 6)]);
        }
        st.done.insert(gid);
      } else if (active && prev == st.goals.end()) {
        event("NAV2", "INFO", a + " goal " + gid.substr(0, 6) + " accepted");
      }
    }
    st.goals = goals;
    if (st.done.size() > 300) {st.done.clear();}
  }

  std::vector<std::string> nav_active()
  {
    std::vector<std::string> out;
    for (auto & [a, st] : nav_) {
      for (auto & [g, s] : st.goals) {
        if (s >= 1 && s <= 3) {out.push_back(a); break;}
      }
    }
    return out;
  }

  // -------------------------------------------------------------- commands
  void handle(const json & c)
  {
    std::string cmd = c.value("cmd", "");
    std::string src = c.value("src", "?");
    bool ok = false;
    json data = nullptr;
    std::string msg;
    try {
      if (cmd == "demand") {set_demand(c.value("ch", json::object())); ok = true;}
      else if (cmd == "teleop") {ok = cmd_teleop(c, src, msg);}
      else if (cmd == "estop") {ok = cmd_estop(src, msg);}
      else if (cmd == "goal") {ok = cmd_goal(c, src, msg);}
      else if (cmd == "waypoints") {ok = cmd_waypoints(c, src, msg);}
      else if (cmd == "cancel") {
        int n = cancel_all();
        event("CONSOLE", "INFO", "cancel all goals from " + src);
        ok = n > 0; msg = "cancel sent to " + std::to_string(n) + " action server(s)";
      } else if (cmd == "initialpose") {ok = cmd_initialpose(c, src, msg);}
      else if (cmd == "clear_costmaps") {ok = cmd_clear(msg);}
      else if (cmd == "node_info") {ok = cmd_node_info(c, data, msg);}
      else {msg = "unknown command " + cmd;}
    } catch (const std::exception & e) {
      ok = false;
      msg = e.what();
    }
    if (c.contains("id")) {
      emit("reply", {{"id", c["id"]}, {"ok", ok}, {"msg", msg}, {"data", data}});
    }
  }

  int cancel_all()
  {
    // waypoint follower first, otherwise it sends the next waypoint
    int n = 0;
    if (ac_wp_->action_server_is_ready()) {ac_wp_->async_cancel_all_goals(); n++;}
    if (ac_through_->action_server_is_ready()) {ac_through_->async_cancel_all_goals(); n++;}
    if (ac_nav_->action_server_is_ready()) {ac_nav_->async_cancel_all_goals(); n++;}
    mission_ = nullptr;
    return n;
  }

  bool demo_stop()
  {
    if (!demo_stop_->service_is_ready()) {return false;}
    demo_stop_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    return true;
  }

  bool cmd_teleop(const json & c, const std::string & src, std::string & msg)
  {
    double now = mono();
    if (now < estop_until_) {msg = "e-stop active"; return false;}
    double vx = std::clamp(c.value("vx", 0.0), -TELEOP_MAX_LIN, TELEOP_MAX_LIN);
    double vy = std::clamp(c.value("vy", 0.0), -TELEOP_MAX_LIN, TELEOP_MAX_LIN);
    double wz = std::clamp(c.value("wz", 0.0), -TELEOP_MAX_ANG, TELEOP_MAX_ANG);
    if (!teleop_) {
      std::string took;
      auto active = nav_active();
      if (!active.empty()) {
        cancel_all();
        took = "cancelled Nav2 goal";
      }
      if ((demo_state_ == "FOLLOWING" || demo_state_ == "RETURN_HOME" || demo_state_ == "DIRECT_NAVIGATION") &&
        demo_stop())
      {
        took += std::string(took.empty() ? "" : "; ") + "demo_manager stopped";
      }
      event("CONSOLE", "WARN", "Manual override from " + src + (took.empty() ? "" : " (" + took + ")"));
    }
    teleop_ = true;
    teleop_v_[0] = vx; teleop_v_[1] = vy; teleop_v_[2] = wz;
    teleop_last_ = now;
    teleop_src_ = src;
    msg = "ok";
    return true;
  }

  bool cmd_estop(const std::string & src, std::string & msg)
  {
    teleop_ = false;
    estop_until_ = mono() + ESTOP_ZERO_S;
    int n = cancel_all();
    bool d = demo_stop();
    teleop_pub_->publish(geometry_msgs::msg::Twist());
    event("CONSOLE", "ERROR", "E-STOP from " + src + " (" + std::to_string(n) + " action servers cancelled, demo stop " +
      (d ? "sent" : "n/a") + ")");
    msg = "stopped";
    return true;
  }

  geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped p;
    p.header.frame_id = "map";
    p.header.stamp = now();
    p.pose.position.x = x;
    p.pose.position.y = y;
    p.pose.orientation.z = std::sin(yaw / 2);
    p.pose.orientation.w = std::cos(yaw / 2);
    return p;
  }

  template<class A>
  void watch_goal(typename rclcpp_action::Client<A>::SendGoalOptions & o, const std::string & label)
  {
    o.goal_response_callback = [this, label](typename rclcpp_action::ClientGoalHandle<A>::SharedPtr h) {
        if (!h) {event("CONSOLE", "WARN", label + ": rejected");}
      };
    o.result_callback = [this, label](const typename rclcpp_action::ClientGoalHandle<A>::WrappedResult & r) {
        std::string name = r.code == rclcpp_action::ResultCode::SUCCEEDED ? "SUCCEEDED" :
          r.code == rclcpp_action::ResultCode::CANCELED ? "CANCELED" :
          r.code == rclcpp_action::ResultCode::ABORTED ? "ABORTED" : "UNKNOWN";
        std::string extra;
        if (r.result && r.result->error_code) {
          extra = " (error_code " + std::to_string(r.result->error_code) + " " + r.result->error_msg + ")";
        }
        event("CONSOLE", name == "SUCCEEDED" ? "INFO" : "WARN", label + ": " + name + extra);
        if (mission_.is_object() && mission_.value("label", "") == label) {mission_["status"] = name;}
      };
  }

  bool cmd_goal(const json & c, const std::string & src, std::string & msg)
  {
    double x = c.at("x"), y = c.at("y"), yaw = c.value("yaw", 0.0);
    if (!ac_nav_->action_server_is_ready()) {msg = "navigate_to_pose action server not available"; return false;}
    auto active = nav_active();
    if (std::find(active.begin(), active.end(), "follow_waypoints") != active.end()) {
      ac_wp_->async_cancel_all_goals();
    }
    NavToPose::Goal g;
    g.pose = make_pose(x, y, yaw);
    std::string label = "goal (" + fmt2("%.2f, %.2f", x, y) + fmt(", %.0f°)", yaw * 180 / M_PI);
    rclcpp_action::Client<NavToPose>::SendGoalOptions o;
    watch_goal<NavToPose>(o, label);
    ac_nav_->async_send_goal(g, o);
    mission_ = {{"kind", "goal"}, {"label", label}, {"poses", json::array({{x, y, yaw}})}, {"status", "SENT"}};
    event("CONSOLE", "INFO", label + " sent from " + src);
    msg = "sent";
    return true;
  }

  bool cmd_waypoints(const json & c, const std::string & src, std::string & msg)
  {
    auto poses = c.value("poses", json::array());
    if (poses.empty()) {msg = "no waypoints"; return false;}
    if (!ac_wp_->action_server_is_ready()) {msg = "follow_waypoints action server not available"; return false;}
    FollowWp::Goal g;
    json mp = json::array();
    for (const auto & p : poses) {
      double x = p.at(0), y = p.at(1), yaw = p.size() > 2 ? p.at(2).get<double>() : 0.0;
      g.poses.push_back(make_pose(x, y, yaw));
      mp.push_back({x, y, yaw});
    }
    std::string label = std::to_string(poses.size()) + " waypoints";
    rclcpp_action::Client<FollowWp>::SendGoalOptions o;
    watch_goal<FollowWp>(o, label);
    ac_wp_->async_send_goal(g, o);
    mission_ = {{"kind", "waypoints"}, {"label", label}, {"poses", mp}, {"status", "SENT"}};
    event("CONSOLE", "INFO", label + " sent from " + src);
    msg = "sent";
    return true;
  }

  bool cmd_initialpose(const json & c, const std::string & src, std::string & msg)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped m;
    auto p = make_pose(c.at("x"), c.at("y"), c.value("yaw", 0.0));
    m.header = p.header;
    m.pose.pose = p.pose;
    m.pose.covariance[0] = m.pose.covariance[7] = 0.25;
    m.pose.covariance[35] = 0.0685;
    init_pub_->publish(m);
    event("CONSOLE", "INFO", "initial pose published from " + src);
    msg = "published on /initialpose (used by AMCL; ignored by slam_toolbox mapping)";
    return true;
  }

  bool cmd_clear(std::string & msg)
  {
    int n = 0;
    for (auto & cli : {clear_global_, clear_local_}) {
      if (cli->service_is_ready()) {
        cli->async_send_request(std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>());
        n++;
      }
    }
    event("CONSOLE", "INFO", "clear costmaps (" + std::to_string(n) + "/2)");
    msg = "cleared " + std::to_string(n) + " costmap(s)";
    return n > 0;
  }

  bool cmd_node_info(const json & c, json & data, std::string & msg)
  {
    std::string full = c.value("node", "");
    auto pos = full.rfind('/');
    std::string ns = pos == std::string::npos || pos == 0 ? "/" : full.substr(0, pos);
    std::string name = pos == std::string::npos ? full : full.substr(pos + 1);
    auto g = get_node_graph_interface();
    auto conv = [](const std::map<std::string, std::vector<std::string>> & m) {
        json a = json::array();
        for (auto & [k, v] : m) {
          std::string t;
          for (auto & s : v) {t += (t.empty() ? "" : ", ") + s;}
          a.push_back({k, t});
        }
        return a;
      };
    try {
      data = {{"node", full},
        {"pubs", conv(g->get_publisher_names_and_types_by_node(name, ns))},
        {"subs", conv(g->get_subscriber_names_and_types_by_node(name, ns))},
        {"srvs", conv(g->get_service_names_and_types_by_node(name, ns))}};
      return true;
    } catch (const std::exception & e) {
      msg = e.what();
      return false;
    }
  }

  // ---------------------------------------------------------------- demand
  void set_demand(const json & d)
  {
    for (auto & [k, v] : d.items()) {
      if (!demand_.contains(k)) {sent_.erase(k);}   // newly watched: send a snapshot
    }
    demand_ = d;
    for (auto & [src, topic] : CAMERAS) {
      bool want = demand_.value("cam:" + src, 0.0) > 0;
      if (want && !cam_subs_.count(src)) {
        std::string s = src;
        cam_subs_[src] = create_subscription<sensor_msgs::msg::CompressedImage>(topic, make_qos("sensor", 1),
          [this, s](sensor_msgs::msg::CompressedImage::ConstSharedPtr m) {on_cam(s, *m);});
      } else if (!want && cam_subs_.count(src)) {
        cam_subs_.erase(src);
      }
    }
    std::set<std::string> wanted;
    for (auto & [k, v] : demand_.items()) {
      if (k.rfind("inspect:", 0) == 0) {wanted.insert(k.substr(8));}
    }
    for (auto it = inspect_.begin(); it != inspect_.end(); ) {
      it = wanted.count(it->first) ? std::next(it) : inspect_.erase(it);
    }
    for (auto & t : wanted) {
      if (!inspect_.count(t)) {
        auto m = std::make_shared<Mon>();
        m->topic = t;
        inspect_[t] = m;
      }
    }
  }
  bool wants(const std::string & ch) {return demand_.value(ch, 0.0) > 0;}

  void inspect_subscribe(std::shared_ptr<Mon> m)
  {
    auto topics = get_topic_names_and_types();
    auto it = topics.find(m->topic);
    if (it == topics.end() || it->second.empty()) {return;}
    m->type = it->second[0];
    try {
      m->codec = std::make_shared<TypeCodec>(m->type);
    } catch (const std::exception & e) {
      m->type += std::string(" (cannot load: ") + e.what() + ")";
    }
    bool latched = false;
    for (auto & i : get_publishers_info_by_topic(m->topic)) {
      if (i.qos_profile().get_rmw_qos_profile().durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL) {
        latched = true;
      }
    }
    auto q = latched ? rclcpp::QoS(rclcpp::KeepLast(5)).reliable().transient_local() : make_qos("sensor", 5);
    std::weak_ptr<Mon> w = m;
    m->sub = create_generic_subscription(m->topic, m->type, q,
      [w](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
        if (auto m = w.lock()) {m->tick(msg->size()); m->raw = msg; m->seq++;}
      });
  }

  void on_cam(const std::string & src, const sensor_msgs::msg::CompressedImage & m)
  {
    double now = mono();
    double fps = std::clamp(demand_.value("cam:" + src, 5.0), 1.0, 15.0);
    if (now - cam_last_[src] < 1.0 / fps - 0.01) {return;}
    cam_last_[src] = now;
    emit("cam:" + src, {{"fmt", m.format}}, std::string(m.data.begin(), m.data.end()));
  }

  // ------------------------------------------------------------------ ticks
  void tick()
  {
    // commands from Go
    for (int i = 0; i < 50; ++i) {
      json c;
      {
        std::lock_guard<std::mutex> lk(cmd_mu_);
        if (cmds_.empty()) {break;}
        c = std::move(cmds_.front());
        cmds_.pop();
      }
      handle(c);
    }
    double now = mono();
    // teleop / e-stop at 20 Hz
    if (now < estop_until_) {
      teleop_pub_->publish(geometry_msgs::msg::Twist());
    } else if (teleop_) {
      if (now - teleop_last_ > TELEOP_DEADMAN_S) {
        teleop_ = false;
        for (int i = 0; i < 3; ++i) {teleop_pub_->publish(geometry_msgs::msg::Twist());}
        event("CONSOLE", "INFO", "Manual override released");
      } else {
        geometry_msgs::msg::Twist t;
        t.linear.x = teleop_v_[0]; t.linear.y = teleop_v_[1]; t.angular.z = teleop_v_[2];
        teleop_pub_->publish(t);
      }
    }
    n_++;
    if (n_ % 2 == 0) {emit_pose(now);}                                    // 10 Hz
    if (n_ % 4 == 0) {                                                    // 5 Hz
      emit_motion(now);
      if (wants("scan")) {emit_scan();}
      if (wants("people")) {emit_people(now);}
      if (wants("cam:color") || wants("detections")) {emit_detections(now);}
    }
    if (n_ % 5 == 0 && !log_buf_.empty()) {                               // 4 Hz
      emit("log", {{"items", log_buf_}});
      log_buf_ = json::array();
    }
    if (n_ % 10 == 0) {                                                   // 2 Hz
      for (auto & [t, m] : inspect_) {
        if (!m->sub) {inspect_subscribe(m);}
        m->sample(now);
        emit_inspect(*m, now);
      }
    }
    if (n_ % 20 == 0) {tick_1hz(now);}                                    // 1 Hz
    if (n_ % 100 == 1) {tick_slow();}                                     // 0.2 Hz
  }

  void tick_1hz(double now)
  {
    json rates = json::object();
    for (auto & [t, m] : mons_) {
      m->sample(now);
      rates[t] = {{"hz", num(m->hz, 1)}, {"age", m->age(now)}, {"bw", static_cast<int64_t>(m->bw)},
        {"n", m->count}, {"pubs", m->pubs}};
    }
    emit("rates", rates);
    for (auto & [k, e] : edges_) {
      e.hist.push_back({now, static_cast<double>(e.count)});
      while (e.hist.size() > 6) {e.hist.pop_front();}
      if (now - e.hist.front().first > 0.5) {
        e.hz = (e.count - e.hist.front().second) / (now - e.hist.front().first);
      }
    }
    emit_tf(now);
    evaluate_health(now);
    if (wants("map")) {emit_grid("map", *mons_["/map"]);}
    if (wants("costmap_local")) {emit_grid("costmap_local", *mons_["/local_costmap/costmap"]);}
    if (wants("costmap_global") && (n_ / 20) % 2 == 0) {emit_grid("costmap_global", *mons_["/global_costmap/costmap"]);}
    if (wants("plan")) {emit_plan();}
    if (!diag_.empty()) {
      json a = json::array();
      for (auto & [k, v] : diag_) {
        json x = v;
        x["age"] = num(now - v["t"].get<double>(), 1);
        a.push_back(x);
      }
      emit("diag", a);
    }
  }

  void tick_slow()
  {
    auto topics = get_topic_names_and_types();
    json tl = json::array();
    for (auto & [name, types] : topics) {
      std::string ty;
      for (auto & s : types) {ty += (ty.empty() ? "" : ", ") + s;}
      size_t p = count_publishers(name), s = count_subscribers(name);
      tl.push_back({{"name", name}, {"type", ty}, {"pubs", p}, {"subs", s}});
      auto it = mons_.find(name);
      if (it != mons_.end()) {it->second->pubs = p;}
    }
    for (auto & [t, m] : mons_) {
      if (!topics.count(t)) {m->pubs = 0;}
    }
    auto nodes = get_node_names();
    std::sort(nodes.begin(), nodes.end());
    emit("graph", {{"nodes", nodes}, {"topics", tl}});

    for (const auto & l : LIFECYCLE) {
      std::string name = l.node;
      auto & cli = lc_cli_[name];
      if (!cli->service_is_ready()) {
        lc_[name] = "unavailable";
        continue;
      }
      cli->async_send_request(std::make_shared<lifecycle_msgs::srv::GetState::Request>(),
        [this, name](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture f) {
          try {lc_[name] = f.get()->current_state.label;} catch (...) {lc_[name] = "error";}
        });
    }
    emit("lifecycle", lc_);
  }

  // --------------------------------------------------------------- emitters
  void emit_pose(double now)
  {
    auto p = lookup2d("map", "base_link");
    json v = nullptr;
    nav_msgs::msg::Odometry od;
    auto & m = *mons_["/odometry/filtered"];
    if (m.get(od)) {
      v = {num(od.twist.twist.linear.x), num(od.twist.twist.linear.y), num(od.twist.twist.angular.z)};
    }
    emit("pose", {{"ok", p.has_value()},
      {"p", p ? json({num(p->x), num(p->y), num(p->yaw)}) : json(nullptr)},
      {"v", v}, {"age", m.age(now)}});
  }

  json stage(const std::string & topic, double now)
  {
    auto & c = cmd_[topic];
    auto & m = *mons_[topic];
    return {{"v", {num(c.v[0]), num(c.v[1]), num(c.v[2])}}, {"age", m.age(now)}, {"hz", num(m.hz, 1)},
      {"nz", c.nz < 0 ? json(nullptr) : num(now - c.nz, 1)}, {"pubs", m.pubs}};
  }

  void emit_motion(double now)
  {
    auto active = nav_active();
    json nav = {{"active", active}, {"goals", json::object()}, {"mission", mission_}};
    for (auto & [a, st] : nav_) {
      if (!st.goals.empty()) {
        nav["goals"][a] = STATUS_NAMES[std::clamp<int>(st.goals.rbegin()->second, 0, 6)];
      }
    }
    json fb = nullptr;
    for (const char * a : {"follow_waypoints", "navigate_through_poses", "navigate_to_pose"}) {
      if (std::find(active.begin(), active.end(), a) != active.end() && nav_[a].fb_t > 0) {
        fb = nav_[a].fb;
        // waypoint mission: distance/recoveries come from the inner navigate_to_pose
        if (std::string(a) == "follow_waypoints" && nav_["navigate_to_pose"].fb_t > 0) {
          for (auto & [k, v] : nav_["navigate_to_pose"].fb.items()) {
            if (k != "action") {fb[k] = v;}
          }
        }
        break;
      }
    }
    nav["fb"] = fb;
    nav["ext_goal"] = (ext_goal_ && now - ext_goal_t_ < 600) ?
      json({num(ext_goal_->x), num(ext_goal_->y), num(ext_goal_->yaw)}) : json(nullptr);

    json col = nullptr;
    nav2_msgs::msg::CollisionMonitorState cm;
    if (mons_["/collision_monitor_state"]->get(cm)) {
      col = {{"action", COLLISION_ACTIONS[std::min<int>(cm.action_type, 4)]}, {"poly", cm.polygon_name},
        {"age", mons_["/collision_monitor_state"]->age(now)}};
    }
    json son = nullptr;
    std_msgs::msg::Float32 us;
    if (mons_["/ultrasonic"]->get(us)) {
      son = {{"d", num(us.data * 0.01, 3)}, {"age", mons_["/ultrasonic"]->age(now)}};
    }
    json od = nullptr;
    nav_msgs::msg::Odometry o;
    if (mons_["/odometry/filtered"]->get(o)) {
      od = {{"v", {num(o.twist.twist.linear.x), num(o.twist.twist.linear.y), num(o.twist.twist.angular.z)}},
        {"age", mons_["/odometry/filtered"]->age(now)}};
    }
    json bt = json::array();
    if (!active.empty()) {
      for (auto & [n, s] : bt_status_) {
        if (s == "RUNNING") {bt.push_back(n);}
      }
    }
    json bl = json::array();
    for (auto & [label, b] : blockers_) {
      if (now - b.t < 5.0) {
        bl.push_back({{"label", label}, {"owner", b.owner}, {"age", num(now - b.t, 1)}, {"text", b.text},
          {"n", b.n}});
      }
    }
    emit("motion", {
      {"demo", {{"state", demo_state_.empty() ? json(nullptr) : json(demo_state_)}, {"sel", demo_selected_},
        {"pubs", mons_["/demo/state"]->pubs}}},
      {"nav", nav}, {"bt", bt},
      {"nav_cmd", stage("/cmd_vel_nav", now)}, {"smoothed", stage("/cmd_vel_smoothed", now)},
      {"out", stage("/cmd_vel", now)}, {"collision", col}, {"sonar", son}, {"odom", od}, {"blockers", bl},
      {"teleop", teleop_ ? json({{"v", {num(teleop_v_[0]), num(teleop_v_[1]), num(teleop_v_[2])}},
        {"src", teleop_src_}}) : json(nullptr)},
      {"estop", now < estop_until_},
    });
  }

  void emit_scan()
  {
    auto & m = *mons_["/scan"];
    if (!m.raw || sent_["scan"] == m.seq) {return;}
    sensor_msgs::msg::LaserScan s;
    if (!m.get(s)) {return;}
    auto pose = lookup2d("map", strip(s.header.frame_id));
    if (!pose) {return;}
    sent_["scan"] = m.seq;
    size_t n = s.ranges.size();
    size_t step = std::max<size_t>(1, n / 720);
    std::vector<float> xy;
    xy.reserve(2 * (n / step + 1));
    for (size_t i = 0; i < n; i += step) {
      float r = s.ranges[i];
      if (!std::isfinite(r) || r <= s.range_min || r >= s.range_max) {continue;}
      double a = s.angle_min + i * s.angle_increment + pose->yaw;
      xy.push_back(static_cast<float>(pose->x + r * std::cos(a)));
      xy.push_back(static_cast<float>(pose->y + r * std::sin(a)));
    }
    emit("scan", {{"n", xy.size() / 2}, {"beams", n}, {"frame", s.header.frame_id},
      {"xy", b64(reinterpret_cast<const uint8_t *>(xy.data()), xy.size() * sizeof(float))}});
  }

  void emit_grid(const std::string & ch, Mon & m)
  {
    if (!m.raw || sent_[ch] == m.seq) {return;}
    nav_msgs::msg::OccupancyGrid g;
    if (!m.get(g)) {return;}
    std::string frame = strip(g.header.frame_id);
    if (frame.empty()) {frame = "map";}
    std::optional<Pose2> tf = frame == "map" ? std::optional<Pose2>(Pose2{}) : lookup2d("map", frame);
    if (!tf) {return;}
    sent_[ch] = m.seq;
    auto & o = g.info.origin;
    Pose2 org = compose(*tf, Pose2{o.position.x, o.position.y, yaw_of(o.orientation)});
    emit(ch, {{"w", g.info.width}, {"h", g.info.height}, {"res", num(g.info.resolution, 4)},
      {"o", {num(org.x, 4), num(org.y, 4), num(org.yaw, 4)}}, {"frame", frame},
      {"data", b64(reinterpret_cast<const uint8_t *>(g.data.data()), g.data.size())}});
  }

  void emit_plan()
  {
    auto & m = *mons_["/plan"];
    if (!m.raw || sent_["plan"] == m.seq) {return;}
    nav_msgs::msg::Path p;
    if (!m.get(p)) {return;}
    sent_["plan"] = m.seq;
    size_t n = p.poses.size();
    size_t step = std::max<size_t>(1, n / 300);
    json pts = json::array();
    for (size_t i = 0; i < n; i += step) {
      pts.push_back({num(p.poses[i].pose.position.x), num(p.poses[i].pose.position.y)});
    }
    if (n && (n - 1) % step) {pts.push_back({num(p.poses.back().pose.position.x), num(p.poses.back().pose.position.y)});}
    emit("plan", {{"frame", p.header.frame_id}, {"n", n}, {"pts", pts}, {"age", num(mono() - m.last, 1)}});
  }

  void emit_people(double now)
  {
    json out = {{"ai", json::array()}, {"demo", json::array()}, {"sel", demo_selected_},
      {"state", demo_state_.empty() ? json(nullptr) : json(demo_state_)}};
    auto & m = *mons_["/person_infos"];
    if (m.raw && person_codec_) {
      try {
        json j = person_codec_->decode(*m.raw, 64);
        std::string frame = strip(j["header"]["frame_id"].get<std::string>());
        std::optional<tf2::Transform> T;
        try {
          auto t = tf_buffer_->lookupTransform("map", frame, tf2::TimePointZero);
          auto & r = t.transform.rotation;
          auto & tr = t.transform.translation;
          T = tf2::Transform(tf2::Quaternion(r.x, r.y, r.z, r.w), tf2::Vector3(tr.x, tr.y, tr.z));
        } catch (...) {}
        for (auto & p : j["persons"]) {
          if (!p.is_object()) {continue;}
          double x = p["position"]["x"], y = p["position"]["y"], z = p["position"]["z"];
          json item = {{"id", p["tracking_id"]}, {"conf", num(p["confidence"].get<double>(), 2)},
            {"tracked", p["is_tracked"]}, {"cam", {num(x), num(y), num(z)}},
            {"range", num(std::sqrt(x * x + y * y + z * z), 2)}, {"map", nullptr}};
          if (T) {
            tf2::Vector3 w = (*T) * tf2::Vector3(x, y, z);
            item["map"] = {num(w.x()), num(w.y())};
          }
          out["ai"].push_back(item);
        }
        out["ai_age"] = m.age(now);
        out["ai_frame"] = frame;
        out["ai_tf"] = T.has_value();
      } catch (const std::exception & e) {
        out["ai_error"] = e.what();
      }
    }
    visualization_msgs::msg::MarkerArray ma;
    if (mons_["/demo/person_markers"]->get(ma)) {
      std::map<int, std::string> labels;
      for (auto & k : ma.markers) {
        if (k.ns == "ai_person_labels") {labels[k.id] = k.text;}
      }
      for (auto & k : ma.markers) {
        if (k.ns == "ai_persons" && k.action == 0) {
          out["demo"].push_back({{"id", k.id}, {"x", num(k.pose.position.x)}, {"y", num(k.pose.position.y)},
            {"c", {num(k.color.r, 2), num(k.color.g, 2), num(k.color.b, 2)}}, {"text", labels[k.id]},
            {"frame", k.header.frame_id}});
        }
      }
    }
    emit("people", out);
  }

  void emit_detections(double now)
  {
    vision_msgs::msg::Detection2DArray d;
    auto & m = *mons_["/human_detections"];
    if (!m.get(d)) {return;}
    json w = nullptr, h = nullptr;
    sensor_msgs::msg::CameraInfo ci;
    if (mons_["/camera/camera/color/camera_info"]->get(ci)) {w = ci.width; h = ci.height;}
    json boxes = json::array();
    for (auto & det : d.detections) {
      json b = {{"cx", num(det.bbox.center.position.x, 1)}, {"cy", num(det.bbox.center.position.y, 1)},
        {"w", num(det.bbox.size_x, 1)}, {"h", num(det.bbox.size_y, 1)}, {"id", det.id},
        {"score", nullptr}, {"cls", nullptr}};
      if (!det.results.empty()) {
        b["score"] = num(det.results[0].hypothesis.score, 2);
        b["cls"] = det.results[0].hypothesis.class_id;
      }
      boxes.push_back(b);
    }
    emit("detections", {{"w", w}, {"h", h}, {"boxes", boxes}, {"age", m.age(now)}});
  }

  void emit_inspect(Mon & m, double now)
  {
    json d = {{"topic", m.topic}, {"type", m.type}, {"hz", num(m.hz, 2)}, {"bw", static_cast<int64_t>(m.bw)},
      {"n", m.count}, {"age", m.age(now)}, {"size", m.raw ? json(m.raw->size()) : json(nullptr)},
      {"subscribed", static_cast<bool>(m.sub)}};
    if (now - m.info_t > 5.0) {
      m.info_t = now;
      m.pubs_info = json::array();
      m.subs_info = json::array();
      try {
        for (auto & i : get_publishers_info_by_topic(m.topic)) {
          json q = qos_json(i.qos_profile());
          q["node"] = (i.node_namespace() == "/" ? "" : i.node_namespace()) + "/" + i.node_name();
          m.pubs_info.push_back(q);
        }
        for (auto & i : get_subscriptions_info_by_topic(m.topic)) {
          if (i.node_name() == "robot_console") {continue;}
          json q = qos_json(i.qos_profile());
          q["node"] = (i.node_namespace() == "/" ? "" : i.node_namespace()) + "/" + i.node_name();
          m.subs_info.push_back(q);
        }
      } catch (...) {}
    }
    d["pubs"] = m.pubs_info;
    d["subs"] = m.subs_info;
    if (m.raw && m.codec) {
      if (m.raw->size() > 4'000'000) {
        d["msg"] = "<" + std::to_string(m.raw->size()) + " bytes, too large to decode>";
      } else {
        try {d["msg"] = m.codec->decode(*m.raw, 24);} catch (const std::exception & e) {
          d["msg"] = std::string("<decode error: ") + e.what() + ">";
        }
      }
    }
    emit("inspect:" + m.topic, d);
  }

  void emit_tf(double now)
  {
    json out = json::array();
    for (auto & [k, e] : edges_) {
      out.push_back({{"p", e.parent}, {"c", e.child}, {"s", e.is_static}, {"hz", num(e.hz, 1)},
        {"age", (e.is_static || e.last < 0) ? json(nullptr) : num(now - e.last, 2)},
        {"lag", e.is_static ? json(nullptr) : num(e.lag, 3)}, {"gaps", e.gaps},
        {"max_gap", num(e.max_gap, 2)}, {"jumps", e.jumps},
        {"pose", e.pose ? json({num(e.pose->x), num(e.pose->y), num(e.pose->yaw)}) : json(nullptr)}});
    }
    emit("tf", out);
  }

  // ----------------------------------------------------------------- health
  void evaluate_health(double now)
  {
    auto & H = health_;
    H.begin();
    for (auto & [t, mp] : mons_) {
      auto & m = *mp;
      if (std::isnan(m.warn) && std::isnan(m.err) && std::isnan(m.expect)) {continue;}
      std::string id = "topic:" + m.topic, name = m.label + " (" + m.topic + ")";
      std::string hz = fmt("%.1f Hz", m.hz) + (std::isnan(m.expect) ? "" : fmt(" / %.0f expected", m.expect));
      if (!m.type_ok) {
        H.add(id, m.owner, name, "WARN", "", "message type " + m.type + " not available to the console");
      } else if (m.pubs == 0) {
        H.add(id, m.owner, name, m.optional ? "OFF" : "ERROR", "no publisher", "node not running", true);
      } else if (!std::isnan(m.err) && m.hz < m.err) {
        H.add(id, m.owner, name, "ERROR", hz, fmt("below %.1f Hz", m.err));
      } else if (!std::isnan(m.warn) && m.hz < m.warn) {
        H.add(id, m.owner, name, "WARN", hz, fmt("below %.1f Hz", m.warn));
      } else {
        H.add(id, m.owner, name, "OK", hz);
      }
    }
    sensor_msgs::msg::LaserScan s;
    if (mons_["/scan"]->get(s)) {
      double lag = ros_now() - stamp_s(s.header.stamp);
      H.add("scan:stamp", "LIDAR", "Scan timestamp lag", lag < 0.5 ? "OK" : (lag < 1.0 ? "WARN" : "ERROR"),
        fmt("%.2f s", lag), std::to_string(s.ranges.size()) + " beams, frame " + s.header.frame_id);
    }
    auto & mp = *mons_["/map"];
    if (mp.pubs > 1) {
      H.add("map:pubs", "SLAM", "Map publishers", "WARN", std::to_string(mp.pubs), "more than one /map publisher");
    }
    if (mp.last >= 0) {
      H.add("map:age", "SLAM", "Last map update", "OK", fmt("%.0f s ago", now - mp.last),
        "event driven (map_update_interval 5 s, only after movement)");
    }
    for (const auto & c : TF_CHECKS) {
      std::string id = std::string("tf:") + c.parent + ">" + c.child;
      auto it = edges_.find(std::string(c.parent) + ">" + c.child);
      if (it == edges_.end()) {
        H.add(id, c.owner, c.label, std::string(c.owner) == "AI" ? "OFF" : "ERROR", "missing",
          "transform never received", true);
        continue;
      }
      auto & e = it->second;
      if (c.is_static) {
        std::string lvl = "OK", detail;
        if (std::string(c.child) == "laser" && e.pose && std::abs(std::abs(e.pose->yaw) - M_PI) > 0.05) {
          lvl = "WARN";
          detail = fmt("yaw %.0f° (expected 180°)", e.pose->yaw * 180 / M_PI);
        }
        H.add(id, c.owner, c.label, lvl, "static", detail);
        continue;
      }
      double age = now - e.last;
      std::string lvl = age <= c.warn ? "OK" : (age <= c.err ? "WARN" : "ERROR");
      std::string detail = fmt("%.1f Hz", e.hz) + ", gaps " + std::to_string(e.gaps) +
        (e.gaps ? fmt(" (max %.1f s)", e.max_gap) : "") +
        (std::string(c.child) == "odom" ? ", jumps " + std::to_string(e.jumps) : "");
      H.add(id, c.owner, c.label, lvl, fmt("age %.2f s", age), detail);
    }
    // stationary drift: not commanded for 6 s -> odom should not move
    if (!odom_hist_.empty() && now - last_nonzero_cmd_ > 6.0 && !teleop_) {
      const std::array<double, 4> * ref = nullptr;
      for (auto & h : odom_hist_) {
        if (now - h[0] <= 5.0) {ref = &h; break;}
      }
      auto & last = odom_hist_.back();
      if (ref && last[0] - (*ref)[0] > 4.0) {
        double d = std::hypot(last[1] - (*ref)[1], last[2] - (*ref)[2]);
        double dy = std::abs(wrap(last[3] - (*ref)[3]));
        H.add("drift", "EKF", "Drift while idle (5 s)", (d > 0.02 || dy > 0.02) ? "WARN" : "OK",
          fmt2("%.1f cm / %.1f°", d * 100, dy * 180 / M_PI), "robot not commanded; odom should stay still");
      }
    }
    for (const auto & l : LIFECYCLE) {
      std::string st = lc_.contains(l.node) ? lc_[l.node].get<std::string>() : "?";
      if (st == "active") {
        H.add(std::string("lc:") + l.node, l.owner, l.node, "OK", "active");
      } else if (st == "unavailable") {
        if (l.required) {H.add(std::string("lc:") + l.node, l.owner, l.node, "ERROR", "not running", "", true);}
      } else if (st != "?") {
        H.add(std::string("lc:") + l.node, l.owner, l.node, "ERROR", st, "lifecycle not active", true);
      }
    }
    // /diagnostics, capped at WARN: the console's own rate/TF checks are
    // authoritative (robot_localization says "No events recorded" at 20 Hz)
    static const char * dn[] = {"OK", "WARN", "ERROR", "STALE"};
    for (auto & [k, v] : diag_) {
      if (now - v["t"].get<double>() > 10) {continue;}
      std::string name = v["name"];
      std::string lname = name;
      std::transform(lname.begin(), lname.end(), lname.begin(), ::tolower);
      std::string owner = (lname.find("ekf") != std::string::npos || lname.find("filter") != std::string::npos) ?
        "EKF" : "NAV2";
      int level = v["level"];
      H.add("diag:" + name, owner, name, level == 0 ? "OK" : "WARN", v["msg"].get<std::string>().substr(0, 60),
        std::string("/diagnostics level ") + dn[std::clamp(level, 0, 3)], true);
    }
    auto & dm = *mons_["/demo/state"];
    if (dm.pubs == 0) {
      H.add("demo:state", "DEMO", "demo_manager state", "OFF", "not running", "", true);
    } else {
      bool bad = demo_state_ == "TARGET_LOST" || demo_state_ == "NAVIGATION_FAILED";
      H.add("demo:state", "DEMO", "demo_manager state", bad ? "WARN" : "OK",
        demo_state_.empty() ? "?" : demo_state_, "selected ID " + std::to_string(demo_selected_), true);
    }
    emit("health", H.items);
  }

  // ------------------------------------------------------------------ state
  double t0_{0};
  uint64_t n_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::map<std::string, Edge> edges_;
  std::deque<std::array<double, 4>> odom_hist_;
  double last_nonzero_cmd_{0};
  std::map<std::string, std::shared_ptr<Mon>> mons_;
  std::map<std::string, CmdVal> cmd_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subs_;
  std::shared_ptr<TypeCodec> person_codec_;
  json log_buf_ = json::array();
  std::map<std::string, std::string> bt_status_;
  std::string demo_state_;
  int demo_selected_{-1};
  std::map<std::string, json> diag_;
  std::optional<Pose2> ext_goal_;
  double ext_goal_t_{0};
  std::map<std::string, NavState> nav_;
  json mission_ = nullptr;
  rclcpp_action::Client<NavToPose>::SharedPtr ac_nav_;
  rclcpp_action::Client<FollowWp>::SharedPtr ac_wp_;
  rclcpp_action::Client<NavThrough>::SharedPtr ac_through_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr demo_stop_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_, clear_local_;
  std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> lc_cli_;
  json lc_ = json::object();
  struct Blocker {std::string owner, text; double t{0}; int n{0};};
  std::map<std::string, Blocker> blockers_;
  std::vector<std::pair<std::regex, std::optional<std::regex>>> blocker_re_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr teleop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pub_;
  bool teleop_{false};
  double teleop_v_[3]{0, 0, 0};
  double teleop_last_{0};
  std::string teleop_src_;
  double estop_until_{0};
  json demand_ = json::object();
  std::map<std::string, uint64_t> sent_;
  std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> cam_subs_;
  std::map<std::string, double> cam_last_;
  std::map<std::string, std::shared_ptr<Mon>> inspect_;
  Health health_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex cmd_mu_;
  std::queue<json> cmds_;
};

int main(int argc, char ** argv)
{
  // Keep the real stdout for frames; everything else printed to fd 1
  // (rcutils console output, libraries) goes to stderr, which Go logs.
  g_out_fd = dup(1);
  dup2(2, 1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Gateway>();
  std::thread reader([node]() {
      std::string line;
      while (std::getline(std::cin, line)) {
        if (line.empty()) {continue;}
        try {
          node->push_command(json::parse(line));
        } catch (const std::exception & e) {
          std::cerr << "bad command: " << e.what() << std::endl;
        }
      }
      _exit(0);  // Go went away
    });
  reader.detach();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
