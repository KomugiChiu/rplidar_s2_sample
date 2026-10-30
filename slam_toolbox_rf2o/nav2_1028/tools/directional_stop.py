#!/usr/bin/env python3
"""Directional forward stop after Nav2's collision_monitor.

Nav2's collision_monitor "stop" polygon zeros the whole Twist, so the robot
cannot rotate or reverse to escape when an obstacle is close.  This node only
blocks *forward* motion when the laser sees something inside the stop
distance; angular velocity and reverse are passed through.

  collision_monitor -> /cmd_vel -> directional_stop -> /cmd_vel_safe -> driver

Parameters:
  input_topic        default /cmd_vel
  output_topic       default /cmd_vel_safe
  scan_topic         default /scan
  stop_distance      default 0.30 m
  scan_forward_angle default pi (laser is mounted 180 deg)
  scan_half_width    default 0.61 rad (~35 deg)
  hold_time          default 0.3 s
  watchdog_timeout   default 0.5 s
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class DirectionalStop(Node):
    def __init__(self):
        super().__init__("directional_stop")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("stop_distance", 0.30)
        self.declare_parameter("scan_forward_angle", 3.14159265)
        self.declare_parameter("scan_half_width", 0.61)
        self.declare_parameter("hold_time", 0.3)
        self.declare_parameter("watchdog_timeout", 0.5)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.stop_distance = abs(float(self.get_parameter("stop_distance").value))
        self.forward_angle = float(self.get_parameter("scan_forward_angle").value)
        self.half_width = abs(float(self.get_parameter("scan_half_width").value))
        self.hold_time = float(self.get_parameter("hold_time").value)
        self.timeout = float(self.get_parameter("watchdog_timeout").value)

        self.last_rx = 0.0
        self.blocked_until = 0.0
        self.last_block_log = 0.0

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            "directional_stop: %s -> %s (forward %.2f m, sector %.0f deg +/- %.0f deg, hold %.2f s)"
            % (self.in_topic, self.out_topic, self.stop_distance,
               math.degrees(self.forward_angle), math.degrees(self.half_width),
               self.hold_time))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, msg):
        inc = float(msg.angle_increment)
        if not math.isfinite(inc) or inc == 0.0 or not msg.ranges:
            return
        best = float("inf")
        for i, r in enumerate(msg.ranges):
            r = float(r)
            if not math.isfinite(r) or r <= 0.06:
                continue
            a = float(msg.angle_min) + i * inc - self.forward_angle
            a = (a + math.pi) % (2.0 * math.pi) - math.pi
            if abs(a) <= self.half_width and r < best:
                best = r
        if best < self.stop_distance:
            self.blocked_until = max(self.blocked_until, self.now() + self.hold_time)
            if self.now() - self.last_block_log > 1.0:
                self.last_block_log = self.now()
                self.get_logger().warn(
                    "forward stop: %.2f m < %.2f m (rotation/reverse still allowed)"
                    % (best, self.stop_distance))

    def blocked(self):
        return self.now() < self.blocked_until

    def on_cmd(self, msg):
        self.last_rx = self.now()
        out = Twist()
        out.linear.x = float(msg.linear.x)
        out.linear.y = float(msg.linear.y)
        out.angular.z = float(msg.angular.z)
        # only block forward; rotation and reverse are allowed to escape
        if self.blocked() and out.linear.x > 0.0:
            out.linear.x = 0.0
        self.pub.publish(out)

    def watchdog(self):
        if self.now() - self.last_rx > self.timeout:
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DirectionalStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
