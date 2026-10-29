#!/usr/bin/env python3
"""Record every Nav2 cmd_vel stage to a CSV file.

Topics:
  /cmd_vel_nav       controller_server (MPPI/shim) raw output
  /cmd_vel_smoothed  velocity_smoother output
  /cmd_vel           collision_monitor output (what the chassis driver gets)

Also records /odometry/filtered so the commands can be compared with motion.

Usage (board):
  source /opt/ros/jazzy/setup.bash
  python3 record_cmd_vel.py /tmp/nav2_cmd.csv          # Ctrl+C to stop
  timeout 60 python3 record_cmd_vel.py /tmp/nav2_cmd.csv

CSV columns:
  t_unix, topic, vx, vy, wz, x, y, yaw
  - cmd rows : topic = nav | smoothed | out, vx/vy/wz filled
  - odom rows: topic = odom, x/y/yaw filled, vx/vy/wz = odom twist
"""
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CmdLogger(Node):
    def __init__(self, path):
        super().__init__("cmd_vel_logger")
        self.f = open(path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow(["t_unix", "topic", "vx", "vy", "wz", "x", "y", "yaw"])
        self.last_flush = time.time()

        self.create_subscription(Twist, "/cmd_vel_nav", lambda m: self.cb("nav", m), 50)
        self.create_subscription(
            Twist, "/cmd_vel_smoothed", lambda m: self.cb("smoothed", m), 50)
        self.create_subscription(Twist, "/cmd_vel", lambda m: self.cb("out", m), 50)
        self.create_subscription(
            Odometry, "/odometry/filtered", self.cb_odom, 50)
        self.get_logger().info("recording to %s" % path)

    def row(self, topic, vx, vy, wz, x="", y="", yaw=""):
        self.w.writerow([
            "%.6f" % time.time(), topic,
            "%.6f" % vx, "%.6f" % vy, "%.6f" % wz, x, y, yaw,
        ])
        now = time.time()
        if now - self.last_flush > 1.0:
            self.f.flush()
            self.last_flush = now

    def cb(self, name, m):
        self.row(name, m.linear.x, m.linear.y, m.angular.z)

    def cb_odom(self, m):
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        yaw = 2.0 * math.atan2(q.z, q.w)
        self.row("odom", m.twist.twist.linear.x, m.twist.twist.linear.y,
                 m.twist.twist.angular.z,
                 "%.6f" % p.x, "%.6f" % p.y, "%.6f" % yaw)

    def close(self):
        self.f.flush()
        self.f.close()


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/nav2_cmd.csv"
    rclpy.init()
    n = CmdLogger(path)
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.close()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
