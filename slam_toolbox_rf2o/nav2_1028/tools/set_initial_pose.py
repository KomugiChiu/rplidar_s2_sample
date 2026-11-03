#!/usr/bin/env python3
"""Publish a 2D initial pose for AMCL (nav2_amcl).

    ./set_initial_pose.py X Y YAW_DEG
    ./set_initial_pose.py 0.358 0.074 171

The pose is expressed in the map frame for base_footprint (what RViz's
"2D Pose Estimate" sends).  Keep AMCL running; it will update map->odom.

Covariance defaults: 0.25 m^2 in x/y, 0.0685 rad^2 in yaw (= 15 deg std).
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("set_initial_pose")
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("x", type=float, help="metres, map frame")
    ap.add_argument("y", type=float, help="metres, map frame")
    ap.add_argument("yaw_deg", type=float, help="degrees, map frame")
    ap.add_argument("--xy-cov", type=float, default=0.25)
    ap.add_argument("--yaw-cov", type=float, default=0.0685)
    ap.add_argument("--frame", default="map")
    args = ap.parse_args()

    rclpy.init()
    node = InitialPosePublisher()

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = args.frame
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    yaw = math.radians(args.yaw_deg)
    msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
    cov = [0.0] * 36
    cov[0] = cov[7] = args.xy_cov
    cov[35] = args.yaw_cov
    msg.pose.covariance = cov

    # Wait until AMCL has matched the subscription (its /initialpose
    # subscription is VOLATILE, so samples published before matching are lost),
    # then publish repeatedly for a few seconds.
    deadline = time.time() + 10.0
    while time.time() < deadline and node.pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    matched = node.pub.get_subscription_count()
    end = time.time() + 3.0
    while time.time() < end:
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)
    node.get_logger().info(
        f"sent initial pose x={args.x:.3f} y={args.y:.3f} yaw={args.yaw_deg:.1f} deg "
        f"(frame {args.frame}, subscribers matched={matched})"
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
