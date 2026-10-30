#!/usr/bin/env python3
"""Minimum-command floor between Nav2 and the chassis driver.

The Yahboomcar MCU has a deadband: commands below ~0.10 m/s (linear) or
~0.20 rad/s (yaw) do not move the wheels at all, while Nav2/MPPI often emits
much smaller values.  This node turns any *nonzero* command below the floor
into the floor value (sign preserved), and passes larger commands through.

  /cmd_vel  ->  speed_floor  ->  /cmd_vel_floor  ->  Mcnamu_driver

It also acts as a cmd_vel watchdog: if no input arrives for
`watchdog_timeout` seconds, it publishes zeros (the driver has no timeout).

Parameters (all settable with --ros-args -p):
  input_topic       default /cmd_vel
  output_topic      default /cmd_vel_floor
  vx_floor          default 0.16   m/s
  wz_floor          default 0.52   rad/s
  vy_floor          default 0.0    m/s
  deadband          default 0.005  (|v| <= deadband -> 0)
  watchdog_timeout  default 0.5    s
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SpeedFloor(Node):
    def __init__(self):
        super().__init__("speed_floor")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_floor")
        self.declare_parameter("vx_floor", 0.16)
        self.declare_parameter("wz_floor", 0.52)
        self.declare_parameter("vy_floor", 0.0)
        self.declare_parameter("deadband", 0.005)
        self.declare_parameter("watchdog_timeout", 0.5)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.vx_floor = abs(float(self.get_parameter("vx_floor").value))
        self.wz_floor = abs(float(self.get_parameter("wz_floor").value))
        self.vy_floor = abs(float(self.get_parameter("vy_floor").value))
        self.deadband = abs(float(self.get_parameter("deadband").value))
        self.timeout = float(self.get_parameter("watchdog_timeout").value)

        self.last_rx = 0.0
        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.timer = self.create_timer(0.05, self.watchdog)

        self.get_logger().info(
            "speed_floor: %s -> %s (vx_floor=%.3f wz_floor=%.3f "
            "vy_floor=%.3f deadband=%.3f watchdog=%.2fs)"
            % (self.in_topic, self.out_topic, self.vx_floor, self.wz_floor,
               self.vy_floor, self.deadband, self.timeout))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def floor_axis(self, value, floor):
        if floor <= 0.0:
            return value
        if abs(value) <= self.deadband:
            return 0.0
        if abs(value) < floor:
            return math.copysign(floor, value)
        return value

    def on_cmd(self, msg):
        self.last_rx = self.now()
        out = Twist()
        out.linear.x = self.floor_axis(float(msg.linear.x), self.vx_floor)
        out.linear.y = self.floor_axis(float(msg.linear.y), self.vy_floor)
        out.angular.z = self.floor_axis(float(msg.angular.z), self.wz_floor)
        self.pub.publish(out)

    def watchdog(self):
        if self.now() - self.last_rx > self.timeout:
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SpeedFloor()
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
