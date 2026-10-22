#!/usr/bin/env python3
"""Translation only: the base may move forward/backward and sideways, but it
must never rotate.

The mecanum chassis of the Yahboomcar can translate in any direction
(driver mixing: speed_fb = vx * 255, speed_lr = -vy * 255), so nav2 can
follow a path by sliding sideways instead of turning.  Keeping the heading
constant is much friendlier for the ICP odometry: the laser always sees the
world from the same orientation, so the yaw estimate cannot drift away.

This node sits between the collision monitor and the chassis driver and
strips the angular velocity:

    /cmd_vel          <- collision monitor (nav2)
    /cmd_vel_straight -> chassis driver

It also acts as a watchdog: if nav2 stops publishing, zeros are sent so the
robot does not keep the last velocity forever.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

DEFAULT_INPUT = "/cmd_vel"
DEFAULT_OUTPUT = "/cmd_vel_straight"


class TranslationOnly(Node):
    def __init__(self):
        super().__init__("translation_only")

        self.declare_parameter("input_topic", DEFAULT_INPUT)
        self.declare_parameter("output_topic", DEFAULT_OUTPUT)
        self.declare_parameter("max_linear_speed", 0.0)   # 0 = no limit

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.max_speed = float(self.get_parameter("max_linear_speed").value)

        self.last_rx = 0.0
        self.rotated_out = 0

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            "translation_only: %s -> %s (angular.z is always forced to 0)"
            % (self.in_topic, self.out_topic)
        )

    def publish(self, vx, vy):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = 0.0        # <- no rotation, ever
        self.pub.publish(msg)

    def on_cmd(self, msg):
        self.last_rx = self.get_clock().now().nanoseconds * 1e-9

        vx = float(msg.linear.x)
        vy = float(msg.linear.y)

        if abs(float(msg.angular.z)) > 1e-3:
            self.rotated_out += 1

        if self.max_speed > 0.0:
            speed = (vx * vx + vy * vy) ** 0.5
            if speed > self.max_speed:
                k = self.max_speed / speed
                vx *= k
                vy *= k

        self.publish(vx, vy)

    def watchdog(self):
        """Send zeros when nav2 stops publishing (the driver has no timeout)."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_rx > 0.5:
            self.publish(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = TranslationOnly()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
