#!/usr/bin/env python3
"""Translation gate between nav2 and the Yahboomcar mecanum chassis.

The mecanum chassis can translate in any direction (driver mixing:
speed_fb = vx * 255, speed_lr = -vy * 255), so nav2 may follow a path by
sliding sideways instead of turning.  In the default "translation only" mode
the angular velocity is stripped, which keeps the heading fixed and is much
friendlier for the ICP odometry: the laser always sees the world from the
same orientation, so the yaw estimate cannot drift away.

Why rotation is off by default: the chassis MCU has a ~0.23 s soft-start and
no usable speed below roughly 0.3 rad/s (measured: a 0.4 s burst gives 11-15
deg, shorter pulses give nothing).  A closed-loop yaw controller therefore
overshoots by ~one burst every correction and the robot oscillates.  Set
allow_rotation:=true only for coarse turns; wz_max_out caps the command so
nav2 cannot ask for a rate the chassis cannot follow.

vx_scale compensates the measured chassis gain (the wheels run ~1.2x faster
than the command asks for), so nav2's velocity model matches reality.

The gate also enforces an ultrasonic safety stop: when /ultrasonic reports an
obstacle closer than ultrasonic_stop_distance the output is forced to zero
for ultrasonic_hold_time.  This catches chairs / table legs / feet that the
2D laser plane misses (the chair collision of 2026-10-27).  It only applies
while driving forward; reversing is always allowed so the robot can back out.

Strafing (vy) is disabled by default.  The single forward ultrasonic cannot
see the sides, and a diagonal mecanum move drove the robot into a chair on
2026-10-27.  Enable allow_strafe only after adding side/rear obstacle sensing.

A second guard watches the laser's forward sector (physical forward is at
scan angle scan_forward_angle, default pi because the lidar is mounted
reversed).  Anything closer than scan_stop_distance stops forward motion too.
This catches thin obstacles and people that the wide ultrasonic cone misses.

    /cmd_vel          <- collision monitor (nav2)
    /cmd_vel_straight -> chassis driver
    /ultrasonic       <- yahboom MCU (cm)
    /scan             <- rplidar (m)

It also acts as a watchdog: if nav2 stops publishing, zeros are sent so the
robot does not keep the last velocity forever.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

DEFAULT_INPUT = "/cmd_vel"
DEFAULT_OUTPUT = "/cmd_vel_straight"


class TranslationOnly(Node):
    def __init__(self):
        super().__init__("translation_only")

        self.declare_parameter("input_topic", DEFAULT_INPUT)
        self.declare_parameter("output_topic", DEFAULT_OUTPUT)
        self.declare_parameter("max_linear_speed", 0.0)   # 0 = no limit
        self.declare_parameter("allow_rotation", False)   # true = pass angular.z
        self.declare_parameter("allow_strafe", False)     # true = pass linear.y
        # zero the output after this long without a new input command
        self.declare_parameter("watchdog_timeout", 0.5)
        # measured chassis gain: actual speed / commanded speed (~1.2)
        self.declare_parameter("vx_scale", 0.85)
        # hard cap for the command sent to the MCU when rotation is enabled
        self.declare_parameter("wz_max_out", 0.30)
        # ultrasonic safety stop (metres, hold time in seconds)
        self.declare_parameter("use_ultrasonic", True)
        self.declare_parameter("ultrasonic_topic", "/ultrasonic")
        self.declare_parameter("ultrasonic_stop_distance", 0.65)
        self.declare_parameter("ultrasonic_hold_time", 0.5)
        # laser forward-sector safety stop (metres / radians)
        self.declare_parameter("use_scan_stop", True)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_stop_distance", 0.45)
        self.declare_parameter("scan_forward_angle", 3.14159265)
        self.declare_parameter("scan_half_width", 0.61)

        self.in_topic = self.get_parameter("input_topic").value
        self.out_topic = self.get_parameter("output_topic").value
        self.max_speed = float(self.get_parameter("max_linear_speed").value)
        self.allow_rotation = bool(self.get_parameter("allow_rotation").value)
        self.allow_strafe = bool(self.get_parameter("allow_strafe").value)
        self.watchdog_timeout = float(self.get_parameter("watchdog_timeout").value)
        self.vx_scale = float(self.get_parameter("vx_scale").value)
        self.wz_max_out = abs(float(self.get_parameter("wz_max_out").value))
        self.use_ultrasonic = bool(self.get_parameter("use_ultrasonic").value)
        self.sonar_topic = self.get_parameter("ultrasonic_topic").value
        self.sonar_stop = float(self.get_parameter("ultrasonic_stop_distance").value)
        self.sonar_hold = float(self.get_parameter("ultrasonic_hold_time").value)
        self.use_scan_stop = bool(self.get_parameter("use_scan_stop").value)
        self.scan_topic = self.get_parameter("scan_topic").value
        self.scan_stop = float(self.get_parameter("scan_stop_distance").value)
        self.scan_forward = float(self.get_parameter("scan_forward_angle").value)
        self.scan_half = abs(float(self.get_parameter("scan_half_width").value))

        self.last_rx = 0.0
        self.rotated_out = 0
        self.sonar_dist = None
        self.blocked_until = 0.0
        self.last_block_log = 0.0

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.sub = self.create_subscription(Twist, self.in_topic, self.on_cmd, 10)
        if self.use_ultrasonic:
            self.create_subscription(Float32, self.sonar_topic, self.on_sonar, 10)
        if self.use_scan_stop:
            self.create_subscription(LaserScan, self.scan_topic, self.on_scan,
                                     qos_profile_sensor_data)
        self.timer = self.create_timer(0.1, self.watchdog)

        self.get_logger().info(
            "translation_only: %s -> %s (allow_rotation=%s allow_strafe=%s vx_scale=%.2f wz_max_out=%.2f watchdog=%.2fs)"
            % (self.in_topic, self.out_topic, self.allow_rotation,
               self.allow_strafe, self.vx_scale, self.wz_max_out,
               self.watchdog_timeout)
        )
        if self.use_ultrasonic:
            self.get_logger().info(
                "translation_only: ultrasonic stop at %.2f m on %s (hold %.1f s)"
                % (self.sonar_stop, self.sonar_topic, self.sonar_hold)
            )
        if self.use_scan_stop:
            self.get_logger().info(
                "translation_only: scan stop at %.2f m on %s (forward %.0f deg +/- %.0f deg)"
                % (self.scan_stop, self.scan_topic,
                   math.degrees(self.scan_forward), math.degrees(self.scan_half))
            )

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_sonar(self, msg):
        # the MCU publishes centimetres; 0 means "no echo"
        dist = float(msg.data) * 0.01
        if 0.02 <= dist <= 3.0:
            self.sonar_dist = dist
            if dist < self.sonar_stop:
                self.blocked_until = self.now() + self.sonar_hold
                if self.now() - self.last_block_log > 1.0:
                    self.last_block_log = self.now()
                    self.get_logger().warn(
                        "ultrasonic stop: %.2f m < %.2f m" % (dist, self.sonar_stop)
                    )

    def on_scan(self, msg):
        inc = float(msg.angle_increment)
        if not math.isfinite(inc) or inc == 0.0 or not msg.ranges:
            return
        best = float("inf")
        for i, r in enumerate(msg.ranges):
            r = float(r)
            if not math.isfinite(r) or r <= 0.06:
                continue
            a = float(msg.angle_min) + i * inc - self.scan_forward
            a = (a + math.pi) % (2.0 * math.pi) - math.pi
            if abs(a) <= self.scan_half and r < best:
                best = r
        if best < self.scan_stop:
            self.blocked_until = max(self.blocked_until, self.now() + self.sonar_hold)
            if self.now() - self.last_block_log > 1.0:
                self.last_block_log = self.now()
                self.get_logger().warn(
                    "scan stop: %.2f m < %.2f m" % (best, self.scan_stop)
                )

    def blocked(self):
        return self.now() < self.blocked_until

    def publish(self, vx, vy, wz=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz if self.allow_rotation else 0.0
        self.pub.publish(msg)

    def on_cmd(self, msg):
        self.last_rx = self.get_clock().now().nanoseconds * 1e-9

        vx = float(msg.linear.x) * self.vx_scale
        vy = float(msg.linear.y) if self.allow_strafe else 0.0
        wz = float(msg.angular.z) if self.allow_rotation else 0.0
        if wz > self.wz_max_out:
            wz = self.wz_max_out
        elif wz < -self.wz_max_out:
            wz = -self.wz_max_out

        if abs(float(msg.angular.z)) > 1e-3:
            self.rotated_out += 1

        if self.max_speed > 0.0:
            speed = (vx * vx + vy * vy) ** 0.5
            if speed > self.max_speed:
                k = self.max_speed / speed
                vx *= k
                vy *= k

        # ultrasonic safety stop: block forward motion, allow reversing out
        if self.blocked() and vx > -0.01:
            self.publish(0.0, 0.0)
            return

        self.publish(vx, vy, wz)

    def watchdog(self):
        """Send zeros when nav2 stops publishing (the driver has no timeout)."""
        if self.blocked():
            self.publish(0.0, 0.0)
            return
        if self.now() - self.last_rx > self.watchdog_timeout:
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
