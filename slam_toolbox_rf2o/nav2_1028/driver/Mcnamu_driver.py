#!/usr/bin/env python3
# encoding: utf-8

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool, Int32MultiArray,Float32
from yahboomcar_msgs.msg import ServoControl
from Raspbot_Lib import Raspbot, LightShow
import time

# ---------------------------------------------------------------------------
# Per-robot chassis calibration.  Edit these when copying to another robot.
# ---------------------------------------------------------------------------
# Motor ids (Ctrl_Muto): 0, 1 = LEFT side, 2, 3 = RIGHT side.
#
# STRAIGHT_TRIM_*: compensates a robot that curves while told to go straight.
#   > 0  boosts the RIGHT wheels by that fraction  (robot drifts RIGHT -> use > 0)
#   < 0  boosts the LEFT  wheels by that fraction  (robot drifts LEFT  -> use < 0)
#   Only the weaker side is boosted, so the slow side never drops into the
#   motor deadband (~PWM 33).  Forward and reverse drift differ on mecanum
#   bases, so each has its own value.  Pure in-place rotation (vx = 0) is not
#   trimmed, so the Nav2 rotation calibration (wz_floor, rot_rate) stays valid.
#   How to tune: send vx=0.14 for 0.7 s, read the yaw change from /odom_rf2o,
#   raise the trim while the yaw is negative (right turn), lower it if positive.
#   This robot (2026-10-30): trim 0 -> fwd -0.20 rad / rev +0.08 rad per 22 cm.
STRAIGHT_TRIM_FORWARD = 0.60
STRAIGHT_TRIM_REVERSE = 0.30
# Extra per-motor gain [m0, m1, m2, m3], multiplied on top of STRAIGHT_TRIM_*.
MOTOR_GAIN = [1.0, 1.0, 1.0, 1.0]
# cmd_vel -> PWM scaling (Yahboom defaults; PWM range is -255..255).
LINEAR_PWM_PER_MPS = 255.0
SPIN_PWM_PER_RADPS = (117 + 132) / 8
# Stop the motors if no /cmd_vel arrives for this long (MCU latches commands).
CMD_TIMEOUT_S = 0.3
# ---------------------------------------------------------------------------


def side_gains(trim):
    """(left_gain, right_gain) for STRAIGHT_TRIM; only the weaker side is boosted."""
    if trim >= 0.0:
        return 1.0, 1.0 + trim
    return 1.0 - trim, 1.0


class YahboomCarDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.car = Raspbot()
        self.car.Ctrl_Ulatist_Switch(1)
        self.last_cmd_time = time.time()
        self.gain_fwd = self.wheel_gains(STRAIGHT_TRIM_FORWARD)
        self.gain_rev = self.wheel_gains(STRAIGHT_TRIM_REVERSE)
        self.gain_spin = self.wheel_gains(0.0)
        self.get_logger().info(
            f'wheel trim: forward={STRAIGHT_TRIM_FORWARD:+.3f} '
            f'reverse={STRAIGHT_TRIM_REVERSE:+.3f} MOTOR_GAIN={MOTOR_GAIN} -> '
            f'gains fwd {self.gain_fwd} rev {self.gain_rev}')
        self.light_show = LightShow()
        # Create subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.sub_RGBLight = self.create_subscription(Int32MultiArray, "rgblight", self.RGBLightcallback, 100)
        self.sub_BUzzer = self.create_subscription(Bool, "buzzer", self.Buzzercallback, 100)
        self.sub_servo = self.create_subscription(ServoControl, "servo", self.servo_callback, 10)

        # Create publishers
        self.pub_line_sensor = self.create_publisher(Int32MultiArray, "line_sensor", 10)
        self.pub_ultrasonic = self.create_publisher(Float32, "ultrasonic", 10)  
        # self.pub_all_sensors = self.create_publisher(Int32MultiArray, "all_sensors", 10)

        # Timer to publish all sensor data
        self.timer = self.create_timer(0.1, self.pub_data)
        self.watchdog_timer = self.create_timer(0.1, self.watchdog)

    @staticmethod
    def wheel_gains(trim):
        left, right = side_gains(trim)
        return [round(g, 3) for g in (left * MOTOR_GAIN[0], left * MOTOR_GAIN[1],
                                      right * MOTOR_GAIN[2], right * MOTOR_GAIN[3])]

    def cmd_vel_callback(self, msg):
        if not isinstance(msg, Twist): return
        self.last_cmd_time = time.time()
        # Issue linear vel and angular vel
        vx = msg.linear.x * 1.0
        vy = msg.linear.y * 1.0
        vz = msg.angular.z * 1.0

        speed_lr = -vy * LINEAR_PWM_PER_MPS
        speed_fb = vx * LINEAR_PWM_PER_MPS
        speed_spin = -vz * SPIN_PWM_PER_RADPS

        r1 = speed_fb + speed_lr + speed_spin
        r2 = speed_fb - speed_lr + speed_spin
        r3 = speed_fb - speed_lr - speed_spin
        r4 = speed_fb + speed_lr - speed_spin

        if speed_fb > 0.0:
            gain = self.gain_fwd
        elif speed_fb < 0.0:
            gain = self.gain_rev
        else:
            gain = self.gain_spin
        for i, r in enumerate((r1, r2, r3, r4)):
            pwm = max(-255, min(255, round(r * gain[i])))
            self.car.Ctrl_Muto(i, pwm)

    def RGBLightcallback(self, msg):
        # RGBLight control
        if not isinstance(msg, Int32MultiArray): return
        if len(msg.data) != 3:
            self.get_logger().warn("Invalid RGBLight data length. \'data: [0, 0, 0]\'. Turning off lights.")
            self.car.Ctrl_WQ2812_brightness_ALL(0, 0, 0)
        # Execute a specific light effect based on the message value
        R, G, B = msg.data
        self.car.Ctrl_WQ2812_brightness_ALL(R, G, B)

    def Buzzercallback(self, msg):
        if not isinstance(msg, Bool): return
        if msg.data:
            self.car.Ctrl_BEEP_Switch(1)
        else:
            self.car.Ctrl_BEEP_Switch(0)

    def servo_callback(self, msg):
        # Control both servos based on the received message
        if not isinstance(msg, ServoControl): return
        self.car.Ctrl_Servo(1, msg.servo_s1)
        self.car.Ctrl_Servo(2, msg.servo_s2)

    def watchdog(self):
        # Stop the motors if Nav2 stops publishing cmd_vel.
        if time.time() - self.last_cmd_time > CMD_TIMEOUT_S:
            for i in range(4):
                self.car.Ctrl_Muto(i, 0)

    def pub_data(self):
        # Read and publish all sensor data
        # Example: Read line sensor data
        track = self.car.read_data_array(0x0a, 1)
        if not track:
            self.get_logger().warn('I2C read failed (line sensor)')
            return
        track = int(track[0])
        x1 = (track >> 3) & 0x01
        x2 = (track >> 2) & 0x01
        x3 = (track >> 1) & 0x01
        x4 = track & 0x01

        # Example: Add more sensors here
        # sensor2_data = self.read_sensor2_data()
        # sensor3_data = self.read_sensor3_data()

        linesensor_msg = Int32MultiArray(data=[x2, x1, x3, x4])
        self.pub_line_sensor.publish(linesensor_msg)

        raw_H = self.car.read_data_array(0x1b, 1)
        raw_L = self.car.read_data_array(0x1a, 1)
        if not raw_H or not raw_L:
            self.get_logger().warn('I2C read failed (ultrasonic)')
            return
        diss_H = raw_H[0]
        diss_L = raw_L[0]
        dis = diss_H << 8 | diss_L  # 计算距离值
        ultrasonic_msg = Float32()
        ultrasonic_msg.data = float(dis/10)
        self.pub_ultrasonic.publish(ultrasonic_msg)

        # 添加日志输出
        #self.get_logger().info(f'Published line sensor data: {sensor_data.data}')

        

def main(args=None):
    rclpy.init(args=args)
    driver = YahboomCarDriver('driver_node')
    driver.get_logger().info(f'Successfully started the chassis drive...')
    try:
        rclpy.spin(driver)
    except KeyboardInterrupt:
        driver.car.Ctrl_Ulatist_Switch(0)
        time.sleep(0.1)
        driver.car.Ctrl_Car(0, 0, 0)
        driver.car.Ctrl_Car(1, 0, 0)
        driver.car.Ctrl_Car(2, 0, 0)
        driver.car.Ctrl_Car(3, 0, 0)
        driver.get_logger().info(f'Ulatist off')
        driver.car.Ctrl_BEEP_Switch(0)
        driver.car.Ctrl_WQ2812_brightness_ALL(0, 0, 0)
        pass

""" if __name__ == '__main__':
    main() """