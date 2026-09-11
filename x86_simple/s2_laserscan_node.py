#!/usr/bin/env python3
"""S2 serial ROS2 Python node：rclpy + pyrplidarsdk，不依賴 rplidar_ros。
把 get_scan_data() 的 (angle[rad], range[m], quality) 裝箱成 sensor_msgs/LaserScan 發到 /scan。

用法 (需 ROS Jazzy + pyrplidarsdk，見下方 venv 作法):
  ros2 run <pkg> s2_laserscan_node --ros-args -p port:=/dev/ttyUSB0
  或直接: python3 s2_laserscan_node.py
參數: port, baud(預設1000000), frame_id(laser), topic_name(scan), bins(720)
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

try:
    import pyrplidarsdk
except ImportError:
    pyrplidarsdk = None


class S2ScanNode(Node):
    def __init__(self) -> None:
        super().__init__('s2_laserscan_node')
        if pyrplidarsdk is None:
            raise RuntimeError('缺 pyrplidarsdk：pip install pyrplidarsdk')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('topic_name', 'scan')
        self.declare_parameter('bins', 720)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.frame_id = self.get_parameter('frame_id').value
        self.bins = self.get_parameter('bins').value

        self.pub = self.create_publisher(
            LaserScan, self.get_parameter('topic_name').value, 10)

        self.driver = pyrplidarsdk.RplidarDriver(port=port, baudrate=baud)
        if not self.driver.connect():
            raise RuntimeError(f'connect({port}@{baud}) 失敗')
        self.get_logger().info(f'device_info: {self.driver.get_device_info()}')
        self.get_logger().info(f'health: {self.driver.get_health()}')
        if not self.driver.start_scan():
            raise RuntimeError('start_scan 失敗（檢查供電/佔用）')

        self.timer = self.create_timer(0.1, self.on_timer)
        self.get_logger().info('s2_laserscan_node started')

    def on_timer(self) -> None:
        data = self.driver.get_scan_data()
        if not data:
            return
        angles, ranges, qualities = data
        n = self.bins
        binned = [float('inf')] * n
        intens = [0.0] * n
        for a, r, q in zip(angles, ranges, qualities):
            idx = int((a % (2.0 * math.pi)) / (2.0 * math.pi) * n) % n
            if r < binned[idx]:
                binned[idx] = float(r)
                intens[idx] = float(q)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * math.pi
        msg.angle_increment = 2.0 * math.pi / n
        msg.time_increment = 0.1 / n
        msg.scan_time = 0.1
        msg.range_min = 0.15
        msg.range_max = 30.0
        msg.ranges = binned
        msg.intensities = intens
        self.pub.publish(msg)

    def destroy_node(self) -> bool:
        try:
            self.driver.stop_scan()
        except Exception:
            pass
        try:
            self.driver.disconnect()
        except Exception:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = S2ScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
