#!/usr/bin/env python3
"""Find the robot pose by matching /scan against the live /map, then set AMCL.

Usage on the board:
    ./relocalize.py                 # search, publish /initialpose, verify
    ./relocalize.py --no-publish    # search only (dry run)

It subscribes /map with TRANSIENT_LOCAL (as published by map_server or
slam_toolbox), takes one /scan, builds a likelihood field (distance transform)
and searches (x, y, yaw).  Coarse step is 0.1 m / 10 deg, then a local refine
of 0.025 m / 1 deg around the best candidates.

The laser is mounted with yaw +180 deg relative to base_footprint, so
    base_yaw = laser_yaw - pi
which is what gets published on /initialpose.

Options:
    --map-topic /map          --scan-topic /scan
    --step 2                  subsample free cells (2 = 0.1 m at 0.05 m/cell)
    --yaw-step 10             coarse yaw step in degrees
    --beam-step 6             use every Nth laser beam
    --no-publish              do not send /initialpose
    --timeout 20              seconds to wait for map/scan
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


def yaw_to_quat(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Relocalizer(Node):
    def __init__(self, args):
        super().__init__("relocalize")
        self.args = args
        self.map_msg = None
        self.scan_msg = None
        self.amcl_msg = None

        map_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        scan_qos = QoSProfile(depth=10, history=QoSHistoryPolicy.KEEP_LAST,
                              reliability=QoSReliabilityPolicy.BEST_EFFORT)
        pose_qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                              reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(OccupancyGrid, args.map_topic, self.on_map, map_qos)
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, scan_qos)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose",
                                 self.on_amcl, pose_qos)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose",
                                         pose_qos)
        self.nomotion = self.create_client(Empty, "/request_nomotion_update")

    def on_map(self, msg):
        self.map_msg = msg

    def on_scan(self, msg):
        if self.scan_msg is None:
            self.scan_msg = msg

    def on_amcl(self, msg):
        self.amcl_msg = msg

    def wait_data(self, timeout):
        end = time.time() + timeout
        while time.time() < end and (self.map_msg is None or self.scan_msg is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.map_msg is not None and self.scan_msg is not None


def build_field(grid: OccupancyGrid):
    res = grid.info.resolution
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    w, h = grid.info.width, grid.info.height
    data = np.array(grid.data, dtype=np.int8).reshape(h, w)
    occ = data >= 65
    free = data == 0
    dt = distance_transform_edt(~occ) * res
    return res, ox, oy, w, h, occ, free, dt


def score_batch(dt, res, ox, oy, w, h, rng, ang, px, py, yaw):
    c = np.cos(yaw + ang)[None, :]
    s = np.sin(yaw + ang)[None, :]
    ex = px[:, None] + rng[None, :] * c
    ey = py[:, None] + rng[None, :] * s
    col = np.floor((ex - ox) / res).astype(np.int32)
    row = np.floor((ey - oy) / res).astype(np.int32)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    d = np.full(ex.shape, 1.0)
    d[inside] = dt[row[inside], col[inside]]
    return d.mean(axis=1), (d < 0.15).mean(axis=1)


def search(node, args):
    res, ox, oy, w, h, occ, free, dt = build_field(node.map_msg)
    scan = node.scan_msg
    ranges = np.array([np.nan if not math.isfinite(r) else r for r in scan.ranges],
                      dtype=float)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    valid = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < 10.0)
    idx = np.arange(0, len(ranges), args.beam_step)
    idx = idx[valid[idx]]
    rng = ranges[idx]
    ang = angles[idx]
    print(f"map {w}x{h} @ {res:.3f} m   beams {len(idx)}   range {rng.min():.2f}..{rng.max():.2f} m")

    ys, xs = np.nonzero(free)
    sel = (xs % args.step == 0) & (ys % args.step == 0)
    xs, ys = xs[sel], ys[sel]
    wx = ox + (xs + 0.5) * res
    wy = oy + (ys + 0.5) * res
    print(f"candidate positions {len(wx)}")

    best = []
    yaws = np.deg2rad(np.arange(-180, 180, args.yaw_step))
    for yi, yaw in enumerate(yaws):
        for k in range(0, len(wx), 4000):
            px, py = wx[k:k + 4000], wy[k:k + 4000]
            sc, hit = score_batch(dt, res, ox, oy, w, h, rng, ang, px, py, yaw)
            j = int(np.argmin(sc))
            best.append((float(sc[j]), float(hit[j]), float(px[j]), float(py[j]), float(yaw)))
        if yi % 6 == 0:
            print(f"  coarse yaw {math.degrees(yaw):+4.0f} deg  best {best[-1][0]:.3f} m", flush=True)

    best.sort(key=lambda t: t[0])
    refined = []
    for sc0, hit0, x0, y0, yaw0 in best[:5]:
        gx, gy = np.meshgrid(np.arange(-0.10, 0.101, 0.025), np.arange(-0.10, 0.101, 0.025))
        gx = x0 + gx.ravel()
        gy = y0 + gy.ravel()
        for dyaw in np.deg2rad(np.arange(-10, 10.1, 1.0)):
            sc, hit = score_batch(dt, res, ox, oy, w, h, rng, ang, gx, gy, yaw0 + dyaw)
            j = int(np.argmin(sc))
            refined.append((float(sc[j]), float(hit[j]), float(gx[j]), float(gy[j]), float(yaw0 + dyaw)))
    refined.sort(key=lambda t: t[0])
    return refined


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--step", type=int, default=2)
    ap.add_argument("--yaw-step", type=float, default=10.0)
    ap.add_argument("--beam-step", type=int, default=6)
    ap.add_argument("--timeout", type=float, default=20.0)
    ap.add_argument("--no-publish", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    node = Relocalizer(args)
    if not node.wait_data(args.timeout):
        print("ERROR: no /map or /scan (timeout)")
        return 1

    t0 = time.time()
    refined = search(node, args)
    print(f"search took {time.time() - t0:.1f} s\n")
    for sc, hit, x, y, yaw in refined[:5]:
        b = math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))
        print(f"  score {sc:.3f} m  hit {hit*100:5.1f}%  base x={x:+.3f} y={y:+.3f} yaw={math.degrees(b):+6.1f} deg")

    sc, hit, x, y, yaw = refined[0]
    base_yaw = math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi))
    print(f"\nBEST base: x={x:+.3f} y={y:+.3f} yaw={math.degrees(base_yaw):+.1f} deg "
          f"({sc:.3f} m, {hit*100:.1f}% of beams within 0.15 m)")

    if args.no_publish:
        print("--no-publish: not sending /initialpose")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    qz, qw = yaw_to_quat(base_yaw)
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.25
    cov[35] = 0.0685
    msg.pose.covariance = cov

    deadline = time.time() + 10.0
    while time.time() < deadline and node.pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"publishing /initialpose (subscribers={node.pub.get_subscription_count()}) ...")
    end = time.time() + 3.0
    while time.time() < end:
        msg.header.stamp = node.get_clock().now().to_msg()
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.nomotion.service_is_ready() or node.nomotion.wait_for_service(timeout_sec=3.0):
        try:
            node.nomotion.call_async(Empty.Request())
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass

    end = time.time() + 2.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.amcl_msg is not None:
        p = node.amcl_msg.pose.pose.position
        q = node.amcl_msg.pose.pose.orientation
        amcl_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        print(f"AMCL now  : x={p.x:+.3f} y={p.y:+.3f} yaw={math.degrees(amcl_yaw):+.1f} deg")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
