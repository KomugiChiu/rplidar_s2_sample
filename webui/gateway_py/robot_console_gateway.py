#!/usr/bin/env python3
"""Python fallback of robot_console_gateway (the C++ one lives in ../gateway).

Same stdio protocol and channels, so robot-console (Go) and the UI work with
either.  Start the console with ../start_console_py.sh to use this one.

  stdin : one JSON command per line   {"cmd": "...", "id": 7, ...}
  stdout: frames [u32 hdr_len][u32 payload_len][hdr JSON][payload]
          hdr = {"ch": <channel>, "d": <data>}, payload = JPEG or empty

rclpy costs ~2 ms CPU per callback on this SoC and the cost grows with the
number of entities in the wait set (measured 2026-11-03), so this version:
  * runs ONE 10 Hz timer for everything periodic,
  * uses raw subscriptions (no Python message objects) and reads Twist /
    Float32 straight from the CDR bytes,
  * samples the 200 Hz gyro and the 30 Hz camera_info topics for 1 s every
    15 s instead of subscribing permanently,
  * drops rf2o's per-scan INFO from /rosout with a DDS content filter,
  * has no action feedback subscriptions (distance left / recoveries come from
    /plan and the BT log), one short-lived lifecycle client at a time, and
    creates action / costmap clients only when first used.
Heavy outputs (scan, map, costmaps, camera, inspector) only run on demand.
"""
import base64
import collections
import json
import math
import os
import queue
import re
import struct
import sys
import threading
import time


def _setup_stdio():
    # The real stdout carries frames only; anything printed to fd 1 goes to stderr.
    out = os.fdopen(os.dup(1), 'wb', buffering=0)
    os.dup2(2, 1)
    sys.stdout = sys.stderr
    return out


OUT = _setup_stdio()

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.duration import Duration  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rclpy.time import Time  # noqa: E402
from rosidl_runtime_py import message_to_ordereddict  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402
from tf2_ros.buffer import Buffer  # noqa: E402

from action_msgs.msg import GoalStatusArray  # noqa: E402
from action_msgs.srv import CancelGoal  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav2_msgs.action import FollowWaypoints, NavigateToPose  # noqa: E402
from nav2_msgs.msg import BehaviorTreeLog  # noqa: E402
from nav2_msgs.srv import ClearEntireCostmap  # noqa: E402
from rcl_interfaces.msg import Log  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import Int32, String  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from tf2_msgs.msg import TFMessage  # noqa: E402

try:
    from rclpy.subscription_content_filter_options import ContentFilterOptions
except ImportError:  # pragma: no cover
    ContentFilterOptions = None

# ---------------------------------------------------------------------------
# Configuration (keep in sync with ../gateway/src/gateway.cpp)
# ---------------------------------------------------------------------------
TELEOP_MAX_LIN = 0.35        # m/s
TELEOP_MAX_ANG = 1.5         # rad/s
TELEOP_DEADMAN_S = 0.5       # stop when the browser stops sending keepalives
TELEOP_TOPIC = '/cmd_vel'    # driver input; collision_monitor is silent when idle
ESTOP_ZERO_S = 1.0
SAMPLE_EVERY_S = 15.0        # sampled topics: subscribe for SAMPLE_WINDOW_S every SAMPLE_EVERY_S
SAMPLE_WINDOW_S = 1.0

# mode: 'always'   = permanent subscription
#       'sampled'  = subscribed for SAMPLE_WINDOW_S every SAMPLE_EVERY_S (rate only)
#       'adaptive' = sampled, but continuous while needed (navigation / manual drive
#                    for the cmd_vel chain, an open map for /scan)
# topic, type, qos, owner, label, expected Hz, WARN below, ERROR below, optional, mode
MONITORS = [
    ('/scan', 'sensor_msgs/msg/LaserScan', 'sensor', 'LIDAR', 'Lidar scan', 10, 8, 5, False, 'adaptive'),
    ('/odom_rf2o', 'nav_msgs/msg/Odometry', 'sensor', 'ODOM', 'rf2o odometry', 10, 8, 3, False, 'sampled'),
    ('/odometry/filtered', 'nav_msgs/msg/Odometry', 'sensor', 'EKF', 'EKF odometry', 20, 15, 8, False, 'sampled'),
    ('/camera/camera/gyro/sample', 'sensor_msgs/msg/Imu', 'sensor', 'CAMERA', 'D455 gyro (EKF input)',
     200, 100, 1, False, 'sampled'),
    ('/camera/camera/color/camera_info', 'sensor_msgs/msg/CameraInfo', 'sensor', 'CAMERA', 'Color stream',
     30, 20, 5, True, 'sampled'),
    ('/camera/camera/aligned_depth_to_color/camera_info', 'sensor_msgs/msg/CameraInfo', 'sensor', 'CAMERA',
     'Aligned depth stream', 30, 20, 5, True, 'sampled'),
    ('/ultrasonic', 'std_msgs/msg/Float32', 'sensor', 'CHASSIS', 'Chassis driver (sonar)', 10, 8, 3, False, 'always'),
    ('/person_infos', 'ai_human_detection/msg/PersonInfoArray', 'sensor', 'AI', 'Person positions',
     None, 2, 0.5, True, 'always'),
    ('/human_detections', 'vision_msgs/msg/Detection2DArray', 'sensor', 'AI', '2D detections',
     None, 2, 0.5, True, 'always'),
    ('/demo/person_markers', 'visualization_msgs/msg/MarkerArray', 'sensor', 'DEMO', 'Person markers',
     None, None, None, True, 'always'),
    ('/map', 'nav_msgs/msg/OccupancyGrid', 'latched', 'SLAM', 'Map', None, None, None, False, 'always'),
    ('/plan', 'nav_msgs/msg/Path', 'sensor', 'NAV2', 'Global plan', None, None, None, True, 'always'),
    ('/collision_monitor_state', 'nav2_msgs/msg/CollisionMonitorState', 'sensor', 'NAV2',
     'Collision monitor state', None, None, None, True, 'always'),
    ('/cmd_vel_nav', 'geometry_msgs/msg/Twist', 'sensor', 'NAV2', 'Controller output', None, None, None, False,
     'adaptive'),
    ('/cmd_vel_smoothed', 'geometry_msgs/msg/Twist', 'sensor', 'NAV2', 'Velocity smoother output',
     None, None, None, False, 'adaptive'),
    ('/cmd_vel', 'geometry_msgs/msg/Twist', 'sensor', 'CHASSIS', 'Driver input', None, None, None, False, 'always'),
]
ACTIVE_HOLD_S = 10.0         # cmd_vel chain stays continuous this long after navigation / manual drive
# demand-only grids: channel -> topic
GRIDS = {'costmap_global': '/global_costmap/costmap', 'costmap_local': '/local_costmap/costmap'}
# counted by typed callbacks
TYPED = [('/tf', 'TF', 'TF'), ('/rosout', '', 'rosout'), ('/debug_log', 'AI', 'AI debug log'),
         ('/behavior_tree_log', 'NAV2', 'BT log'), ('/demo/state', 'DEMO', 'Demo state')]

# parent, child, owner, label, WARN age s, ERROR age s, static
TF_CHECKS = [
    ('map', 'odom', 'SLAM', 'map→odom (slam_toolbox)', 0.3, 1.0, False),
    ('odom', 'base_footprint', 'EKF', 'odom→base_footprint (EKF)', 0.15, 0.5, False),
    ('base_footprint', 'base_link', 'TF', 'base_footprint→base_link', None, None, True),
    ('base_link', 'laser', 'TF', 'base_link→laser (yaw 180°)', None, None, True),
    ('base_footprint', 'camera_link', 'TF', 'base_footprint→camera_link', None, None, True),
    ('base_link', 'ai_human_detection', 'AI', 'base_link→ai_human_detection', None, None, True),
]

# node, owner, required
LIFECYCLE = [
    ('controller_server', 'NAV2', True), ('planner_server', 'NAV2', True),
    ('smoother_server', 'NAV2', True), ('behavior_server', 'NAV2', True),
    ('bt_navigator', 'NAV2', True), ('waypoint_follower', 'NAV2', True),
    ('velocity_smoother', 'NAV2', True), ('collision_monitor', 'NAV2', True),
    ('global_costmap/global_costmap', 'NAV2', True), ('local_costmap/local_costmap', 'NAV2', True),
    ('route_server', 'NAV2', False), ('docking_server', 'NAV2', False),
    ('slam_toolbox', 'SLAM', False),
]

CAMERAS = {'color': '/camera/camera/color/image_raw/compressed', 'debug': '/debug_image/compressed'}

# rosout text that explains a stop; shown in the motion pipeline for 5 s.
BLOCKERS = [
    (r'costmap stop', 'NAV2', 'Controller: costmap stop (obstacle ahead)', None),
    (r'ultrasonic stop', 'NAV2', 'Controller: ultrasonic stop', None),
    (r'escape:', 'NAV2', 'Controller: escape manoeuvre', None),
    (r'Robot to stop due to invalid source', 'NAV2', 'collision_monitor: invalid source (TF/scan timing)',
     r'Robot to continue normal operation'),
    (r'Failed to make progress', 'NAV2', 'Progress checker: no progress', None),
    (r'Collision Ahead', 'NAV2', 'Recovery refused: collision ahead', None),
    (r'Transform data too old|xtrapolation', 'TF', 'TF timing error (data too old / extrapolation)', None),
    (r'Message Filter dropping', 'TF', 'Consumer dropping scans (TF lag / queue full)', None),
    (r'Goal (failed|was aborted)|Aborting handle', 'NAV2', 'Nav2 aborted the goal', None),
]
RECOVERY_NODES = {'BackUp', 'Wait', 'Spin', 'ClearLocalCostmap-Subtree', 'ClearGlobalCostmap-Subtree',
                  'DriveOnHeading'}

STATUS_NAMES = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING',
                4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
ACTIVE = (1, 2, 3)
COLLISION_ACTIONS = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH', 4: 'LIMIT'}
LOG_LEVELS = {10: 'D', 20: 'I', 30: 'W', 40: 'E', 50: 'F'}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
_out_lock = threading.Lock()


def emit(ch, data=None, payload=b''):
    try:
        hdr = json.dumps({'ch': ch, 'd': data}, separators=(',', ':'), allow_nan=False)
    except (ValueError, TypeError):
        hdr = json.dumps({'ch': ch, 'd': clean(data)}, separators=(',', ':'))
    hdr = hdr.encode()
    with _out_lock:
        try:
            OUT.write(struct.pack('>II', len(hdr), len(payload)) + hdr + payload)
        except (BrokenPipeError, OSError):
            os._exit(0)  # Go went away


def clean(v, depth=0):
    """JSON-safe copy (NaN -> None, bytes -> length, numpy -> python)."""
    if depth > 12:
        return '…'
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    if isinstance(v, (bool, int, str)) or v is None:
        return v
    if isinstance(v, (bytes, bytearray)):
        return f'<{len(v)} bytes>'
    if isinstance(v, np.generic):
        return clean(v.item(), depth + 1)
    if isinstance(v, np.ndarray):
        return clean(v.tolist(), depth + 1)
    if isinstance(v, dict):
        return {str(k): clean(x, depth + 1) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [clean(x, depth + 1) for x in v]
    return str(v)


def fnum(x, nd=3):
    try:
        x = float(x)
    except (TypeError, ValueError):
        return None
    return round(x, nd) if math.isfinite(x) else None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


def tf2d(t):
    tr = t.transform.translation
    return (tr.x, tr.y, yaw_of(t.transform.rotation))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def qrot(q, v):
    """Rotate vector v by quaternion q (x, y, z, w)."""
    qx, qy, qz, qw = q
    tx = 2.0 * (qy * v[2] - qz * v[1])
    ty = 2.0 * (qz * v[0] - qx * v[2])
    tz = 2.0 * (qx * v[1] - qy * v[0])
    return (v[0] + qw * tx + (qy * tz - qz * ty),
            v[1] + qw * ty + (qz * tx - qx * tz),
            v[2] + qw * tz + (qx * ty - qy * tx))


def cdr_unpack(fmt, raw):
    """Fixed-layout primitives right after the 4-byte CDR encapsulation header."""
    endian = '<' if raw[1] & 1 else '>'
    return struct.unpack_from(endian + fmt, raw, 4)


def qos(kind, depth=10):
    if kind == 'latched':
        return QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    if kind == 'reliable':
        return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                          reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    # best effort + volatile matches every publisher
    return QoSProfile(depth=depth, history=HistoryPolicy.KEEP_LAST,
                      reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)


def qos_json(p):
    def n(e):
        return getattr(e, 'name', str(e)).replace('RMW_QOS_POLICY_', '')
    return {'reliability': n(p.reliability), 'durability': n(p.durability), 'depth': p.depth}


class Mon:
    """Rate / bandwidth / age of one topic plus its latest raw message."""

    def __init__(self, topic, owner='', label='', expect=None, warn=None, err=None, optional=False,
                 mode='always'):
        self.topic, self.owner, self.label = topic, owner, label
        self.expect, self.warn, self.err, self.optional, self.mode = expect, warn, err, optional, mode
        self.continuous = mode == 'always'
        self.type = ''
        self.kind = 'sensor'
        self.cls = None
        self.type_ok = True
        self.sub = None
        self.count = 0
        self.nbytes = 0
        self.last = None
        self.raw = None
        self.seq = 0
        self._msg = None
        self._msg_seq = -1
        self.hist = collections.deque(maxlen=6)
        self.hz = 0.0
        self.bw = 0.0
        self.pubs = 0
        self.win = None              # sampled: (start mono, count at start)
        self.info_t = 0.0
        self.pubs_info = []
        self.subs_info = []

    def tick(self, raw):
        self.count += 1
        self.nbytes += len(raw)
        self.last = time.monotonic()
        self.raw = raw
        self.seq += 1

    @property
    def sampled(self):
        return not self.continuous

    def sample(self, now):
        if not self.continuous:
            return  # rate comes from the sampling window
        self.hist.append((now, self.count, self.nbytes))
        t0, c0, b0 = self.hist[0]
        if now - t0 > 0.5:
            self.hz = (self.count - c0) / (now - t0)
            self.bw = (self.nbytes - b0) / (now - t0)

    def age(self, now):
        return None if self.last is None else now - self.last

    def msg(self):
        if self.raw is None or self.cls is None:
            return None
        if self._msg_seq != self.seq:
            self._msg = deserialize_message(self.raw, self.cls)
            self._msg_seq = self.seq
        return self._msg


class Edge:
    __slots__ = ('parent', 'child', 'static', 'count', 'last', 'lag', 'hist', 'hz',
                 'gaps', 'max_gap', 'pose', 'jumps')

    def __init__(self, parent, child, static):
        self.parent, self.child, self.static = parent, child, static
        self.count = 0
        self.last = None
        self.lag = None
        self.hist = collections.deque(maxlen=6)
        self.hz = 0.0
        self.gaps = 0
        self.max_gap = 0.0
        self.pose = None
        self.jumps = 0


class Health:
    """OK / WARN / ERROR / OFF items; ERROR only after 3 bad samples in a row."""

    def __init__(self):
        self.bad = collections.Counter()
        self.items = []

    def begin(self):
        self.items = []

    def add(self, cid, owner, name, level, value='', detail='', immediate=False):
        if level == 'ERROR' and not immediate:
            self.bad[cid] += 1
            if self.bad[cid] < 3:
                level = 'WARN'
        elif level != 'ERROR':
            self.bad[cid] = 0
        self.items.append({'id': cid, 'owner': owner, 'name': name, 'level': level,
                           'value': value, 'detail': detail})


# ---------------------------------------------------------------------------
# Gateway node
# ---------------------------------------------------------------------------
class Gateway(Node):

    def __init__(self, cmd_queue):
        super().__init__('robot_console')
        self.cmds = cmd_queue
        self.n = 0
        self.demand = {}
        self.health = Health()
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.edges = {}
        self.odom_hist = collections.deque(maxlen=300)
        self.last_nonzero_cmd = 0.0

        self.mons = {}
        for topic, typ, kind, owner, label, exp, warn, err, opt, mode in MONITORS:
            m = Mon(topic, owner, label, exp, warn, err, opt, mode)
            m.type, m.kind = typ, kind
            try:
                m.cls = get_message(typ)
            except Exception as e:  # e.g. ai_human_detection not sourced
                m.type_ok = False
                self.get_logger().warn(f'{topic}: cannot load {typ}: {e}')
            self.mons[topic] = m
            if m.type_ok and mode == 'always':
                self._subscribe_mon(m)
        for topic, owner, label in TYPED:
            self.mons[topic] = Mon(topic, owner, label)
        self.cmd_vals = {t: {'v': (0.0, 0.0, 0.0), 'nz': None} for t in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel')}
        self.sample_next = time.monotonic() + 3.0
        self.last_active = 0.0       # last time navigation or manual drive was active

        self.create_subscription(TFMessage, '/tf', self.on_tf, qos('sensor', 100))
        self.create_subscription(TFMessage, '/tf_static', self.on_tf_static,
                                 QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.log_buf = []
        self.rosout_filtered = self._subscribe_rosout()
        self.create_subscription(String, '/debug_log', self.on_ai_debug, qos('reliable', 20))
        self.bt_status = {}
        self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self.on_bt, qos('reliable', 20))
        self.demo_state = None
        self.demo_selected = -1
        # demo_manager QoS differs between builds (source: transient_local, the
        # installed binary on 2026-11-03: volatile) -> match the publisher, see tick_slow
        self.demo_subs = {}
        self.diag = {}
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, qos('reliable', 10))
        self.external_goal = None
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal_pose, qos('reliable', 5))

        # everybody's Nav2 goals (RViz, demo_manager, us)
        self.nav = {a: {'goals': {}, 'done': set()} for a in ('navigate_to_pose', 'follow_waypoints')}
        status_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for a in self.nav:
            self.create_subscription(GoalStatusArray, f'/{a}/_action/status',
                                     lambda m, a=a: self.on_nav_status(a, m), status_qos)
        self.goal_t0 = None          # start of the current navigation
        self.recoveries = 0
        self.wp_goals = 0            # navigate_to_pose goals seen during a waypoint mission
        self.plan_pts = None
        self.plan_seq = -1
        self.cancel_cli = {a: self.create_client(CancelGoal, f'/{a}/_action/cancel_goal') for a in self.nav}
        self.demo_stop_cli = self.create_client(Trigger, '/demo/stop')
        self.ac = {}                 # action clients, created on first use
        self.pending_goal = None     # (action, goal, label, reply id, deadline)
        self.mission = None
        self.oneshot = []            # [client, request, deadline, done-callback]

        # lifecycle: one short-lived client at a time (round robin)
        self.lc = {}
        self.lc_idx = 0
        self.lc_cur = None           # (name, client, deadline)

        self.blockers = {}
        self.blocker_re = [(re.compile(p), o, lb, re.compile(c) if c else None) for p, o, lb, c in BLOCKERS]

        self.teleop_pub = self.create_publisher(Twist, TELEOP_TOPIC, 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.teleop = None
        self.estop_until = 0.0

        self.cam_subs = {}
        self.cam_last = {}
        self.grid_mons = {}
        self.inspect = {}
        self.sent = {}

        self.create_timer(0.1, self.tick)
        self.event('CONSOLE', 'INFO', 'gateway started (python' +
                   (', rosout filtered' if self.rosout_filtered else '') + ')')

    # ------------------------------------------------------------------ util
    def ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def event(self, owner, level, text):
        emit('events', {'items': [{'t': time.time(), 'owner': owner, 'level': level, 'text': text}]})

    def lookup2d(self, target, source):
        try:
            return tf2d(self.tf_buffer.lookup_transform(target, source, Time()))
        except Exception:
            return None

    def _subscribe_mon(self, m, depth=10):
        if m.type.endswith('/Twist'):
            cb = (lambda raw, m=m: self.on_twist(m, raw))
        else:
            cb = (lambda raw, m=m: m.tick(raw))
        q = qos(m.kind, depth)
        m.sub = self.create_subscription(m.cls, m.topic, cb, q, raw=True)

    def _subscribe_rosout(self):
        # drop rf2o's per-scan INFO (~30 lines/s) inside DDS when supported
        if ContentFilterOptions is not None:
            try:
                sub = self.create_subscription(
                    Log, '/rosout', self.on_rosout, qos('reliable', 200),
                    content_filter_options=ContentFilterOptions(
                        filter_expression='name <> %0 OR level > %1',
                        expression_parameters=["'CLaserOdometry2D'", '20']))
                if getattr(sub, 'is_cft_enabled', False):
                    return True
                self.destroy_subscription(sub)
            except Exception as e:
                self.get_logger().info(f'rosout content filter unavailable: {e}')
        self.create_subscription(Log, '/rosout', self.on_rosout, qos('reliable', 200))
        return False

    # --------------------------------------------------------------- tf
    def _edge(self, parent, child, static):
        key = parent + '>' + child
        e = self.edges.get(key)
        if e is None:
            e = self.edges[key] = Edge(parent, child, static)
        return e

    def on_tf(self, msg):
        now = time.monotonic()
        rnow = self.ros_now()
        self.mons['/tf'].count += 1
        self.mons['/tf'].last = now
        for t in msg.transforms:
            parent = t.header.frame_id.lstrip('/')
            child = t.child_frame_id.lstrip('/')
            t.header.frame_id, t.child_frame_id = parent, child
            try:
                self.tf_buffer.set_transform(t, 'console')
            except Exception:
                pass
            e = self._edge(parent, child, False)
            pose = tf2d(t)
            if e.last is not None:
                gap = now - e.last
                if gap > 0.5:
                    e.gaps += 1
                    e.max_gap = max(e.max_gap, gap)
                    owner = 'SLAM' if child == 'odom' else ('EKF' if child == 'base_footprint' else 'TF')
                    self.event(owner, 'ERROR' if gap > 1.0 else 'WARN', f'TF {parent}→{child} gap {gap:.2f} s')
            if parent == 'map' and child == 'odom' and e.pose is not None:
                d = math.hypot(pose[0] - e.pose[0], pose[1] - e.pose[1])
                dy = abs(wrap(pose[2] - e.pose[2]))
                if d > 0.15 or dy > 0.10:
                    e.jumps += 1
                    self.event('SLAM', 'ERROR' if (d > 0.5 or dy > 0.3) else 'WARN',
                               f'SLAM correction: map→odom jumped {d:.2f} m / {math.degrees(dy):.1f}°')
            if parent == 'odom':
                self.odom_hist.append((now, pose[0], pose[1], pose[2]))
            e.pose = pose
            e.count += 1
            e.last = now
            e.lag = rnow - stamp_s(t.header.stamp)

    def on_tf_static(self, msg):
        now = time.monotonic()
        for t in msg.transforms:
            parent = t.header.frame_id.lstrip('/')
            child = t.child_frame_id.lstrip('/')
            t.header.frame_id, t.child_frame_id = parent, child
            try:
                self.tf_buffer.set_transform_static(t, 'console')
            except Exception:
                pass
            e = self._edge(parent, child, True)
            e.pose = tf2d(t)
            e.count += 1
            e.last = now

    # --------------------------------------------------------- callbacks
    def on_twist(self, m, raw):
        m.tick(raw)
        try:
            v = cdr_unpack('6d', raw)
        except struct.error:
            return
        c = self.cmd_vals[m.topic]
        c['v'] = (v[0], v[1], v[5])
        if abs(v[0]) > 1e-3 or abs(v[1]) > 1e-3 or abs(v[5]) > 1e-3:
            c['nz'] = m.last
            if m.topic == '/cmd_vel':
                self.last_nonzero_cmd = m.last

    def on_rosout(self, m):
        mon = self.mons['/rosout']
        mon.count += 1
        mon.last = time.monotonic()
        if m.name == 'robot_console':
            return
        now = mon.last
        for rx, owner, label, clear in self.blocker_re:
            if rx.search(m.msg):
                b = self.blockers.get(label) or {'n': 0}
                self.blockers[label] = {'owner': owner, 't': now, 'text': m.msg[:200], 'n': b['n'] + 1}
            elif clear is not None and label in self.blockers and clear.search(m.msg):
                self.blockers[label]['t'] = 0.0
        if len(self.log_buf) < 2000:
            self.log_buf.append({'t': stamp_s(m.stamp), 'lvl': LOG_LEVELS.get(m.level, 'I'),
                                 'node': m.name, 'msg': m.msg, 'src': 'rosout'})

    def on_ai_debug(self, m):
        mon = self.mons['/debug_log']
        mon.count += 1
        mon.last = time.monotonic()
        text = m.data
        lvl = 'E' if re.search(r'error|fail', text, re.I) else ('W' if re.search(r'warn', text, re.I) else 'I')
        if len(self.log_buf) < 2000:
            self.log_buf.append({'t': time.time(), 'lvl': lvl, 'node': 'ai_human_detection',
                                 'msg': text, 'src': 'ai_debug'})

    def on_bt(self, m):
        mon = self.mons['/behavior_tree_log']
        mon.count += 1
        mon.last = time.monotonic()
        for ev in m.event_log:
            prev = self.bt_status.get(ev.node_name)
            self.bt_status[ev.node_name] = ev.current_status
            if ev.current_status == 'FAILURE' and ev.node_name not in (
                    'WouldAControllerRecoveryHelp', 'WouldAPlannerRecoveryHelp', 'GoalUpdated'):
                self.event('NAV2', 'WARN', f'BT {ev.node_name}: {ev.previous_status} → FAILURE')
            elif ev.current_status == 'RUNNING' and prev != 'RUNNING' and ev.node_name in RECOVERY_NODES:
                self.recoveries += 1
                self.event('NAV2', 'INFO', f'BT recovery: {ev.node_name}')

    def on_demo_state(self, m):
        mon = self.mons['/demo/state']
        mon.count += 1
        mon.last = time.monotonic()
        if m.data != self.demo_state:
            lvl = 'WARN' if m.data in ('TARGET_LOST', 'NAVIGATION_FAILED') else 'INFO'
            self.event('DEMO', lvl, f'demo_manager state {self.demo_state or "?"} → {m.data}')
        self.demo_state = m.data

    def on_diag(self, m):
        now = time.monotonic()
        for s in m.status:
            lvl = s.level if isinstance(s.level, int) else ord(s.level)
            self.diag[s.name] = {'name': s.name, 'level': lvl, 'msg': s.message, 'hw': s.hardware_id, 't': now}

    def on_goal_pose(self, m):
        p = m.pose
        self.external_goal = (p.position.x, p.position.y, yaw_of(p.orientation), time.monotonic())

    def on_nav_status(self, a, m):
        st = self.nav[a]
        goals = {}
        for s in m.status_list:
            gid = bytes(s.goal_info.goal_id.uuid).hex()
            goals[gid] = s.status
            prev = st['goals'].get(gid)
            if s.status not in ACTIVE and gid not in st['done']:
                if prev is not None and prev in ACTIVE:
                    lvl = 'INFO' if s.status == 4 else 'WARN'
                    self.event('NAV2', lvl, f'{a} goal {gid[:6]} {STATUS_NAMES.get(s.status, s.status)}')
                st['done'].add(gid)
            elif s.status in ACTIVE and prev is None:
                self.event('NAV2', 'INFO', f'{a} goal {gid[:6]} accepted')
                self.last_active = time.monotonic()
                self._sampler(self.last_active)
                if a == 'navigate_to_pose':
                    if 'follow_waypoints' in self.nav_active():
                        self.wp_goals += 1
                    else:
                        self.goal_t0 = time.monotonic()
                        self.recoveries = 0
                else:
                    self.goal_t0 = time.monotonic()
                    self.recoveries = 0
                    self.wp_goals = 0
        st['goals'] = goals
        if len(st['done']) > 300:
            st['done'] = set()

    def nav_active(self):
        return [a for a, st in self.nav.items() if any(s in ACTIVE for s in st['goals'].values())]

    # ------------------------------------------------------------ commands
    def handle_cmd(self, c):
        cmd = c.get('cmd')
        rid = c.get('id')
        fn = getattr(self, 'cmd_' + str(cmd), None)
        if fn is None:
            res = (False, f'unknown command {cmd}')
        else:
            try:
                res = fn(c)
            except Exception as e:  # never let a bad command kill the gateway
                res = (False, f'{type(e).__name__}: {e}')
        if res is None:
            return               # reply comes later (deferred goal)
        ok, msg = res
        if rid is not None:
            self.reply(rid, ok, msg)

    def reply(self, rid, ok, msg):
        if rid is None:
            return
        emit('reply', {'id': rid, 'ok': ok, 'msg': msg if isinstance(msg, str) else '',
                       'data': None if isinstance(msg, str) else msg})

    def cmd_demand(self, c):
        self.set_demand(c.get('ch') or {})
        return True, 'ok'

    def cancel_all(self):
        # waypoint follower first, otherwise it sends the next waypoint
        n = 0
        for a in ('follow_waypoints', 'navigate_to_pose'):
            cli = self.cancel_cli[a]
            if cli.service_is_ready():
                cli.call_async(CancelGoal.Request())   # zero id + zero stamp = cancel all
                n += 1
        self.mission = None
        self.pending_goal = None
        return n

    def demo_stop(self):
        if self.demo_stop_cli.service_is_ready():
            self.demo_stop_cli.call_async(Trigger.Request())
            return True
        return False

    def cmd_teleop(self, c):
        now = time.monotonic()
        if now < self.estop_until:
            return False, 'e-stop active'
        vx = max(-TELEOP_MAX_LIN, min(TELEOP_MAX_LIN, float(c.get('vx', 0.0))))
        vy = max(-TELEOP_MAX_LIN, min(TELEOP_MAX_LIN, float(c.get('vy', 0.0))))
        wz = max(-TELEOP_MAX_ANG, min(TELEOP_MAX_ANG, float(c.get('wz', 0.0))))
        if self.teleop is None:
            took = []
            if self.nav_active():
                self.cancel_all()
                took.append('cancelled Nav2 goal')
            if self.demo_state in ('FOLLOWING', 'RETURN_HOME', 'DIRECT_NAVIGATION') and self.demo_stop():
                took.append('demo_manager stopped')
            self.event('CONSOLE', 'WARN', 'Manual override from ' + str(c.get('src', '?')) +
                       (' (' + '; '.join(took) + ')' if took else ''))
            self.teleop = {'v': (vx, vy, wz), 'last': now, 'src': c.get('src', '')}
            self.last_active = now
            self._sampler(now)
        self.teleop = {'v': (vx, vy, wz), 'last': now, 'src': c.get('src', '')}
        return True, 'ok'

    def cmd_estop(self, c):
        self.teleop = None
        self.estop_until = time.monotonic() + ESTOP_ZERO_S
        n = self.cancel_all()
        d = self.demo_stop()
        self.teleop_pub.publish(Twist())
        self.event('CONSOLE', 'ERROR', f'E-STOP from {c.get("src", "?")} '
                                       f'({n} action servers cancelled, demo stop {"sent" if d else "n/a"})')
        return True, 'stopped'

    def _pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.orientation.z = math.sin(float(yaw) / 2.0)
        p.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return p

    def _queue_goal(self, action, goal, label, rid, mission):
        # action clients are created on first use; discovery takes a moment,
        # so the goal is sent from tick() once the server is ready
        if action not in self.ac:
            self.ac[action] = ActionClient(self, NavigateToPose if action == 'navigate_to_pose' else FollowWaypoints,
                                           action)
        self.pending_goal = (action, goal, label, rid, time.monotonic() + 4.0, mission)

    def _send_pending(self):
        action, goal, label, rid, deadline, mission = self.pending_goal
        cli = self.ac[action]
        if not cli.server_is_ready():
            if time.monotonic() > deadline:
                self.pending_goal = None
                self.reply(rid, False, f'{action} action server not available')
            return
        self.pending_goal = None
        fut = cli.send_goal_async(goal)

        def on_resp(f):
            try:
                h = f.result()
            except Exception as e:
                self.event('CONSOLE', 'WARN', f'{label}: send failed: {e}')
                return
            if not h.accepted:
                self.event('CONSOLE', 'WARN', f'{label}: rejected by {action}')
                return

            def on_result(rf):
                try:
                    r = rf.result()
                except Exception:
                    return
                res = r.result
                extra = ''
                code = getattr(res, 'error_code', 0)
                if code:
                    extra = f' (error_code {code} {getattr(res, "error_msg", "")})'
                missed = getattr(res, 'missed_waypoints', None)
                if missed:
                    extra += f' missed {len(missed)} waypoint(s)'
                name = STATUS_NAMES.get(r.status, str(r.status))
                self.event('CONSOLE', 'INFO' if r.status == 4 else 'WARN', f'{label}: {name}{extra}')
                if self.mission and self.mission.get('label') == label:
                    self.mission['status'] = name
            h.get_result_async().add_done_callback(on_result)
        fut.add_done_callback(on_resp)
        self.mission = mission
        self.event('CONSOLE', 'INFO', f'{label} sent')
        self.reply(rid, True, 'sent')

    def cmd_goal(self, c):
        x, y, yaw = float(c['x']), float(c['y']), float(c.get('yaw', 0.0))
        if 'follow_waypoints' in self.nav_active() and self.cancel_cli['follow_waypoints'].service_is_ready():
            self.cancel_cli['follow_waypoints'].call_async(CancelGoal.Request())
        g = NavigateToPose.Goal()
        g.pose = self._pose(x, y, yaw)
        label = f'goal ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)'
        self._queue_goal('navigate_to_pose', g, label, c.get('id'),
                         {'kind': 'goal', 'label': label, 'poses': [[x, y, yaw]], 'status': 'SENT'})
        return None

    def cmd_waypoints(self, c):
        poses = c.get('poses') or []
        if not poses:
            return False, 'no waypoints'
        g = FollowWaypoints.Goal()
        g.poses = [self._pose(p[0], p[1], p[2] if len(p) > 2 else 0.0) for p in poses]
        label = f'{len(poses)} waypoints'
        mp = [[float(p[0]), float(p[1]), float(p[2] if len(p) > 2 else 0.0)] for p in poses]
        self._queue_goal('follow_waypoints', g, label, c.get('id'),
                         {'kind': 'waypoints', 'label': label, 'poses': mp, 'status': 'SENT'})
        return None

    def cmd_cancel(self, c):
        n = self.cancel_all()
        self.event('CONSOLE', 'INFO', f'cancel all goals from {c.get("src", "?")}')
        return (n > 0), f'cancel sent to {n} action server(s)'

    def cmd_initialpose(self, c):
        m = PoseWithCovarianceStamped()
        p = self._pose(c['x'], c['y'], c.get('yaw', 0.0))
        m.header = p.header
        m.pose.pose = p.pose
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25
        cov[35] = 0.0685
        m.pose.covariance = cov
        self.init_pub.publish(m)
        self.event('CONSOLE', 'INFO', f'initial pose published from {c.get("src", "?")}')
        return True, 'published on /initialpose (used by AMCL; ignored by slam_toolbox mapping)'

    def cmd_clear_costmaps(self, c):
        for srv in ('/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap'):
            self.oneshot.append([self.create_client(ClearEntireCostmap, srv), ClearEntireCostmap.Request(),
                                 time.monotonic() + 3.0, None])
        self.event('CONSOLE', 'INFO', 'clear costmaps')
        return True, 'clear requested for global + local costmap'

    def cmd_node_info(self, c):
        full = c.get('node', '')
        ns, _, name = full.rpartition('/')
        ns = ns or '/'

        def fmt(lst):
            return [[n, ', '.join(t)] for n, t in sorted(lst)]
        return True, {
            'node': full,
            'pubs': fmt(self.get_publisher_names_and_types_by_node(name, ns)),
            'subs': fmt(self.get_subscriber_names_and_types_by_node(name, ns)),
            'srvs': fmt(self.get_service_names_and_types_by_node(name, ns)),
        }

    # ------------------------------------------------------------- demand
    def set_demand(self, d):
        old = self.demand
        self.demand = d
        for ch in d:
            if ch not in old:
                self.sent.pop(ch, None)      # newly watched: send a snapshot
        if d.get('scan', 0) != old.get('scan', 0):
            self._sampler(time.monotonic())
        for src, topic in CAMERAS.items():
            want = d.get('cam:' + src, 0) > 0
            if want and src not in self.cam_subs:
                self.cam_subs[src] = self.create_subscription(
                    CompressedImage, topic, lambda m, s=src: self.on_cam(s, m), qos('sensor', 1))
            elif not want and src in self.cam_subs:
                self.destroy_subscription(self.cam_subs.pop(src))
        for ch, topic in GRIDS.items():
            want = d.get(ch, 0) > 0
            if want and ch not in self.grid_mons:
                m = Mon(topic)
                m.type, m.cls = 'nav_msgs/msg/OccupancyGrid', get_message('nav_msgs/msg/OccupancyGrid')
                self._subscribe_mon(m, depth=1)
                self.grid_mons[ch] = m
            elif not want and ch in self.grid_mons:
                self.destroy_subscription(self.grid_mons.pop(ch).sub)
        wanted = {k[len('inspect:'):] for k in d if k.startswith('inspect:')}
        for topic in list(self.inspect):
            if topic not in wanted:
                m = self.inspect.pop(topic)
                if m.sub is not None:
                    self.destroy_subscription(m.sub)
        for topic in wanted:
            if topic not in self.inspect:
                self.inspect[topic] = Mon(topic)

    def wants(self, ch):
        return self.demand.get(ch, 0) > 0

    def _inspect_subscribe(self, m):
        types = dict(self.get_topic_names_and_types()).get(m.topic)
        if not types:
            return
        m.type = types[0]
        try:
            m.cls = get_message(m.type)
        except Exception as e:
            m.type = f'{m.type} (cannot load: {e})'
            m.sub = False
            return
        infos = self.get_publishers_info_by_topic(m.topic)
        latched = any(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
        q = qos('latched') if latched else qos('sensor', 5)
        if latched:
            q.depth = 5
        m.sub = self.create_subscription(m.cls, m.topic, lambda raw, m=m: m.tick(raw), q, raw=True)

    def on_cam(self, src, msg):
        now = time.monotonic()
        fps = max(1.0, min(15.0, float(self.demand.get('cam:' + src, 5))))
        if now - self.cam_last.get(src, 0.0) < 1.0 / fps - 0.01:
            return
        self.cam_last[src] = now
        emit('cam:' + src, {'fmt': msg.format}, bytes(msg.data))

    # -------------------------------------------------------------- ticks
    def tick(self):
        """Single 10 Hz timer: commands, teleop, and every periodic output."""
        for _ in range(50):
            try:
                c = self.cmds.get_nowait()
            except queue.Empty:
                break
            self.handle_cmd(c)
        now = time.monotonic()
        if now < self.estop_until:
            self.teleop_pub.publish(Twist())
        elif self.teleop is not None:
            if now - self.teleop['last'] > TELEOP_DEADMAN_S:
                self.teleop = None
                for _ in range(3):
                    self.teleop_pub.publish(Twist())
                self.event('CONSOLE', 'INFO', 'Manual override released')
            else:
                t = Twist()
                t.linear.x, t.linear.y, t.angular.z = self.teleop['v']
                self.teleop_pub.publish(t)
        if self.pending_goal is not None:
            self._send_pending()
        self._service_oneshots(now)

        self.n += 1
        n = self.n
        self.emit_pose(now)                                   # 10 Hz
        if n % 2 == 0:                                        # 5 Hz
            self.emit_motion(now)
            if self.wants('scan'):
                self.emit_scan()
            if self.wants('people'):
                self.emit_people(now)
            if self.wants('cam:color') or self.wants('detections'):
                self.emit_detections(now)
        if n % 4 == 0 and self.log_buf:                       # 2.5 Hz
            items, self.log_buf = self.log_buf, []
            emit('log', {'items': items})
        if n % 5 == 0:                                        # 2 Hz
            for m in self.inspect.values():
                if m.sub is None:
                    self._inspect_subscribe(m)
                m.sample(now)
                self.emit_inspect(m, now)
        if n % 10 == 0:                                       # 1 Hz
            self.tick_1hz(now)
        if n % 50 == 1:                                       # 0.2 Hz
            self.tick_slow()

    def _service_oneshots(self, now):
        keep = []
        for item in self.oneshot:
            cli, req, deadline, cb = item
            if cli.service_is_ready():
                fut = cli.call_async(req)
                fut.add_done_callback(lambda f, cli=cli, cb=cb: self._oneshot_done(cli, cb, f))
            elif now > deadline:
                self.destroy_client(cli)
                if cb:
                    cb(None)
            else:
                keep.append(item)
        self.oneshot = keep

    def _oneshot_done(self, cli, cb, fut):
        try:
            res = fut.result()
        except Exception:
            res = None
        if cb:
            cb(res)
        self.destroy_client(cli)

    def _want_continuous(self, m, now):
        if m.mode != 'adaptive':
            return False
        if m.topic == '/scan':
            return self.wants('scan')
        if self.nav_active() or self.teleop is not None or now < self.estop_until:
            self.last_active = now
        return now - self.last_active < ACTIVE_HOLD_S

    def _sampler(self, now):
        for m in self.mons.values():
            if m.mode == 'always' or not m.type_ok:
                continue
            want = self._want_continuous(m, now)
            if want and not m.continuous:
                m.win = None
                if m.sub is None:
                    self._subscribe_mon(m)
                m.continuous = True
                m.hist.clear()
            elif not want and m.continuous:
                m.continuous = False
                if m.sub is not None:
                    self.destroy_subscription(m.sub)
                    m.sub = None
        sampled = [m for m in self.mons.values() if m.mode != 'always' and m.type_ok and not m.continuous]
        if any(m.win for m in sampled):
            for m in sampled:
                if m.win and now - m.win[0] >= SAMPLE_WINDOW_S:
                    dt = now - m.win[0]
                    m.hz = (m.count - m.win[1]) / dt
                    m.bw = (m.nbytes - m.win[2]) / dt
                    m.win = None
                    if m.sub is not None:
                        self.destroy_subscription(m.sub)
                        m.sub = None
        elif now >= self.sample_next:
            self.sample_next = now + SAMPLE_EVERY_S
            for m in sampled:
                if m.pubs == 0:
                    m.hz = 0.0
                    continue
                self._subscribe_mon(m, depth=300)
                m.win = (now, m.count, m.nbytes)

    def tick_1hz(self, now):
        self._sampler(now)
        for m in self.mons.values():
            m.sample(now)
        for e in self.edges.values():
            e.hist.append((now, e.count))
            t0, c0 = e.hist[0]
            if now - t0 > 0.5:
                e.hz = (e.count - c0) / (now - t0)
        emit('rates', {t: {'hz': fnum(m.hz, 1), 'age': fnum(m.age(now), 2), 'bw': int(m.bw), 'n': m.count,
                           'pubs': m.pubs, **({'sampled': True} if m.sampled else {})}
                       for t, m in self.mons.items()})
        self.emit_tf(now)
        self.evaluate_health(now)
        if self.wants('map'):
            self.emit_grid('map', self.mons['/map'])
        if 'costmap_local' in self.grid_mons:
            self.emit_grid('costmap_local', self.grid_mons['costmap_local'])
        if 'costmap_global' in self.grid_mons and self.n % 20 == 0:
            self.emit_grid('costmap_global', self.grid_mons['costmap_global'])
        if self.wants('plan'):
            self.emit_plan()
        if self.diag:
            emit('diag', [dict(v, age=fnum(now - v['t'], 1)) for v in self.diag.values()])
        self._lifecycle_step(now)

    def _lifecycle_step(self, now):
        # one short-lived GetState client at a time
        if self.lc_cur is not None:
            name, cli, deadline, called = self.lc_cur
            if called:
                if now > deadline + 3.0:          # no answer: give up on this one
                    self.lc[name] = 'error'
                    self.destroy_client(cli)
                    self.lc_cur = None
                return
            if cli.service_is_ready():
                fut = cli.call_async(GetState.Request())
                self.lc_cur = (name, cli, deadline, True)
                fut.add_done_callback(lambda f, name=name, cli=cli: self._lc_done(name, cli, f))
            elif now > deadline:
                self.lc[name] = 'unavailable'
                self.destroy_client(cli)
                self.lc_cur = None
            return
        name = LIFECYCLE[self.lc_idx % len(LIFECYCLE)][0]
        self.lc_idx += 1
        self.lc_cur = (name, self.create_client(GetState, f'/{name}/get_state'), now + 2.5, False)

    def _lc_done(self, name, cli, fut):
        try:
            self.lc[name] = fut.result().current_state.label
        except Exception:
            self.lc[name] = 'error'
        self.destroy_client(cli)
        self.lc_cur = None
        emit('lifecycle', self.lc)

    def tick_slow(self):
        try:
            topics = self.get_topic_names_and_types()
            nodes = sorted(n if ns == '/' else f'{ns}/{n}' for n, ns in self.get_node_names_and_namespaces())
        except Exception:
            return
        present = set()
        tl = []
        for name, types in topics:
            present.add(name)
            try:
                p = self.count_publishers(name)
                s = self.count_subscribers(name)
            except Exception:
                p = s = 0
            tl.append({'name': name, 'type': ', '.join(types), 'pubs': p, 'subs': s})
            if name in self.mons:
                self.mons[name].pubs = p
        for t, m in self.mons.items():
            if t not in present:
                m.pubs = 0
        emit('graph', {'nodes': nodes, 'topics': tl})
        emit('lifecycle', self.lc)
        self._match_demo_qos()

    def _match_demo_qos(self):
        """Subscribe /demo/state + /demo/selected_tracking_id with the publisher's durability
        (a transient_local subscriber gets nothing from a volatile publisher)."""
        for topic, cls, cb in (('/demo/state', String, self.on_demo_state),
                               ('/demo/selected_tracking_id', Int32,
                                lambda m: setattr(self, 'demo_selected', m.data))):
            infos = self.get_publishers_info_by_topic(topic)
            if not infos:
                continue
            latched = all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
            cur = self.demo_subs.get(topic)
            if cur is not None and cur[1] == latched:
                continue
            if cur is not None:
                self.destroy_subscription(cur[0])
            self.demo_subs[topic] = (self.create_subscription(cls, topic, cb, qos('latched' if latched else 'reliable')),
                                     latched)

    # ------------------------------------------------------------ emitters
    def odom_velocity(self, now):
        """Robot-frame velocity from the EKF's odom->base_footprint TF (no /odometry/filtered
        subscription needed).  Returns ([vx, vy, wz], age of the newest sample) or (None, None)."""
        h = self.odom_hist
        if len(h) < 2:
            return None, None
        last = h[-1]
        ref = None
        for s in reversed(h):
            if last[0] - s[0] >= 0.3:
                ref = s
                break
        if ref is None:
            ref = h[0]
        dt = last[0] - ref[0]
        if dt <= 0.05:
            return None, now - last[0]
        dx, dy = last[1] - ref[1], last[2] - ref[2]
        c, s = math.cos(last[3]), math.sin(last[3])
        v = [(dx * c + dy * s) / dt, (-dx * s + dy * c) / dt, wrap(last[3] - ref[3]) / dt]
        return [fnum(x) for x in v], now - last[0]

    def emit_pose(self, now):
        pose = self.lookup2d('map', 'base_link')
        vel, age = self.odom_velocity(now)
        emit('pose', {'ok': pose is not None,
                      'p': None if pose is None else [fnum(pose[0]), fnum(pose[1]), fnum(pose[2])],
                      'v': vel, 'age': fnum(age, 2)})

    def _plan(self):
        m = self.mons['/plan']
        if m.raw is None:
            return None
        if self.plan_seq != m.seq:
            try:
                p = m.msg()
                self.plan_pts = [(q.pose.position.x, q.pose.position.y) for q in p.poses]
                self.plan_frame = p.header.frame_id
            except Exception:
                self.plan_pts = None
            self.plan_seq = m.seq
        return self.plan_pts

    def emit_motion(self, now):
        def stage(topic):
            c = self.cmd_vals[topic]
            m = self.mons[topic]
            return {'v': [fnum(x) for x in c['v']], 'age': fnum(m.age(now), 2), 'hz': fnum(m.hz, 1),
                    'nz': fnum(now - c['nz'], 1) if c['nz'] else None, 'pubs': m.pubs}

        active = self.nav_active()
        nav = {'active': active, 'goals': {}, 'mission': self.mission}
        for a, st in self.nav.items():
            if st['goals']:
                last = list(st['goals'].values())[-1]
                nav['goals'][a] = STATUS_NAMES.get(last, str(last))
        fb = None
        if active:
            fb = {'action': active[-1] if 'follow_waypoints' not in active else 'follow_waypoints',
                  'rec': self.recoveries}
            if self.goal_t0 is not None:
                fb['elapsed'] = fnum(now - self.goal_t0, 1)
            pts = self._plan()
            pose = self.lookup2d('map', 'base_link')
            if pts and pose:
                # remaining length of the global plan from the closest point
                arr = np.asarray(pts)
                i = int(np.argmin(np.hypot(arr[:, 0] - pose[0], arr[:, 1] - pose[1])))
                seg = np.hypot(np.diff(arr[i:, 0]), np.diff(arr[i:, 1])).sum() if len(arr) - i > 1 else 0.0
                fb['dist'] = fnum(math.hypot(arr[i, 0] - pose[0], arr[i, 1] - pose[1]) + seg, 2)
            if 'follow_waypoints' in active:
                fb['wp'] = max(0, self.wp_goals - 1)
        nav['fb'] = fb
        eg = self.external_goal
        nav['ext_goal'] = None if eg is None or now - eg[3] > 600 else [fnum(eg[0]), fnum(eg[1]), fnum(eg[2])]

        col = self.mons['/collision_monitor_state']
        col_d = None
        if col.raw is not None:
            try:
                cm = col.msg()
                col_d = {'action': COLLISION_ACTIONS.get(cm.action_type, str(cm.action_type)),
                         'poly': cm.polygon_name, 'age': fnum(col.age(now), 1)}
            except Exception:
                pass
        son = self.mons['/ultrasonic']
        son_d = None
        if son.raw is not None:
            try:
                son_d = {'d': fnum(cdr_unpack('f', son.raw)[0] * 0.01, 3), 'age': fnum(son.age(now), 2)}
            except struct.error:
                pass
        vel, age = self.odom_velocity(now)
        od = None if vel is None else {'v': vel, 'age': fnum(age, 2)}
        running = [n for n, s in self.bt_status.items() if s == 'RUNNING'] if active else []
        blockers = [{'label': k, 'owner': v['owner'], 'age': fnum(now - v['t'], 1), 'text': v['text'], 'n': v['n']}
                    for k, v in self.blockers.items() if now - v['t'] < 5.0]
        emit('motion', {
            'demo': {'state': self.demo_state, 'sel': self.demo_selected, 'pubs': self.mons['/demo/state'].pubs},
            'nav': nav, 'bt': running[-6:],
            'nav_cmd': stage('/cmd_vel_nav'), 'smoothed': stage('/cmd_vel_smoothed'), 'out': stage('/cmd_vel'),
            'collision': col_d, 'sonar': son_d, 'odom': od, 'blockers': blockers,
            'teleop': None if self.teleop is None else {'v': [fnum(x) for x in self.teleop['v']],
                                                        'src': self.teleop['src']},
            'estop': now < self.estop_until,
        })

    def emit_scan(self):
        m = self.mons['/scan']
        if m.raw is None or self.sent.get('scan') == m.seq:
            return
        try:
            s = m.msg()
        except Exception:
            return
        pose = self.lookup2d('map', s.header.frame_id.lstrip('/'))
        if pose is None:
            return
        self.sent['scan'] = m.seq
        r = np.asarray(s.ranges, dtype=np.float32)
        n = r.size
        step = max(1, n // 720)
        idx = np.arange(0, n, step)
        r = r[idx]
        a = s.angle_min + idx * s.angle_increment + pose[2]
        ok = np.isfinite(r) & (r > s.range_min) & (r < s.range_max)
        r, a = r[ok], a[ok]
        xy = np.empty(r.size * 2, dtype=np.float32)
        xy[0::2] = pose[0] + r * np.cos(a)
        xy[1::2] = pose[1] + r * np.sin(a)
        emit('scan', {'n': int(r.size), 'beams': int(n), 'frame': s.header.frame_id,
                      'xy': base64.b64encode(xy.tobytes()).decode()})

    def emit_grid(self, ch, m):
        if m.raw is None or self.sent.get(ch) == m.seq:
            return
        try:
            g = m.msg()
        except Exception:
            return
        frame = g.header.frame_id.lstrip('/') or 'map'
        tf = (0.0, 0.0, 0.0) if frame == 'map' else self.lookup2d('map', frame)
        if tf is None:
            return
        self.sent[ch] = m.seq
        o = g.info.origin
        origin = compose(tf, (o.position.x, o.position.y, yaw_of(o.orientation)))
        data = np.asarray(g.data, dtype=np.int8).astype(np.uint8)   # -1 -> 255
        emit(ch, {'w': g.info.width, 'h': g.info.height, 'res': fnum(g.info.resolution, 4),
                  'o': [fnum(origin[0], 4), fnum(origin[1], 4), fnum(origin[2], 4)], 'frame': frame,
                  'data': base64.b64encode(data.tobytes()).decode()})

    def emit_plan(self):
        m = self.mons['/plan']
        if m.raw is None or self.sent.get('plan') == m.seq:
            return
        pts = self._plan()
        if pts is None:
            return
        self.sent['plan'] = m.seq
        step = max(1, len(pts) // 300)
        sel = pts[::step]
        if pts and sel[-1] != pts[-1]:
            sel.append(pts[-1])
        emit('plan', {'frame': getattr(self, 'plan_frame', 'map'), 'n': len(pts),
                      'pts': [[fnum(x), fnum(y)] for x, y in sel], 'age': fnum(m.age(time.monotonic()), 1)})

    def emit_people(self, now):
        out = {'ai': [], 'demo': [], 'sel': self.demo_selected, 'state': self.demo_state}
        m = self.mons['/person_infos']
        if m.raw is not None and m.cls is not None:
            try:
                msg = m.msg()
                frame = msg.header.frame_id.lstrip('/')
                t = None
                try:
                    t = self.tf_buffer.lookup_transform('map', frame, Time())
                except Exception:
                    pass
                for p in msg.persons:
                    v = (p.position.x, p.position.y, p.position.z)
                    item = {'id': p.tracking_id, 'conf': fnum(p.confidence, 2), 'tracked': bool(p.is_tracked),
                            'cam': [fnum(c) for c in v], 'map': None,
                            'range': fnum(math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2), 2)}
                    if t is not None:
                        r = t.transform.rotation
                        w = qrot((r.x, r.y, r.z, r.w), v)
                        tr = t.transform.translation
                        item['map'] = [fnum(tr.x + w[0]), fnum(tr.y + w[1])]
                    out['ai'].append(item)
                out['ai_age'] = fnum(m.age(now), 2)
                out['ai_frame'] = frame
                out['ai_tf'] = t is not None
            except Exception as e:
                out['ai_error'] = str(e)
        mk = self.mons['/demo/person_markers']
        if mk.raw is not None:
            try:
                labels = {}
                cyl = []
                for k in mk.msg().markers:
                    if k.ns == 'ai_person_labels':
                        labels[k.id] = k.text
                    elif k.ns == 'ai_persons' and k.action == 0:
                        cyl.append(k)
                for k in cyl:
                    out['demo'].append({'id': k.id, 'x': fnum(k.pose.position.x), 'y': fnum(k.pose.position.y),
                                        'c': [fnum(k.color.r, 2), fnum(k.color.g, 2), fnum(k.color.b, 2)],
                                        'text': labels.get(k.id, ''), 'frame': k.header.frame_id})
            except Exception:
                pass
        emit('people', out)

    def emit_detections(self, now):
        m = self.mons['/human_detections']
        if m.raw is None or m.cls is None:
            return
        try:
            d = m.msg()
            w = h = None
            info = self.mons['/camera/camera/color/camera_info']
            if info.raw is not None:
                ci = info.msg()
                w, h = ci.width, ci.height
            boxes = []
            for det in d.detections:
                b = det.bbox
                score = cls = None
                if det.results:
                    score = fnum(det.results[0].hypothesis.score, 2)
                    cls = det.results[0].hypothesis.class_id
                boxes.append({'cx': fnum(b.center.position.x, 1), 'cy': fnum(b.center.position.y, 1),
                              'w': fnum(b.size_x, 1), 'h': fnum(b.size_y, 1), 'id': det.id,
                              'score': score, 'cls': cls})
            emit('detections', {'w': w or 640, 'h': h or 360, 'boxes': boxes, 'age': fnum(m.age(now), 2)})
        except Exception:
            pass

    def emit_inspect(self, m, now):
        d = {'topic': m.topic, 'type': m.type, 'hz': fnum(m.hz, 2), 'bw': int(m.bw), 'n': m.count,
             'age': fnum(m.age(now), 2), 'size': len(m.raw) if m.raw is not None else None,
             'subscribed': bool(m.sub)}
        if now - m.info_t > 5.0:
            m.info_t = now
            try:
                def ep(i):
                    return {'node': (i.node_namespace.rstrip('/') + '/' + i.node_name), **qos_json(i.qos_profile)}
                m.pubs_info = [ep(i) for i in self.get_publishers_info_by_topic(m.topic)]
                m.subs_info = [ep(i) for i in self.get_subscriptions_info_by_topic(m.topic)
                               if i.node_name != 'robot_console']
            except Exception:
                pass
        d['pubs'] = m.pubs_info
        d['subs'] = m.subs_info
        if m.raw is not None and m.cls is not None:
            if len(m.raw) > 4_000_000:
                d['msg'] = f'<{len(m.raw)} bytes, too large to decode>'
            else:
                try:
                    d['msg'] = clean(message_to_ordereddict(m.msg(), truncate_length=24))
                except Exception as e:
                    d['msg'] = f'<decode error: {e}>'
        emit('inspect:' + m.topic, d)

    def emit_tf(self, now):
        emit('tf', [{'p': e.parent, 'c': e.child, 's': e.static, 'hz': fnum(e.hz, 1),
                     'age': None if e.static or e.last is None else fnum(now - e.last, 2),
                     'lag': None if e.static or e.lag is None else fnum(e.lag, 3),
                     'gaps': e.gaps, 'max_gap': fnum(e.max_gap, 2), 'jumps': e.jumps,
                     'pose': None if e.pose is None else [fnum(x) for x in e.pose]}
                    for e in self.edges.values()])

    # -------------------------------------------------------------- health
    def evaluate_health(self, now):
        H = self.health
        H.begin()
        for m in self.mons.values():
            if m.warn is None and m.err is None and m.expect is None:
                continue
            cid = 'topic:' + m.topic
            name = f'{m.label} ({m.topic})'
            hz = f'{m.hz:.1f} Hz' + (f' / {m.expect} expected' if m.expect else '') + (' (sampled)' if m.sampled else '')
            if not m.type_ok:
                H.add(cid, m.owner, name, 'WARN', '', f'message type {m.type} not available to the console')
            elif m.pubs == 0:
                H.add(cid, m.owner, name, 'OFF' if m.optional else 'ERROR', 'no publisher', 'node not running',
                      immediate=True)
            elif m.sampled and m.count == 0:
                H.add(cid, m.owner, name, 'OK', 'measuring…')
            elif m.err is not None and m.hz < m.err:
                H.add(cid, m.owner, name, 'ERROR', hz, f'below {m.err} Hz')
            elif m.warn is not None and m.hz < m.warn:
                H.add(cid, m.owner, name, 'WARN', hz, f'below {m.warn} Hz')
            else:
                H.add(cid, m.owner, name, 'OK', hz)
        sm = self.mons['/scan']
        if sm.raw is not None:
            try:
                s = sm.msg()
                lag = self.ros_now() - stamp_s(s.header.stamp)
                lvl = 'OK' if lag < 0.5 else ('WARN' if lag < 1.0 else 'ERROR')
                H.add('scan:stamp', 'LIDAR', 'Scan timestamp lag', lvl, f'{lag:.2f} s',
                      f'{len(s.ranges)} beams, frame {s.header.frame_id}')
            except Exception:
                pass
        mp = self.mons['/map']
        if mp.pubs > 1:
            H.add('map:pubs', 'SLAM', 'Map publishers', 'WARN', f'{mp.pubs}', 'more than one /map publisher')
        if mp.last is not None:
            H.add('map:age', 'SLAM', 'Last map update', 'OK', f'{now - mp.last:.0f} s ago',
                  'event driven (map_update_interval 5 s, only after movement)')
        for parent, child, owner, label, warn, err, static in TF_CHECKS:
            e = self.edges.get(parent + '>' + child)
            cid = f'tf:{parent}>{child}'
            if e is None:
                H.add(cid, owner, label, 'OFF' if owner == 'AI' else 'ERROR', 'missing',
                      'transform never received', immediate=True)
                continue
            if static:
                lvl, detail = 'OK', ''
                if child == 'laser' and e.pose is not None and abs(abs(e.pose[2]) - math.pi) > 0.05:
                    lvl, detail = 'WARN', f'yaw {math.degrees(e.pose[2]):.0f}° (expected 180°)'
                H.add(cid, owner, label, lvl, 'static', detail)
                continue
            age = now - e.last
            lvl = 'OK' if age <= warn else ('WARN' if age <= err else 'ERROR')
            detail = f'{e.hz:.1f} Hz, gaps {e.gaps}' + (f' (max {e.max_gap:.1f} s)' if e.gaps else '')
            if child == 'odom':
                detail += f', jumps {e.jumps}'
            H.add(cid, owner, label, lvl, f'age {age:.2f} s', detail)
        if self.odom_hist and now - self.last_nonzero_cmd > 6.0 and self.teleop is None:
            ref = next((s for s in self.odom_hist if now - s[0] <= 5.0), None)
            last = self.odom_hist[-1]
            if ref is not None and last[0] - ref[0] > 4.0:
                d = math.hypot(last[1] - ref[1], last[2] - ref[2])
                dy = abs(wrap(last[3] - ref[3]))
                H.add('drift', 'EKF', 'Drift while idle (5 s)', 'WARN' if (d > 0.02 or dy > 0.02) else 'OK',
                      f'{d * 100:.1f} cm / {math.degrees(dy):.1f}°', 'robot not commanded; odom should stay still')
        for name, owner, required in LIFECYCLE:
            st = self.lc.get(name, '?')
            if st == 'active':
                H.add('lc:' + name, owner, name, 'OK', 'active')
            elif st == 'unavailable':
                if required:
                    H.add('lc:' + name, owner, name, 'ERROR', 'not running', '', immediate=True)
            elif st != '?':
                H.add('lc:' + name, owner, name, 'ERROR', st, 'lifecycle not active', immediate=True)
        names = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
        for v in self.diag.values():
            if now - v['t'] > 10:
                continue
            low = v['name'].lower()
            owner = 'EKF' if ('ekf' in low or 'filter' in low) else 'NAV2'
            # capped at WARN: the console's own checks are authoritative
            # (robot_localization says "No events recorded" at 20 Hz output)
            H.add('diag:' + v['name'], owner, v['name'], 'OK' if v['level'] == 0 else 'WARN', v['msg'][:60],
                  f'/diagnostics level {names.get(v["level"], v["level"])}', immediate=True)
        if self.mons['/demo/state'].pubs == 0:
            H.add('demo:state', 'DEMO', 'demo_manager state', 'OFF', 'not running', '', immediate=True)
        else:
            lvl = 'WARN' if self.demo_state in ('TARGET_LOST', 'NAVIGATION_FAILED') else 'OK'
            H.add('demo:state', 'DEMO', 'demo_manager state', lvl, self.demo_state or '?',
                  f'selected ID {self.demo_selected}', immediate=True)
        emit('health', H.items)


def stdin_reader(q):
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            q.put(json.loads(line))
        except ValueError:
            sys.stderr.write(f'bad command: {line[:200]}\n')
    os._exit(0)  # Go went away


def main():
    q = queue.Queue()
    threading.Thread(target=stdin_reader, args=(q,), daemon=True).start()
    rclpy.init()
    node = Gateway(q)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.teleop_pub.publish(Twist())
        except Exception:
            pass


if __name__ == '__main__':
    main()
