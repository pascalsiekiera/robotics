#!/usr/bin/env python3
"""Task 2a: helper node for replaying a lab rosbag in RViz (used by record_lab_replay.sh).

- /tf_static: the static transforms of the robot are in the bag only once, at its start, so they
  are lost with ros2 bag play --start-offset. They are read from the bag here and published
  transient local (play the bag without /tf_static).
- /tf: the bag is played with /tf remapped to /bag_tf; its transforms are forwarded to /tf except
  map -> odom, which is computed from /amcl_pose and /odom instead (AMCL pose composed with the
  inverse odometry pose at the stamp of the AMCL pose) and sent at 20 Hz, stamped 1 s ahead like
  AMCL does. Reason: the recorder lost many /tf messages in the lab (gaps of up to 17 s in
  map -> odom while /amcl_pose was complete), the robot model then stood up to 2 m away from the
  AMCL pose.
- mp2/driven_estimated: the AMCL poses since the start as nav_msgs/Path (green in mp2.rviz),
  transient local, like waypoint_nav.py does on the robot.
- /waypoints: the waypoints as markers (MarkerArray display of mp2.rviz), sent once a second.
- --clock-file: for every /clock message of the bag player the offset wall time - bag time is
  measured; "<median offset> <last bag time>" is written to this file twice a second, so the
  recording script can convert bag times (waypoint log) to wall times (video) and knows when the
  end of the run has been played.

Example (ROS_DOMAIN_ID of the replay, not of the robot!):
    python3 replay_helpers.py --bag ~/mp2_bags/dijkstra_dwb_test --clock-file /tmp/clock.txt
"""
import argparse
import bisect
import math
import os
import statistics
import time

import rclpy
import rclpy.executors
import rosbag2_py
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray

MP2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp_of(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def read_tf_static(bag):
    """All transforms of /tf_static in the bag (last one per child frame)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf_static']))
    transforms = {}
    while reader.has_next():
        _, data, _ = reader.read_next()
        for tf in deserialize_message(data, TFMessage).transforms:
            transforms[tf.child_frame_id] = tf
    return list(transforms.values())


class ReplayHelpers(Node):
    def __init__(self, args):
        super().__init__('mp2_replay_helpers')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_static = self.create_publisher(TFMessage, '/tf_static', latched)
        transforms = read_tf_static(os.path.expanduser(args.bag))
        self.pub_static.publish(TFMessage(transforms=transforms))
        self.get_logger().info(f'/tf_static: {len(transforms)} transforms from the bag')

        self.pub_path = self.create_publisher(Path, 'mp2/driven_estimated', latched)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.pub_path.publish(self.path)  # clears an old path in RViz
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, 10)

        # /tf of the bag without map -> odom, map -> odom from /amcl_pose + /odom
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 100)
        self.create_subscription(TFMessage, '/bag_tf', self.on_bag_tf, 100)
        self.odom = []          # (stamp, x, y, yaw) of /odom, robot clock
        self.map_odom = None    # (x, y, yaw)
        self.create_subscription(Odometry, 'odom', self.on_odom, 100)
        self.create_timer(0.05, self.send_map_odom)

        self.pub_wp = self.create_publisher(MarkerArray, 'waypoints', 5)
        with open(args.waypoints) as f:
            self.markers = self.waypoint_markers(yaml.safe_load(f)['waypoints'])
        self.create_timer(1.0, lambda: self.pub_wp.publish(self.markers))

        self.clock_file = args.clock_file
        self.offsets = []
        self.last_sim = None
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Clock, '/clock', self.on_clock, best_effort)
        if self.clock_file:
            self.create_timer(0.5, self.write_clock_file)

    @staticmethod
    def waypoint_markers(waypoints):
        out = MarkerArray()
        for i, wp in enumerate(waypoints):
            for kind in ('sphere', 'text'):
                m = Marker()
                m.header.frame_id = 'map'  # stamp 0: latest transform
                m.ns = f'waypoints_{kind}'
                m.id = i
                m.action = Marker.ADD
                m.pose.position.x, m.pose.position.y = float(wp['x']), float(wp['y'])
                m.pose.orientation.w = 1.0
                if kind == 'sphere':
                    m.type = Marker.CYLINDER
                    m.scale.x = m.scale.y = 0.2
                    m.scale.z = 0.02
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 0.8
                else:
                    m.type = Marker.TEXT_VIEW_FACING
                    m.text = str(wp['name'])
                    m.pose.position.x += 0.18
                    m.pose.position.y += 0.18
                    m.pose.position.z = 0.3
                    m.scale.z = 0.3
                    m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
                out.markers.append(m)
        return out

    def on_bag_tf(self, msg):
        transforms = [tf for tf in msg.transforms
                      if not (tf.header.frame_id == 'map' and tf.child_frame_id == 'odom')]
        if transforms:
            self.pub_tf.publish(TFMessage(transforms=transforms))

    def on_odom(self, msg):
        p = msg.pose.pose
        self.odom.append((stamp_of(msg.header), p.position.x, p.position.y, yaw_of(p.orientation)))
        self.odom = self.odom[-2000:]

    def odom_at(self, t):
        """Odometry pose (x, y, yaw) at robot time t, linear between the neighbouring messages."""
        times = [o[0] for o in self.odom]
        k = bisect.bisect_left(times, t)
        if k == 0 or k == len(times):
            return self.odom[min(k, len(times) - 1)][1:]
        (t0, x0, y0, a0), (t1, x1, y1, a1) = self.odom[k - 1], self.odom[k]
        w = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
        da = math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
        return x0 + w * (x1 - x0), y0 + w * (y1 - y0), a0 + w * da

    def send_map_odom(self):
        if self.map_odom is None:
            return
        x, y, yaw = self.map_odom
        tf = TransformStamped()
        now = self.get_clock().now().nanoseconds * 1e-9 + 1.0   # 1 s ahead, like AMCL
        tf.header.stamp.sec = int(now)
        tf.header.stamp.nanosec = int((now - int(now)) * 1e9)
        tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.transform.translation.x, tf.transform.translation.y = x, y
        tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.pub_tf.publish(TFMessage(transforms=[tf]))

    def on_amcl(self, msg):
        if self.odom:
            ox, oy, oyaw = self.odom_at(stamp_of(msg.header))
            p = msg.pose.pose
            yaw = yaw_of(p.orientation) - oyaw
            c, s = math.cos(yaw), math.sin(yaw)
            self.map_odom = (p.position.x - (c * ox - s * oy), p.position.y - (s * ox + c * oy), yaw)
        self.path.poses.append(PoseStamped(header=msg.header, pose=msg.pose.pose))
        self.path.header.stamp = msg.header.stamp
        self.pub_path.publish(self.path)

    def on_clock(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.offsets.append(time.time() - sim)
        self.offsets = self.offsets[-2000:]
        self.last_sim = sim

    def write_clock_file(self):
        if self.last_sim is None:
            return
        tmp = self.clock_file + '.tmp'
        with open(tmp, 'w') as f:
            f.write(f'{statistics.median(self.offsets):.4f} {self.last_sim:.4f}\n')
        os.replace(tmp, self.clock_file)


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--bag', required=True, help='rosbag directory (for /tf_static)')
    parser.add_argument('--waypoints', default=os.path.join(MP2, 'config', 'waypoints_lsdc3.yaml'))
    parser.add_argument('--clock-file', default='', help='file for "<wall-bag offset> <bag time>"')
    args, _ = parser.parse_known_args()
    rclpy.init()
    node = ReplayHelpers(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
