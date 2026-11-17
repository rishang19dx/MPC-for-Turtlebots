"""LaserScan -> inflated obstacle circles in the odom frame.

/scan (sensor-data QoS) --segment + circle fit (core.lidar)--> tf2 (odom <- scan frame,
at the scan's own timestamp) --> inflate by robot radius + margin --> K nearest
--> /obstacles (Float32MultiArray [x, y, r]*K) and /obstacles_markers (RViz).
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from tb3_mpc.core import lidar
from tb3_mpc.core.model import BURGER
from tb3_mpc.nodes.common import declare_dataclass_params, yaw_from_quat


class ObstacleExtractor(Node):
    def __init__(self):
        super().__init__("obstacle_extractor")
        self.declare_parameter("fixed_frame", "odom")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("inflation_margin", 0.05)
        self.declare_parameter("max_obstacles", 8)
        self.params = lidar.LidarParams(**declare_dataclass_params(self, "lidar", lidar.LidarParams))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(Float32MultiArray, "/obstacles", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/obstacles_markers", 10)

    def lookup(self, target, source, stamp):
        try:
            return self.tf_buffer.lookup_transform(target, source, stamp, timeout=Duration(seconds=0.15))
        except TransformException:
            try:  # fall back to the latest transform (e.g. clock skew); log it
                self.get_logger().warn(f"tf at scan stamp unavailable for {source}; using latest",
                                       throttle_duration_sec=2.0)
                return self.tf_buffer.lookup_transform(target, source, Time())
            except TransformException as e:
                self.get_logger().warn(str(e), throttle_duration_sec=2.0)
                return None

    def on_scan(self, msg: LaserScan):
        fixed = self.get_parameter("fixed_frame").value
        T = self.lookup(fixed, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if T is None:
            return
        circ = lidar.extract_circles(msg.ranges, msg.angle_min, msg.angle_increment,
                                     msg.range_min, msg.range_max, self.params)
        tx, ty = T.transform.translation.x, T.transform.translation.y
        yaw = yaw_from_quat(T.transform.rotation)
        c, s = np.cos(yaw), np.sin(yaw)
        if len(circ):
            circ = np.column_stack([tx + c * circ[:, 0] - s * circ[:, 1],
                                    ty + s * circ[:, 0] + c * circ[:, 1],
                                    circ[:, 2] + BURGER.robot_radius
                                    + self.get_parameter("inflation_margin").value])
        circ = lidar.nearest_k(circ, np.array([tx, ty]), self.get_parameter("max_obstacles").value)
        out = Float32MultiArray()
        out.data = [float(v) for v in circ.ravel()]
        self.pub.publish(out)
        self.publish_markers(circ, fixed, msg.header.stamp)

    def publish_markers(self, circ, frame, stamp):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        for i, (x, y, r) in enumerate(circ):
            m = Marker()
            m.header.frame_id, m.header.stamp = frame, stamp
            m.ns, m.id, m.type, m.action = "obstacles", i, Marker.CYLINDER, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = float(x), float(y), 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = float(2 * r)
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.35, 0.1, 0.45
            arr.markers.append(m)
        self.marker_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
