"""On-robot safety relay (run on the TurtleBot's Raspberry Pi).

turtlebot3_node has no cmd_vel timeout, so a Wi-Fi drop would leave the last
command running. This relay forwards ``in_topic`` -> ``out_topic`` and, if no
command arrives within ``timeout`` seconds (or /estop is true), publishes zero
for ``zero_burst`` seconds and then goes quiet (so teleop still works).
Only a node on the robot itself can catch a lost network link.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self.declare_parameter("in_topic", "/cmd_vel_safe")
        self.declare_parameter("out_topic", "/cmd_vel")
        self.declare_parameter("in_stamped", False)
        self.declare_parameter("out_stamped", False)
        self.declare_parameter("timeout", 0.3)
        self.declare_parameter("zero_burst", 0.5)
        self.declare_parameter("rate_hz", 20.0)
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.out_stamped = g("out_stamped")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped if self.out_stamped else Twist, g("out_topic"), qos)
        self.create_subscription(TwistStamped if g("in_stamped") else Twist, g("in_topic"), self.on_cmd, qos)
        self.create_subscription(Bool, "/estop", self.on_estop, 10)
        self.last = None
        self.last_t = None
        self.stale_since = None
        self.estop = False
        self.create_timer(1.0 / g("rate_hz"), self.on_timer)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        tw = msg.twist if isinstance(msg, TwistStamped) else msg
        self.last = (tw.linear.x, tw.angular.z)
        self.last_t = self.now()

    def on_estop(self, msg: Bool):
        self.estop = bool(msg.data)

    def send(self, v, w):
        if self.out_stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_footprint"
            tw = m.twist
        else:
            m = tw = Twist()
        tw.linear.x, tw.angular.z = float(v), float(w)
        self.pub.publish(m)

    def on_timer(self):
        t = self.now()
        fresh = self.last_t is not None and (t - self.last_t) <= self.get_parameter("timeout").value
        if fresh and not self.estop:
            self.stale_since = None
            self.send(*self.last)
            return
        if self.last_t is None and not self.estop:
            return  # nothing received yet: stay quiet
        if self.stale_since is None:
            self.stale_since = t
            self.get_logger().warn("estop" if self.estop else "command timeout -> zero velocity")
        if self.estop or t - self.stale_since <= self.get_parameter("zero_burst").value:
            self.send(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send(0.0, 0.0)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
