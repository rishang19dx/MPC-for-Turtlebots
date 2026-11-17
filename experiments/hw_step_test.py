#!/usr/bin/env python3
"""Hardware identification: velocity step test -> dead time + first-order lag.

Run on the laptop with the robot side up (robot.launch.py relay running), the
robot on the floor with >= 1 m of free space ahead:

  source ~/ar521_ws/install/setup.bash
  python3 experiments/hw_step_test.py --v 0.15 --duration 3 --topic /cmd_vel_safe

It publishes v = 0 for 1 s, then a step to --v, then 0 again, while recording
/odom twist and stamps. It fits  v(t) = V (1 - exp(-(t - t0 - tau_d)/T))  and
prints the dead time tau_d (use it as ``delay_compensation``) and lag T. It also
reports /odom and /scan rates and the odom stamp age (needs chrony-synced clocks).
"""
from __future__ import annotations

import argparse
import csv
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.optimize import curve_fit
from sensor_msgs.msg import LaserScan


class StepTest(Node):
    def __init__(self, a):
        super().__init__("hw_step_test")
        self.a = a
        self.pub = self.create_publisher(TwistStamped if a.stamped else Twist, a.topic, 1)
        self.odom = []   # (t_recv, t_stamp, v, w)
        self.scan_t = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(LaserScan, "/scan", lambda m: self.scan_t.append(self.now()),
                                 qos_profile_sensor_data)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, m):
        st = m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec
        self.odom.append((self.now(), st, m.twist.twist.linear.x, m.twist.twist.angular.z))

    def send(self, v):
        if self.a.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.twist.linear.x = float(v)
        else:
            m = Twist()
            m.linear.x = float(v)
        self.pub.publish(m)


def first_order(t, V, tau_d, T):
    return np.where(t > tau_d, V * (1 - np.exp(-(t - tau_d) / max(T, 1e-3))), 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=0.15)
    ap.add_argument("--duration", type=float, default=3.0)
    ap.add_argument("--topic", default="/cmd_vel_safe")
    ap.add_argument("--stamped", action="store_true")
    ap.add_argument("--out", default="step_test.csv")
    a = ap.parse_args()
    assert 0 < a.v <= 0.22, "step must be within the Burger's limits"
    rclpy.init()
    node = StepTest(a)
    t_step = None
    t_start = time.time()
    try:
        while rclpy.ok():
            el = time.time() - t_start
            if el < 1.0:
                node.send(0.0)
            elif el < 1.0 + a.duration:
                if t_step is None:
                    t_step = node.now()
                node.send(a.v)
            elif el < 2.5 + a.duration:
                node.send(0.0)
            else:
                break
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        for _ in range(5):
            node.send(0.0)
            time.sleep(0.02)
    od = np.array(node.odom)
    with open(a.out, "w", newline="") as f:
        csv.writer(f).writerows([["t_recv", "t_stamp", "v", "w"], *od.tolist()])
    rate = 1 / np.median(np.diff(od[:, 0]))
    scan_rate = 1 / np.median(np.diff(node.scan_t)) if len(node.scan_t) > 2 else float("nan")
    age = od[:, 0] - od[:, 1]
    print(f"/odom rate {rate:.1f} Hz, /scan rate {scan_rate:.1f} Hz")
    print(f"odom stamp age: median {1e3 * np.median(age):.1f} ms, p95 {1e3 * np.percentile(age, 95):.1f} ms"
          " (meaningful only with chrony-synced clocks)")
    m = (od[:, 1] >= t_step - 0.5) & (od[:, 1] <= t_step + a.duration)
    t = od[m, 1] - t_step
    (V, tau_d, T), _ = curve_fit(first_order, t, od[m, 2], p0=[a.v, 0.1, 0.1],
                                 bounds=([0, 0, 0], [0.3, 1.0, 2.0]))
    print(f"step fit: V = {V:.3f} m/s (commanded {a.v}), dead time = {1e3 * tau_d:.0f} ms, lag T = {1e3 * T:.0f} ms")
    print(f"-> set delay_compensation:={tau_d + 0.5 * T:.2f}  (dead time + half the lag)")
    print(f"raw data: {a.out}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
