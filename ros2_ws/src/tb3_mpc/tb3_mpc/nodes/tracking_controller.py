"""Trajectory-tracking controller node (NMPC or a classical baseline).

Subscribes
  /odom (nav_msgs/Odometry)            state feedback (odom frame, REP-105)
  /obstacles (std_msgs/Float32MultiArray)  inflated circles [x, y, r]*K in odom
  /scan (sensor_msgs/LaserScan)        forward-sector proximity stop only
  /estop (std_msgs/Bool)               latched emergency stop
Publishes
  cmd_vel topic (Twist or TwistStamped, see ``stamped_cmd_vel``)
  /mpc/reference_path, /mpc/predicted_path, /mpc/actual_path (nav_msgs/Path)
  /mpc/diagnostics (std_msgs/Float64MultiArray, layout in DIAG_FIELDS)
and writes one CSV row per control tick to ``log_dir``.

Safety layers: stale-odom / stale-obstacle watchdogs, proximity stop, e-stop,
shared saturation (box + wheel diamond + rate limits), zero command on exit.
"""
from __future__ import annotations

import csv
import json
import math
import os
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray
from nav_msgs.msg import Path

from tb3_mpc.core import make_controller
from tb3_mpc.core.controllers import KanayamaGains, SamsonGains, SMCGains, tracking_error
from tb3_mpc.core.model import BURGER, rk4_step, saturate
from tb3_mpc.core.nmpc import NMPCConfig
from tb3_mpc.core.references import make_reference
from tb3_mpc.core.scenarios import reference_frame_in
from tb3_mpc.nodes.common import declare_dataclass_params, make_path, yaw_from_quat

DIAG_FIELDS = ["t", "solve_ms", "success", "e_pos", "e_lat", "e_head", "v", "w",
               "v_raw", "w_raw", "n_obstacles", "min_clearance", "state"]
STATES = {"waiting": 0, "running": 1, "finished": 2, "stopped_safety": 3, "estop": 4}


class TrackingController(Node):
    def __init__(self):
        super().__init__("tracking_controller")
        p = self.declare_parameter
        p("controller", "nmpc")                 # nmpc | kanayama | samson | smc
        p("safety_filter", False)               # CBF-QP filter for baselines
        p("rate_hz", 10.0)
        p("reference", '{"type": "figure8"}')   # JSON spec, see core.references.make_reference
        p("ref_offset", [0.0, 0.0, 0.0])        # robot start pose relative to reference start
        p("start_delay", 2.0)                   # s after the first odom message
        p("hold_after_end", 3.0)                # s of tracking the final pose after the ref ends
        p("exit_on_finish", False)
        p("odom_topic", "/odom")
        p("cmd_vel_topic", "/cmd_vel")
        p("stamped_cmd_vel", False)
        p("use_obstacles", True)
        p("obstacles_topic", "/obstacles")
        p("odom_timeout", 0.3)
        p("obstacle_timeout", 0.6)
        p("proximity_stop_distance", 0.13)      # m from LiDAR, forward sector
        p("proximity_sector_deg", 30.0)
        p("delay_compensation", 0.0)            # s; 0 disables forward prediction
        p("log_dir", os.path.expanduser("~/tb3_mpc_logs"))
        p("run_name", "")
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self.ctrl_name = g("controller")
        self.dt = 1.0 / g("rate_hz")
        nmpc_p = declare_dataclass_params(self, "nmpc", NMPCConfig)
        gains = {
            "kanayama": declare_dataclass_params(self, "kanayama", KanayamaGains),
            "samson": declare_dataclass_params(self, "samson", SamsonGains),
            "smc": declare_dataclass_params(self, "smc", SMCGains),
        }
        params = nmpc_p if self.ctrl_name == "nmpc" else gains[self.ctrl_name]
        if self.ctrl_name == "nmpc":
            params["dt"] = self.dt
        self.controller = make_controller(self.ctrl_name, params, dt=self.dt,
                                          safety_filter=g("safety_filter"),
                                          max_obstacles=int(nmpc_p.get("max_obstacles", 8)))
        self.get_logger().info(f"controller={getattr(self.controller, 'name', self.ctrl_name)} "
                               f"dt={self.dt:.3f}s")

        self.ref_local = make_reference(json.loads(g("reference")))
        self.ref_offset = np.asarray(g("ref_offset"), float)
        self.ref = None  # reference expressed in odom, fixed at start
        self.use_obstacles = g("use_obstacles")
        self.stamped = g("stamped_cmd_vel")
        self.tau = float(g("delay_compensation"))

        # ---------------- state shared between callbacks
        self.lock = threading.Lock()
        self.odom = None            # (stamp_sec, np.array([x, y, th]))
        self.obstacles = None       # (stamp_sec, np.ndarray (K,3))
        self.front_min = math.inf
        self.estop = False
        self.cmd_hist = deque(maxlen=50)  # (time_sec, u) commands sent
        self.u_sent = np.zeros(2)
        self.state = "waiting"
        self.done = False
        self.t_first_odom = None
        self.t0 = None
        self.actual_xy = deque(maxlen=3000)

        # ---------------- ROS interfaces
        sensors = ReentrantCallbackGroup()
        self.create_subscription(Odometry, g("odom_topic"), self.on_odom, 10, callback_group=sensors)
        self.create_subscription(Float32MultiArray, g("obstacles_topic"), self.on_obstacles, 10,
                                 callback_group=sensors)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data,
                                 callback_group=sensors)
        self.create_subscription(Bool, "/estop", self.on_estop, 10, callback_group=sensors)
        cmd_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(TwistStamped if self.stamped else Twist,
                                             g("cmd_vel_topic"), cmd_qos)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ref_pub = self.create_publisher(Path, "/mpc/reference_path", latched)
        self.pred_pub = self.create_publisher(Path, "/mpc/predicted_path", 1)
        self.act_pub = self.create_publisher(Path, "/mpc/actual_path", 1)
        self.diag_pub = self.create_publisher(Float64MultiArray, "/mpc/diagnostics", 10)
        self.timer = self.create_timer(self.dt, self.on_timer,
                                       callback_group=MutuallyExclusiveCallbackGroup())

        # ---------------- CSV log
        os.makedirs(g("log_dir"), exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = g("run_name") or f"{self.controller.name}_{self.ref_local.name}"
        self.csv_path = os.path.join(g("log_dir"), f"{stamp}_{name}.csv")
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(["t", "x", "y", "th", "xr", "yr", "thr", "vr", "wr", "v", "w", "v_raw", "w_raw",
                           "solve_ms", "success", "fallback", "n_obs", "min_clearance", "odom_age",
                           "state"])
        self.get_logger().info(f"logging to {self.csv_path}")

    # ------------------------------------------------------------ callbacks
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        x = np.array([p.position.x, p.position.y, yaw_from_quat(p.orientation)])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self.lock:
            self.odom = (stamp, x)
            if self.t_first_odom is None:
                self.t_first_odom = self.now()

    def on_obstacles(self, msg: Float32MultiArray):
        o = np.asarray(msg.data, float).reshape(-1, 3)
        with self.lock:
            self.obstacles = (self.now(), o)

    def on_scan(self, msg: LaserScan):
        r = np.asarray(msg.ranges, float)
        ang = msg.angle_min + msg.angle_increment * np.arange(len(r))
        ang = (ang + np.pi) % (2 * np.pi) - np.pi
        sector = np.radians(self.get_parameter("proximity_sector_deg").value)
        valid = np.isfinite(r) & (r > max(msg.range_min, 0.05)) & (np.abs(ang) < sector)
        with self.lock:
            self.front_min = float(r[valid].min()) if valid.any() else math.inf

    def on_estop(self, msg: Bool):
        with self.lock:
            if msg.data and not self.estop:
                self.get_logger().warn("E-STOP received")
            self.estop = bool(msg.data)

    # ------------------------------------------------------------ helpers
    def publish_cmd(self, u):
        if self.stamped:
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = "base_footprint"
            m.twist.linear.x, m.twist.angular.z = float(u[0]), float(u[1])
        else:
            m = Twist()
            m.linear.x, m.angular.z = float(u[0]), float(u[1])
        self.cmd_pub.publish(m)

    def stop(self, reason: str | None = None):
        self.publish_cmd(np.zeros(2))
        self.u_sent = np.zeros(2)
        if reason:
            self.get_logger().warn(reason, throttle_duration_sec=1.0)

    def compensate(self, x, t_meas, t_now):
        """Predict the state at t_now + tau using the commands actually sent.

        Command sent at time s is assumed to act on the robot from s + tau.
        """
        if self.tau <= 0:
            return x, 0.0
        t_end = t_now + self.tau
        hist = list(self.cmd_hist)
        t, h = t_meas, 0.02
        while t < t_end - 1e-9:
            step = min(h, t_end - t)
            u = np.zeros(2)
            for ts, uc in hist:
                if ts + self.tau <= t:
                    u = uc
            x = rk4_step(x, u, step)
            t += step
        return x, t_end - t_now

    # ------------------------------------------------------------ main loop
    def on_timer(self):
        t_now = self.now()
        with self.lock:
            odom, obstacles, front_min, estop = self.odom, self.obstacles, self.front_min, self.estop
        if estop:
            self.state = "estop"
            return self.stop("e-stop active")
        if odom is None:
            return
        if self.state == "waiting":
            if t_now - self.t_first_odom < self.get_parameter("start_delay").value:
                return
            # Fix the reference in odom: robot start = ref start (+) offset
            T = reference_frame_in(odom[1], self.ref_local, self.ref_offset)
            self.ref = self.ref_local.transformed(T)
            self.ref_pub.publish(make_path(self.ref.x[::10], self.ref.y[::10], self.ref.theta[::10],
                                           "odom", self.get_clock().now().to_msg()))
            self.t0 = t_now
            self.controller.reset()
            self.state = "running"
            self.get_logger().info(f"start: robot {np.round(odom[1], 3)}, ref start "
                                   f"{np.round(self.ref.start_pose, 3)}, duration {self.ref.duration:.1f}s")
        if self.state in ("finished", "stopped_safety"):
            return

        t = t_now - self.t0
        if t > self.ref.duration + self.get_parameter("hold_after_end").value:
            self.state = "finished"
            self.stop()
            self.get_logger().info(f"finished; log: {self.csv_path}")
            # (raising inside a MultiThreadedExecutor callback is swallowed; main() polls this flag)
            self.done = bool(self.get_parameter("exit_on_finish").value)
            return

        # ---- watchdogs
        odom_age = t_now - odom[0]
        if odom_age > self.get_parameter("odom_timeout").value:
            return self.stop(f"stale odom ({odom_age:.2f}s) -> zero command")
        obs_list = None
        if self.use_obstacles:
            if obstacles is None or t_now - obstacles[0] > self.get_parameter("obstacle_timeout").value:
                return self.stop("no fresh obstacle list -> zero command")
            obs_list = obstacles[1]

        # ---- control
        x_meas = odom[1]
        x_plan, lead = self.compensate(x_meas, odom[0], t_now)
        u_raw, info = self.controller.compute(t + lead, x_plan, self.ref, obs_list)
        u = saturate(u_raw, BURGER, u_prev=self.u_sent, dt=self.dt)
        if front_min < self.get_parameter("proximity_stop_distance").value and u[0] > 0:
            u[0] = 0.0
            self.get_logger().warn(f"proximity stop ({front_min:.2f} m)", throttle_duration_sec=1.0)
        if hasattr(self.controller, "set_applied_input"):
            self.controller.set_applied_input(u)
        self.publish_cmd(u)
        self.u_sent = u
        self.cmd_hist.append((t_now, u.copy()))

        # ---- diagnostics / logging
        xr, ur = self.ref.sample(t)
        xr, ur = xr[:, 0], ur[:, 0]
        e = tracking_error(x_meas, xr)
        n_obs = 0 if obs_list is None else len(obs_list)
        clear = math.inf
        if n_obs:
            clear = float(np.min(np.hypot(obs_list[:, 0] - x_meas[0], obs_list[:, 1] - x_meas[1]) - obs_list[:, 2]))
        solve_ms = 1e3 * info.get("solve_time", float("nan"))
        succ = float(info.get("success", True))
        diag = Float64MultiArray()
        diag.data = [t, solve_ms, succ, float(np.hypot(e[0], e[1])), e[1], e[2], u[0], u[1],
                     float(u_raw[0]), float(u_raw[1]), float(n_obs), clear, float(STATES[self.state])]
        self.diag_pub.publish(diag)
        self.csv.writerow([f"{t:.3f}", *np.round(x_meas, 4), *np.round(xr, 4), *np.round(ur, 4),
                           *np.round(u, 4), *np.round(u_raw, 4), f"{solve_ms:.2f}", int(succ),
                           info.get("fallback", "none"), n_obs, f"{clear:.3f}", f"{odom_age:.3f}",
                           self.state])
        stamp = self.get_clock().now().to_msg()
        self.actual_xy.append((x_meas[0], x_meas[1], x_meas[2]))
        if "pred_X" in info:
            P = info["pred_X"]
            self.pred_pub.publish(make_path(P[0], P[1], P[2], "odom", stamp))
        if len(self.actual_xy) % 5 == 0:
            a = np.array(self.actual_xy)
            self.act_pub.publish(make_path(a[:, 0], a[:, 1], a[:, 2], "odom", stamp))

    def close(self):
        try:
            for _ in range(3):
                self.publish_cmd(np.zeros(2))
                time.sleep(0.02)
        finally:
            self.csv_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = TrackingController()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.done:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        # Publish zero before the context goes away. On Humble SIGINT may already have
        # invalidated it, which is why the on-robot cmd_vel_relay timeout exists.
        try:
            node.close()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
