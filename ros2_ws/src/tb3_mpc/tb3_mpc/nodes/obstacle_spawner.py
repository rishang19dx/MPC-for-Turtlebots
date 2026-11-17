"""Spawn cylinder obstacles in Gazebo (world frame) for a seeded or explicit scenario.

Obstacles are specified in the *reference frame* (like every scenario in
core.scenarios) and mapped to the Gazebo world using the robot's spawn pose:
    T_world<-R = T_world<-robot0 * inv(T_R<-robot0).
Also publishes the true (un-inflated) circles, latched, on
/ground_truth_obstacles for evaluation.
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float32MultiArray

from tb3_mpc.core.references import make_reference
from tb3_mpc.core.scenarios import obstacles_on_path, reference_frame_in, transform_points

CYLINDER_SDF = """<?xml version='1.0'?>
<sdf version='1.6'><model name='{name}'><static>true</static>
<link name='link'>
 <collision name='c'><geometry><cylinder><radius>{r}</radius><length>0.3</length></cylinder></geometry></collision>
 <visual name='v'><geometry><cylinder><radius>{r}</radius><length>0.3</length></cylinder></geometry>
  <material><ambient>0.8 0.3 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse></material></visual>
</link></model></sdf>"""


class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__("obstacle_spawner")
        self.declare_parameter("reference", '{"type": "figure8"}')
        self.declare_parameter("ref_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("robot_spawn_pose", [0.0, 0.0, 0.0])   # world frame
        self.declare_parameter("obstacles", [0.0])   # flat [x, y, r, ...] in ref frame; [0.0] = none
        self.declare_parameter("random_seed", -1)    # >= 0: place obstacles on the path
        self.declare_parameter("n_random", 2)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        ref = make_reference(json.loads(g("reference")))
        flat = list(g("obstacles"))
        if g("random_seed") >= 0:
            obs = obstacles_on_path(ref, np.random.default_rng(g("random_seed")), g("n_random"))
        elif len(flat) >= 3:
            obs = np.asarray(flat[: 3 * (len(flat) // 3)], float).reshape(-1, 3)
        else:
            obs = np.zeros((0, 3))
        T = reference_frame_in(np.asarray(g("robot_spawn_pose"), float), ref, g("ref_offset"))
        self.obs_world = transform_points(T, obs) if len(obs) else obs

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.gt_pub = self.create_publisher(Float32MultiArray, "/ground_truth_obstacles", latched)
        self.spawn = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete = self.create_client(DeleteEntity, "/delete_entity")

    def run(self):
        if not self.spawn.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("/spawn_entity not available")
            return
        for i, (x, y, r) in enumerate(self.obs_world):
            name = f"mpc_obstacle_{i}"
            if self.delete.wait_for_service(timeout_sec=1.0):
                rclpy.spin_until_future_complete(self, self.delete.call_async(DeleteEntity.Request(name=name)),
                                                 timeout_sec=2.0)
            req = SpawnEntity.Request()
            req.name = name
            req.xml = CYLINDER_SDF.format(name=name, r=float(r))
            req.initial_pose.position.x, req.initial_pose.position.y = float(x), float(y)
            req.initial_pose.position.z = 0.15
            fut = self.spawn.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            self.get_logger().info(f"spawned {name} at ({x:.2f}, {y:.2f}) r={r:.2f}")
        msg = Float32MultiArray()
        msg.data = [float(v) for v in np.asarray(self.obs_world).ravel()]
        self.gt_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleSpawner()
    try:
        node.run()
        rclpy.spin(node)  # keep the latched ground-truth topic alive
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
