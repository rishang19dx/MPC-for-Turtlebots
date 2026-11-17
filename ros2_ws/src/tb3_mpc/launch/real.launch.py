"""Laptop side of a real-robot run: obstacle extractor + tracking controller (+ RViz).

The controller publishes to /cmd_vel_safe; the on-robot ``robot.launch.py``
relay forwards it to /cmd_vel with a timeout. Start the robot side FIRST.

  ros2 launch tb3_mpc real.launch.py controller:=nmpc reference:='{"type": "figure8", "a": 0.8}'
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    pkg = get_package_share_directory("tb3_mpc")
    params_file = os.path.join(pkg, "config", "tb3_mpc.yaml")
    offset = [float(v) for v in lc("ref_offset").split(",")]
    use_obstacles = lc("use_obstacles").lower() == "true"
    return [
        Node(package="tb3_mpc", executable="obstacle_extractor", output="screen",
             parameters=[params_file], condition=IfCondition(lc("use_obstacles"))),
        Node(package="tb3_mpc", executable="tracking_controller", output="screen",
             parameters=[params_file, {
                 "controller": lc("controller"), "safety_filter": lc("safety_filter").lower() == "true",
                 "reference": lc("reference"), "ref_offset": offset,
                 "use_obstacles": use_obstacles, "cmd_vel_topic": "/cmd_vel_safe",
                 "delay_compensation": float(lc("delay_compensation")),
                 "run_name": lc("run_name"), "log_dir": os.path.expanduser(lc("log_dir")),
                 "start_delay": 3.0,
             }]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(pkg, "rviz", "tb3_mpc.rviz")],
             condition=IfCondition(lc("rviz"))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("controller", default_value="nmpc"),
        DeclareLaunchArgument("safety_filter", default_value="false"),
        DeclareLaunchArgument("reference", default_value='{"type": "figure8", "a": 0.8}'),
        DeclareLaunchArgument("ref_offset", default_value="0.0,0.0,0.0"),
        DeclareLaunchArgument("use_obstacles", default_value="true"),
        DeclareLaunchArgument("delay_compensation", default_value="0.15",
                              description="s; set from the measured step-response dead time"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("run_name", default_value=""),
        DeclareLaunchArgument("log_dir", default_value="~/tb3_mpc_logs/real"),
        OpaqueFunction(function=launch_setup),
    ])
