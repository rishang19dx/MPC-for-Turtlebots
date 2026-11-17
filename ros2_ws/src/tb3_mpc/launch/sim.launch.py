"""Gazebo Classic simulation: TB3 Burger + obstacles + extractor + tracking controller.

Example
  ros2 launch tb3_mpc sim.launch.py controller:=nmpc \
      reference:='{"type": "figure8"}' ref_offset:='0.1,-0.1,0.3' obstacle_seed:=3

The stock Burger SDF is patched at launch time so that
  * /odom comes from wheel encoders (odometry_source 0) instead of Gazebo's
    perfect world pose -> realistic drift, like the real robot
  * a p3d plugin publishes the true pose on /ground_truth (world frame)
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

P3D = """
    <plugin name="ground_truth" filename="libgazebo_ros_p3d.so">
      <ros><remapping>odom:=ground_truth</remapping></ros>
      <frame_name>world</frame_name>
      <body_name>base_link</body_name>
      <update_rate>30</update_rate>
    </plugin>
  </model>"""


def patched_burger_sdf() -> str:
    src = os.path.join(get_package_share_directory("turtlebot3_gazebo"), "models",
                       "turtlebot3_burger", "model.sdf")
    with open(src) as f:
        sdf = f.read()
    if "<odometry_source>" not in sdf:
        sdf = sdf.replace("<publish_wheel_tf>false</publish_wheel_tf>",
                          "<publish_wheel_tf>false</publish_wheel_tf>\n      <odometry_source>0</odometry_source>")
    sdf = sdf.replace("</model>", P3D, 1)
    out = os.path.join(tempfile.gettempdir(), "tb3_mpc_burger_gt.sdf")
    with open(out, "w") as f:
        f.write(sdf)
    return out


def triple(s: str):
    return [float(v) for v in s.replace(" ", "").split(",")]


def launch_setup(context, *args, **kwargs):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    pkg = get_package_share_directory("tb3_mpc")
    params_file = os.path.join(pkg, "config", "tb3_mpc.yaml")
    spawn = triple(lc("spawn_pose"))
    offset = triple(lc("ref_offset"))
    obstacles = [float(v) for v in lc("obstacles").split(",") if v.strip()] or [0.0]
    use_obstacles = lc("use_obstacles").lower() == "true"

    common = {"use_sim_time": True}
    controller = Node(package="tb3_mpc", executable="tracking_controller", output="screen",
                      parameters=[params_file, common, {
                          "controller": lc("controller"), "safety_filter": lc("safety_filter").lower() == "true",
                          "reference": lc("reference"), "ref_offset": offset,
                          "use_obstacles": use_obstacles, "run_name": lc("run_name"),
                          "start_delay": 4.0, "exit_on_finish": lc("exit_on_finish").lower() == "true",
                          "delay_compensation": float(lc("delay_compensation")),
                          "log_dir": os.path.expanduser(lc("log_dir")),
                      }])
    actions = [controller]
    if lc("exit_on_finish").lower() == "true":  # batch runs: tear everything down when done
        actions.append(RegisterEventHandler(OnProcessExit(target_action=controller,
                                                          on_exit=[Shutdown(reason="run finished")])))
    return actions + [
        Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
             arguments=["-entity", "burger", "-file", patched_burger_sdf(),
                        "-x", str(spawn[0]), "-y", str(spawn[1]), "-z", "0.01", "-Y", str(spawn[2])]),
        Node(package="tb3_mpc", executable="obstacle_spawner", output="screen",
             parameters=[common, {"reference": lc("reference"), "ref_offset": offset,
                                  "robot_spawn_pose": spawn, "obstacles": obstacles,
                                  "random_seed": int(lc("obstacle_seed")),
                                  "n_random": int(lc("n_obstacles"))}],
             condition=IfCondition(lc("use_obstacles"))),
        Node(package="tb3_mpc", executable="obstacle_extractor", output="screen",
             parameters=[params_file, common], condition=IfCondition(lc("use_obstacles"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(pkg, "rviz", "tb3_mpc.rviz")],
             parameters=[common], condition=IfCondition(lc("rviz"))),
    ]


def generate_launch_description():
    gazebo_ros = get_package_share_directory("gazebo_ros")
    tb3_gazebo = get_package_share_directory("turtlebot3_gazebo")
    world = os.path.join(tb3_gazebo, "worlds", "empty_world.world")
    args = [
        DeclareLaunchArgument("controller", default_value="nmpc", description="nmpc|kanayama|samson|smc"),
        DeclareLaunchArgument("safety_filter", default_value="false", description="CBF-QP filter for baselines"),
        DeclareLaunchArgument("reference", default_value='{"type": "figure8"}', description="JSON reference spec"),
        DeclareLaunchArgument("ref_offset", default_value="0.0,0.0,0.0",
                              description="robot start pose relative to the reference start (x,y,yaw)"),
        DeclareLaunchArgument("spawn_pose", default_value="0.0,0.0,0.0", description="robot pose in Gazebo world"),
        DeclareLaunchArgument("use_obstacles", default_value="true"),
        DeclareLaunchArgument("obstacles", default_value="", description="x,y,r,... in reference frame"),
        DeclareLaunchArgument("obstacle_seed", default_value="-1", description=">=0: random obstacles on path"),
        DeclareLaunchArgument("n_obstacles", default_value="2"),
        DeclareLaunchArgument("delay_compensation", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("exit_on_finish", default_value="false"),
        DeclareLaunchArgument("run_name", default_value=""),
        DeclareLaunchArgument("log_dir", default_value="~/tb3_mpc_logs/sim"),
    ]
    return LaunchDescription(args + [
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzserver.launch.py")),
                                 launch_arguments={"world": world}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo_ros, "launch", "gzclient.launch.py")),
                                 condition=IfCondition(LaunchConfiguration("gui"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo, "launch", "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items()),
        OpaqueFunction(function=launch_setup),
    ])
