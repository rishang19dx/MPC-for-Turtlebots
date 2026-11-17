"""Robot (Raspberry Pi) side: TurtleBot3 bringup + cmd_vel safety relay.

  export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 ROS_DOMAIN_ID=<unique>
  ros2 launch tb3_mpc robot.launch.py out_stamped:=false

Check the driver's cmd_vel type first:  ros2 topic info -v /cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("turtlebot3_bringup"), "launch", "robot.launch.py")),
        condition=IfCondition(LaunchConfiguration("bringup")))
    return LaunchDescription([
        DeclareLaunchArgument("bringup", default_value="true"),
        DeclareLaunchArgument("out_stamped", default_value="false",
                              description="true if /cmd_vel on the robot is TwistStamped"),
        DeclareLaunchArgument("timeout", default_value="0.3"),
        bringup,
        Node(package="tb3_mpc", executable="cmd_vel_relay", output="screen",
             parameters=[{"out_stamped": LaunchConfiguration("out_stamped"),
                          "timeout": LaunchConfiguration("timeout")}]),
    ])
