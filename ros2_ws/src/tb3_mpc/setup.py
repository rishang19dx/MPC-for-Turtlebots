from glob import glob
import os

from setuptools import find_packages, setup

package_name = "tb3_mpc"


def data(dir_, pattern="*"):
    return (os.path.join("share", package_name, dir_), glob(os.path.join(dir_, pattern)))


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        data("launch", "*.launch.py"),
        data("config", "*.yaml"),
        data("rviz", "*.rviz"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ctrl-foxtrot",
    maintainer_email="noreply@example.com",
    description="NMPC trajectory tracking + obstacle avoidance for TurtleBot3 Burger",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tracking_controller = tb3_mpc.nodes.tracking_controller:main",
            "obstacle_extractor = tb3_mpc.nodes.obstacle_extractor:main",
            "cmd_vel_relay = tb3_mpc.nodes.cmd_vel_relay:main",
            "obstacle_spawner = tb3_mpc.nodes.obstacle_spawner:main",
        ],
    },
)
