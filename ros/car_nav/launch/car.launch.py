"""The whole NAV2 mode on the real skid-steer car: sensors + nav stack + motor bridge.

    ros2 launch car_nav car.launch.py map:=/data/nav/maps/campus.yaml   # AMCL on a saved map
    ros2 launch car_nav car.launch.py mode:=load map_name:=lab          # AMCL on a map of the web UI's store
    ros2 launch car_nav car.launch.py slam:=true                        # build a map while driving (= mode:=create)
    ros2 launch car_nav car.launch.py mode:=flat                        # no map: odometry only, goals relative to the start
    ros2 launch car_nav car.launch.py                                   # pick the mode in the web UI (:8080)
    ros2 launch car_nav car.launch.py motors:=false ...                 # everything, wheels stay 0 0

Starts sensors.launch.py (RPLIDAR, Astra, TF, self-mask), nav.launch.py
(scan_upright, rf2o + adaptive_odom, AMCL + pose_gate or slam_toolbox, Nav2,
PD follower, collision monitor, cmd_vel -> /car/wheels/cmd) and
ugv_serial_bridge (/car/wheels/cmd -> ugv-controller on /dev/ugv, its IMU ->
/imu/data_raw). Send goals from RViz ("Nav2 Goal") or the dashboard.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("car_nav")
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value="", description="map yaml for AMCL (mode load)"),
        DeclareLaunchArgument("map_name", default_value="", description="a map of the web UI's store"),
        DeclareLaunchArgument("mode", default_value="", description="load | create | flat | idle ('' = from slam / map)"),
        DeclareLaunchArgument("slam", default_value="false", description="same as mode:=create"),
        DeclareLaunchArgument("controller", default_value="pd"),
        DeclareLaunchArgument("ugv_port", default_value="/dev/ugv"),
        DeclareLaunchArgument("motors", default_value="true",
                              description="false: the bridge only reads the IMU and sends 0 0"),
        DeclareLaunchArgument("depth", default_value="false",
                              description="true: start the Astra and use the depth camera as a second "
                                          "source for the safety guard (low boxes, table tops)"),
        DeclareLaunchArgument("enable_astra", default_value=arg("depth"),
                              description="the Astra; follows depth (false saves CPU)"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "sensors.launch.py")),
            launch_arguments={"enable_astra": arg("enable_astra")}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "nav.launch.py")),
            launch_arguments={"map": arg("map"), "slam": arg("slam"), "mode": arg("mode"),
                              "map_name": arg("map_name"), "controller": arg("controller"),
                              "depth": arg("depth")}.items()),
        Node(package="car_nav", executable="ugv_serial_bridge", name="ugv_serial_bridge", output="screen",
             parameters=[{"port": arg("ugv_port"),
                          "enable_motors": ParameterValue(arg("motors"), value_type=bool)}]),
    ])
