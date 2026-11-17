"""NAV2 mode sensor bring-up (WO-01): RPLIDAR A1 + Astra Pro + static TF
+ self-mask. Sends NOTHING to the car.

    ros2 launch car_nav sensors.launch.py [enable_lidar:=true] [enable_astra:=true]

Geometry comes from car.yaml (env CAR_NAV_CONFIG, else the package copy).
"""
from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from car_nav import config_loader


def _static_tf(name: str, parent: str, child: str, pose) -> Node:
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", f"{pose.x}", "--y", f"{pose.y}", "--z", f"{pose.z}",
            "--roll", f"{pose.roll}", "--pitch", f"{pose.pitch}", "--yaw", f"{pose.yaw}",
            "--frame-id", parent, "--child-frame-id", child,
        ],
    )


def _setup(context, *args, **kwargs):
    cfg = config_loader.load()
    actions = [LogInfo(msg=f"[car_nav] geometry from {cfg['_path']}")]
    missing = config_loader.unmeasured(cfg)
    if missing:
        actions.append(LogInfo(msg=(
            "[car_nav] WARNING: NOT MEASURED YET (using 0): " + ", ".join(missing)
            + ". Measure them and edit car.yaml. Distances along x are off by the "
            "missing offsets until then."
        )))
    laser = config_loader.laser_pose(cfg)
    camera = config_loader.camera_pose(cfg)
    actions.append(_static_tf("tf_base_to_laser", "base_link", "laser", laser))
    actions.append(_static_tf("tf_base_to_camera", "base_link", "camera_link", camera))
    actions.append(LogInfo(msg=(
        f"[car_nav] laser z={laser.z} roll={laser.roll:.3f} ({'upside down' if abs(laser.roll) > 1.0 else 'upright'}); "
        f"camera z={camera.z} pitch={math.degrees(camera.pitch):.1f} deg down"
    )))
    return actions


def generate_launch_description() -> LaunchDescription:
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_astra = LaunchConfiguration("enable_astra")
    lidar_port = LaunchConfiguration("lidar_port")
    astra_point_cloud = LaunchConfiguration("astra_point_cloud")

    astra_launch = os.path.join(
        get_package_share_directory("astra_camera"), "launch", "astra_pro.launch.xml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("enable_astra", default_value="true"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument("astra_point_cloud", default_value="true"),
        OpaqueFunction(function=_setup),
        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="sllidar_node",
            condition=IfCondition(enable_lidar),
            parameters=[{
                "channel_type": "serial",
                "serial_port": lidar_port,
                "serial_baudrate": 115200,  # RPLIDAR A1
                "frame_id": "laser",
                # The A1 is mounted upright (car.yaml roll 0). An upside-down
                # mount would be handled by roll = pi in the TF only: setting
                # this too would cancel the correction out.
                "inverted": False,
                "angle_compensate": True,
            }],
            output="screen",
        ),
        Node(
            package="car_nav",
            executable="scan_self_mask",
            name="scan_self_mask",
            condition=IfCondition(enable_lidar),
            output="screen",
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
            condition=IfCondition(enable_astra),
            launch_arguments={
                "camera_name": "camera",
                # Astra Pro cannot stream IR and depth at once; IR is not needed.
                "enable_ir": "false",
                "enable_point_cloud": astra_point_cloud,
                "publish_tf": "true",
                "oni_log_level": "warning",
            }.items(),
        ),
    ])
