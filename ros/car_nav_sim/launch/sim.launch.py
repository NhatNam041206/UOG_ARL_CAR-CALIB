"""NAV2 simulation: simulated car + LiDAR, then the SAME nav stack as the car.

    # headless 2D backend (no GPU), drive a mission, write a report, exit
    ros2 launch car_nav_sim sim.launch.py mission:=loop_left
    # Gazebo Harmonic with GUI + RViz, send goals by hand with "2D Goal Pose"
    ros2 launch car_nav_sim sim.launch.py backend:=gazebo gui:=true rviz:=true

Arguments:
  backend      lite | gazebo
  world        world name in worlds/ (or a path)            [test_floor]
  mission      mission name in missions/ ('' = drive by hand in RViz)
  odom_source  rf2o (like the car) | truth (perfect odometry)
  slam         true = build the map with slam_toolbox instead of AMCL on the known map
  rviz, gui    show RViz / the Gazebo window
  report_dir   where mission reports go                      [/data/sim]
"""
from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from car_nav import config_loader
from car_nav_sim import sim_config
from car_nav_sim.make_world import build

OUT_DIR = os.environ.get("CAR_NAV_SIM_OUT", "/tmp/car_nav_sim")


def _setup(context, *args, **kwargs):
    arg = lambda name: context.launch_configurations[name]  # noqa: E731
    backend = arg("backend")
    mission = arg("mission")
    odom_source = arg("odom_source")
    slam = arg("slam").lower() == "true"
    world_name = arg("world")
    start = None
    if mission:
        with open(sim_config.resolve("missions", mission), "r", encoding="utf-8") as fh:
            m = yaml.safe_load(fh) or {}
        world_name = m.get("world", world_name)
        start = m.get("start")
    map_yaml, sdf_path = build(world_name, OUT_DIR, start)
    start_param = [float(v) for v in start] if start else [0.0]
    sim_time = {"use_sim_time": True}

    car = config_loader.load()
    laser = config_loader.laser_pose(car)
    actions = [
        LogInfo(msg=f"[sim] backend={backend} world={world_name} mission={mission or '(none)'} "
                    f"odom={odom_source} slam={slam} map={map_yaml}"),
        LogInfo(msg=f"[sim] car.yaml={car['_path']} sim.yaml={sim_config.load_sim()['_path']}"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="tf_base_to_laser",
             arguments=["--x", f"{laser.x}", "--y", f"{laser.y}", "--z", f"{laser.z}",
                        "--roll", f"{laser.roll}", "--pitch", f"{laser.pitch}", "--yaw", f"{laser.yaw}",
                        "--frame-id", "base_link", "--child-frame-id", "laser"],
             parameters=[sim_time]),
        Node(package="car_nav", executable="scan_self_mask", name="scan_self_mask",
             output="screen", parameters=[sim_time]),
    ]

    if backend == "lite":
        actions.append(Node(package="car_nav_sim", executable="lite_sim", name="lite_sim", output="screen",
                            parameters=[{"world": world_name, "start": start_param}]))
    elif backend == "gazebo":
        gz_args = ["gz", "sim", "-r", sdf_path]
        if arg("gui").lower() != "true":
            gz_args[2:2] = ["-s", "--headless-rendering"]
        actions += [
            ExecuteProcess(cmd=gz_args, output="screen"),
            Node(package="ros_gz_bridge", executable="parameter_bridge", name="gz_bridge", output="screen",
                 arguments=[
                     "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                     "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                     "/model/car/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                     "/model/car/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                 ],
                 remappings=[("/model/car/odometry", "/sim/ground_truth")],
                 parameters=[sim_time]),
            Node(package="car_nav_sim", executable="gz_actuator", name="gz_actuator",
                 output="screen", parameters=[sim_time]),
        ]
    else:
        raise RuntimeError(f"backend must be lite or gazebo, got {backend!r}")

    if odom_source == "truth":
        actions.append(Node(package="car_nav_sim", executable="truth_odom_tf", name="truth_odom_tf",
                            output="screen", parameters=[sim_time]))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("car_nav"), "launch", "nav.launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "map": "" if slam else map_yaml,
            "slam": "true" if slam else "false",
            "odom_source": "rf2o" if odom_source == "rf2o" else "external",
        }.items(),
    ))

    if arg("rviz").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="log",
                            arguments=["-d", os.path.join(get_package_share_directory("car_nav_sim"),
                                                          "rviz", "sim.rviz")],
                            parameters=[sim_time]))

    if mission:
        runner = Node(package="car_nav_sim", executable="mission", name="sim_mission", output="screen",
                      parameters=[{"use_sim_time": True, "mission": mission, "report_dir": arg("report_dir"),
                                   "localizer": "slam" if slam else "amcl",
                                   "odom_source": odom_source,
                                   "start": start_param}])
        actions.append(runner)
        if arg("exit_after_mission").lower() == "true":
            actions.append(RegisterEventHandler(OnProcessExit(
                target_action=runner, on_exit=[EmitEvent(event=Shutdown(reason="mission finished"))])))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="lite"),
        DeclareLaunchArgument("world", default_value="test_floor"),
        DeclareLaunchArgument("mission", default_value=""),
        DeclareLaunchArgument("odom_source", default_value="rf2o"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("report_dir", default_value="/data/sim"),
        DeclareLaunchArgument("exit_after_mission", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
