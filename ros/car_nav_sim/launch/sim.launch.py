"""NAV2 simulation: simulated car + LiDAR, then the SAME nav stack as the car.

    # headless 2D backend (no GPU), drive a mission, write a report, exit
    ros2 launch car_nav_sim sim.launch.py mission:=loop_left
    # Gazebo Harmonic with GUI + RViz, send goals by hand with "2D Goal Pose"
    ros2 launch car_nav_sim sim.launch.py backend:=gazebo gui:=true rviz:=true

Arguments:
  backend      lite | gazebo
  world        world name in worlds/ (or a path)            [test_floor]
               campus_path = the school tile map (maps/tile_maps/campus-path.json)
  start        car start: 'x,y,yaw' or an anchor id of the world (AAD-1, gym-01,
               F.218, F.203, ...); '' = the world's start. Ignored with a mission.
  mission      mission name in missions/ ('' = drive by hand in RViz)
  odom_source  rf2o (like the car) | truth (perfect odometry)
  slam         true = build the map with slam_toolbox instead of AMCL on the known map
  rviz, gui    show RViz / the Gazebo window
  report_dir   where mission reports go                      [/data/sim]
  seed         LiDAR noise / dropout random seed (lite)      [1]
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


def _start_pose(world_name: str, ref: str) -> list[float]:
    """'x,y,yaw' or an anchor id of the world (worlds/campus_path.yaml: AAD-1, gym-01, ...)."""
    if "," in ref:
        vals = [float(v) for v in ref.split(",")]
        return vals if len(vals) == 3 else [*vals, 0.0]
    with open(sim_config.resolve("worlds", world_name), "r", encoding="utf-8") as fh:
        anchors = (yaml.safe_load(fh) or {}).get("anchors") or {}
    if ref not in anchors:
        raise RuntimeError(f"start:={ref!r} is not 'x,y,yaw' or an anchor of {world_name}: {sorted(anchors)}")
    return [float(v) for v in anchors[ref]["pose"]]


def _rviz_config(world_name: str) -> str:
    """sim.rviz with the top-down view fitted to this world (the school map is
    56 x 38 m, the test floor 9 x 6 m)."""
    with open(os.path.join(get_package_share_directory("car_nav_sim"), "rviz", "sim.rviz"), "r",
              encoding="utf-8") as fh:
        text = fh.read()
    with open(sim_config.resolve("worlds", world_name), "r", encoding="utf-8") as fh:
        x0, y0, x1, y1 = (float(v) for v in (yaml.safe_load(fh) or {})["bounds"])
    scale = 0.9 * min(1100.0 / (x1 - x0), 850.0 / (y1 - y0))  # px per metre (window minus panels)
    text = text.replace("      Scale: 80\n      X: 4.5\n      Y: 3\n",
                        f"      Scale: {scale:.1f}\n      X: {(x0 + x1) / 2:.2f}\n      Y: {(y0 + y1) / 2:.2f}\n")
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, "sim.rviz")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path


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
    if not start and arg("start"):
        start = _start_pose(world_name, arg("start"))
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
        # The sim's "world" frame IS the map frame (the map is rasterised from the world),
        # so RViz can draw the true pose (/sim/ground_truth, frame world) next to AMCL's.
        Node(package="tf2_ros", executable="static_transform_publisher", name="tf_map_to_world",
             arguments=["--frame-id", "map", "--child-frame-id", "world"], parameters=[sim_time]),
        Node(package="car_nav", executable="scan_self_mask", name="scan_self_mask",
             output="screen", parameters=[sim_time]),
    ]

    if backend == "lite":
        actions.append(Node(package="car_nav_sim", executable="lite_sim", name="lite_sim", output="screen",
                            parameters=[{"world": world_name, "start": start_param,
                                         "seed": int(arg("seed"))}]))
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

    if not mission and not slam:
        # Driving by hand in RViz: give AMCL the true start (the mission runner does this itself).
        actions.append(Node(package="car_nav_sim", executable="initial_pose", name="sim_initial_pose",
                            output="screen", parameters=[{"use_sim_time": True, "world": world_name,
                                                          "start": start_param}]))

    if arg("rviz").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", output="screen",
                            arguments=["-d", _rviz_config(world_name)],
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
        DeclareLaunchArgument("start", default_value="", description="'x,y,yaw' or a world anchor id"),
        DeclareLaunchArgument("mission", default_value=""),
        DeclareLaunchArgument("odom_source", default_value="rf2o"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("report_dir", default_value="/data/sim"),
        DeclareLaunchArgument("exit_after_mission", default_value="true"),
        DeclareLaunchArgument("seed", default_value="1", description="LiDAR noise seed (lite backend)"),
        OpaqueFunction(function=_setup),
    ])
