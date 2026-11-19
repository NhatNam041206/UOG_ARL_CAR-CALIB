"""The restartable part of the NAV2 stack: localization + Nav2 for one map mode.

    ros2 launch car_nav nav_stack.launch.py mode:=load map:=/path/map.yaml   # AMCL on a stored map
    ros2 launch car_nav nav_stack.launch.py mode:=create                     # slam_toolbox builds a map
    ros2 launch car_nav nav_stack.launch.py mode:=flat                       # no map: map == odom

Started by nav.launch.py (ui:=false) or as a child process of the web UI's
ui_bridge, which stops it and starts it again with another mode when you
switch. It holds exactly what depends on the map:

  load    map_server + AMCL (+ pose_gate: map -> odom) + lifecycle_manager_localization
  create  slam_toolbox (map -> odom, /map)
  flat    flat_origin (map -> odom = start of the mode), global costmap without static layer
  all     planner (Smac 2D), controller (RPP) / pd_path_follower, behavior server, bt_navigator,
          teleop_mux (web page joystick, in front of the collision monitor), collision_monitor, dynamic_scan, safety_guard, lifecycle_manager_navigation

Sensors, odometry (scan_upright, rf2o, adaptive_odom), cmd_vel_to_control, rosbridge
and ui_bridge stay in nav.launch.py and keep running while this restarts.
dynamic_scan has no /map in flat mode and passes every LiDAR point on (fallback: all).
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from car_nav.map_store import MapStore

NAV_NODES = ["planner_server", "controller_server", "behavior_server", "bt_navigator", "collision_monitor"]


def _footprint_str() -> str:
    from car_nav import config_loader
    fp = config_loader.footprint(config_loader.load())
    return (f"[[{fp.x_front}, {fp.half_width}], [{fp.x_front}, {-fp.half_width}], "
            f"[{fp.x_rear}, {-fp.half_width}], [{fp.x_rear}, {fp.half_width}]]")


def _stack(context, *args, **kwargs):
    share = get_package_share_directory("car_nav")
    a = lambda n: context.launch_configurations[n]  # noqa: E731
    mode = a("mode")
    use_sim_time = a("use_sim_time").lower() == "true"
    controller = a("controller")
    pose_gate = a("pose_gate").lower() == "true"
    autostart = a("autostart").lower() == "true"
    depth = a("depth").lower() == "true"
    map_yaml = a("map")
    if not map_yaml and a("map_name"):
        map_yaml = MapStore(a("data_dir")).yaml_path(a("map_name")) or ""
    if mode not in ("load", "create", "flat"):
        raise RuntimeError(f"mode must be load, create or flat, got {mode!r}")
    if mode == "load" and not map_yaml:
        raise RuntimeError("mode:=load needs map:=<yaml> or map_name:=<stored map>")
    bt_xml = a("bt_xml") or os.path.join(
        share, "behavior_trees", "navigate_forward_only" + ("_pd" if controller == "pd" else "") + ".xml")

    params = RewrittenYaml(
        source_file=a("params_file"),
        param_rewrites={"use_sim_time": str(use_sim_time).lower(), "yaml_filename": map_yaml,
                        "default_nav_to_pose_bt_xml": bt_xml,
                        # costmap footprints follow car.yaml (the web UI's Car tab edits it)
                        "footprint": _footprint_str()},
        convert_types=True,
    )
    common = {"output": "screen", "arguments": ["--ros-args", "--log-level", a("log_level")]}
    remap_tf = [("/tf", "tf"), ("/tf_static", "tf_static")]
    acts = [LogInfo(msg=f"[car_nav] map mode {mode}" + (f", map {map_yaml}" if mode == "load" else ""))]

    if mode == "load":
        acts += [
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 parameters=[params], remappings=remap_tf, **common),
            # with the gate, AMCL only publishes /amcl_pose; pose_gate owns map -> odom
            Node(package="nav2_amcl", executable="amcl", name="amcl",
                 parameters=[params, {"tf_broadcast": not pose_gate}], remappings=remap_tf, **common),
        ]
        if pose_gate:
            acts.append(Node(package="car_nav", executable="pose_gate", name="pose_gate",
                             parameters=[params], remappings=remap_tf, **common))
        acts.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                         name="lifecycle_manager_localization", output="screen",
                         parameters=[{"use_sim_time": use_sim_time, "autostart": autostart,
                                      "node_names": ["map_server", "amcl"]}]))
    elif mode == "create":
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
            launch_arguments={"use_sim_time": str(use_sim_time).lower(),
                              "slam_params_file": a("slam_params_file"),
                              "autostart": str(autostart).lower()}.items()))
    else:
        acts.append(Node(package="car_nav", executable="flat_origin", name="flat_origin", output="screen",
                         parameters=[{"use_sim_time": use_sim_time}], remappings=remap_tf))

    planner_params = [params]
    if mode == "flat":
        planner_params.append(os.path.join(share, "config", "nav2_flat_overrides.yaml"))
    pd_follow = Node(package="car_nav", executable="pd_path_follower", name="pd_path_follower",
                     parameters=[params, {"max_pose_age_s": float(a("max_pose_age_s"))}],
                     remappings=remap_tf + [("cmd_vel", "cmd_vel_nav")], **common)
    acts += [
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=planner_params, remappings=remap_tf, **common),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[params], remappings=remap_tf + [("cmd_vel", "cmd_vel_nav")], **common),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             parameters=[params], remappings=remap_tf + [("cmd_vel", "cmd_vel_nav")], **common),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             parameters=[params], remappings=remap_tf, **common),
        *([pd_follow] if controller == "pd" else []),
        # manual drive from the web page: /cmd_vel_nav (Nav2) or /cmd_vel_teleop -> /cmd_vel_mux -> collision monitor
        Node(package="car_nav", executable="teleop_mux", name="teleop_mux", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             parameters=[params], remappings=remap_tf, **common),
        Node(package="car_nav", executable="dynamic_scan", name="dynamic_scan",
             parameters=[params], remappings=remap_tf, **common),
        Node(package="car_nav", executable="safety_guard", name="safety_guard",
             parameters=[params, {"depth_enabled": depth}], **common),
        *([Node(package="car_nav", executable="depth_guard_source", name="depth_obstacles",
                parameters=[params], remappings=remap_tf, **common)] if depth else []),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": autostart, "node_names": NAV_NODES}]),
    ]
    return acts


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("car_nav")
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="load", description="load | create | flat"),
        DeclareLaunchArgument("map", default_value="", description="map yaml (mode:=load)"),
        DeclareLaunchArgument("map_name", default_value="", description="stored map name (mode:=load, instead of map:=)"),
        DeclareLaunchArgument("data_dir", default_value="/data/nav/ui", description="where the stored maps are (<data_dir>/maps)"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=os.environ.get(
            "CAR_NAV_NAV2_PARAMS", os.path.join(share, "config", "nav2_params.yaml"))),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("controller", default_value="pd", description="pd | rpp"),
        DeclareLaunchArgument("bt_xml", default_value="", description="'' = chosen from controller"),
        DeclareLaunchArgument("pose_gate", default_value="true"),
        DeclareLaunchArgument("max_pose_age_s", default_value="1.0"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("depth", default_value="false",
                              description="true: depth camera as a second safety guard source"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_stack),
    ])
