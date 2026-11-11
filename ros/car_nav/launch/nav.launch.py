"""NAV2 navigation stack, shared by the simulator and the real car.

    ros2 launch car_nav nav.launch.py map:=/path/map.yaml            # AMCL on a saved map
    ros2 launch car_nav nav.launch.py slam:=true                     # build a map while driving
    ros2 launch car_nav nav.launch.py odom_source:=external ...       # someone else publishes odom->base_link

Expects base_link -> laser TF and /scan_filtered (sensors.launch.py on the
car, sim.launch.py in the simulator). Starts:

  scan_upright      /scan_filtered -> /scan_nav (frame laser_upright) + its static TF
  rf2o              laser odometry, odom -> base_link   (odom_source:=rf2o)
  map_server + AMCL map -> odom                         (slam:=false)
  slam_toolbox      map -> odom + /map                  (slam:=true)
  planner (Smac Hybrid-A*, DUBIN forward only), controller (RPP),
  behavior server (wait only), bt_navigator (no spin/backup),
  collision_monitor (cmd_vel_nav -> cmd_vel, stop zones)
  pd_path_follower   PD path follower (controller:=pd, default): FollowPath action
                     pd_follow_path -> cmd_vel_nav. controller:=rpp uses Pure Pursuit.
  cmd_vel_to_control /cmd_vel -> "<angle>,<drive_code>" on /car/raspi5/control (ROS only)

Nothing here talks MQTT: on the car, the WO-02 forwarder copies
/car/raspi5/control to MQTT only while the drive mode is NAV2.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

from car_nav import config_loader
from car_nav.actuation import table_from_cfg

NAV_NODES = ["planner_server", "controller_server", "behavior_server", "bt_navigator", "collision_monitor"]


def _upright_tf(context, *args, **kwargs):
    cfg = config_loader.load()
    laser = config_loader.laser_pose(cfg)
    table = table_from_cfg(cfg)
    msgs = [LogInfo(msg=(
        f"[car_nav] steering: wheelbase {table.cfg.wheelbase_m} m, min turning radius "
        f"{table.min_turning_radius:.2f} m ({'measured' if table.measured else 'PLACEHOLDER table, not measured'})"
    ))]
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    msgs.append(Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_base_to_laser_upright",
        arguments=[
            "--x", f"{laser.x}", "--y", f"{laser.y}", "--z", f"{laser.z}",
            "--roll", "0", "--pitch", "0", "--yaw", f"{laser.yaw}",
            "--frame-id", "base_link", "--child-frame-id", "laser_upright",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    ))
    return msgs


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("car_nav")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    slam = LaunchConfiguration("slam")
    odom_source = LaunchConfiguration("odom_source")
    bt_xml = LaunchConfiguration("bt_xml")
    controller = LaunchConfiguration("controller")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml,
            "default_nav_to_pose_bt_xml": bt_xml,
        },
        convert_types=True,
    )
    common = {"output": "screen", "arguments": ["--ros-args", "--log-level", log_level]}
    remap_tf = [("/tf", "tf"), ("/tf_static", "tf_static")]

    localization = GroupAction(
        condition=UnlessCondition(slam),
        actions=[
            Node(package="nav2_map_server", executable="map_server", name="map_server",
                 parameters=[params], remappings=remap_tf, **common),
            Node(package="nav2_amcl", executable="amcl", name="amcl",
                 parameters=[params], remappings=remap_tf, **common),
            Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                 name="lifecycle_manager_localization", output="screen",
                 parameters=[{"use_sim_time": use_sim_time, "autostart": autostart,
                              "node_names": ["map_server", "amcl"]}]),
        ],
    )
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
        ),
        condition=IfCondition(slam),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": LaunchConfiguration("slam_params_file"),
            "autostart": autostart,
        }.items(),
    )

    navigation = GroupAction(actions=[
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=[params], remappings=remap_tf, **common),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[params], remappings=remap_tf + [("cmd_vel", "cmd_vel_nav")], **common),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             parameters=[params], remappings=remap_tf + [("cmd_vel", "cmd_vel_nav")], **common),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             parameters=[params], remappings=remap_tf, **common),
        Node(package="car_nav", executable="pd_path_follower", name="pd_path_follower",
             parameters=[params], remappings=remap_tf + [("cmd_vel", "cmd_vel_nav")],
             condition=IfCondition(PythonExpression(["'", controller, "' == 'pd'"])), **common),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor",
             parameters=[params], remappings=remap_tf, **common),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "autostart": autostart, "node_names": NAV_NODES}]),
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=os.environ.get(
            "CAR_NAV_NAV2_PARAMS", os.path.join(share, "config", "nav2_params.yaml"))),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("controller", default_value="pd",
                              description="pd (PD path follower) | rpp (Nav2 Regulated Pure Pursuit)"),
        DeclareLaunchArgument("bt_xml", default_value=PythonExpression([
            "'", os.path.join(share, "behavior_trees", "navigate_forward_only"), "' + "
            "('_pd' if '", controller, "' == 'pd' else '') + '.xml'"])),
        DeclareLaunchArgument("map", default_value="", description="map yaml for AMCL (slam:=false)"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("odom_source", default_value="rf2o", description="rf2o | external"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_upright_tf),
        Node(package="car_nav", executable="scan_upright", name="scan_upright", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
             name="rf2o_laser_odometry", output="screen", parameters=[params],
             # rf2o logs every scan at INFO and "Waiting for laser_scans" at WARN (harmless)
             arguments=["--ros-args", "--log-level", "rf2o_laser_odometry:=error"],
             condition=IfCondition(PythonExpression(["'", odom_source, "' == 'rf2o'"]))),
        localization,
        slam_launch,
        navigation,
        Node(package="car_nav", executable="cmd_vel_to_control", name="cmd_vel_to_control", output="screen",
             parameters=[params], remappings=[("control", "/car/raspi5/control")]),
    ])
