"""NAV2 navigation stack, shared by the simulator and the real car.

    ros2 launch car_nav nav.launch.py map:=/path/map.yaml            # load: AMCL on a stored map
    ros2 launch car_nav nav.launch.py map_name:=lab                  # load a map of the UI's store
    ros2 launch car_nav nav.launch.py slam:=true                     # create: build a map while driving
    ros2 launch car_nav nav.launch.py mode:=flat                     # unknown / flat: map == odom
    ros2 launch car_nav nav.launch.py                                # nothing yet: choose in the web UI
    ros2 launch car_nav nav.launch.py odom_source:=external ...      # someone else publishes odom->base_link

Expects base_link -> laser TF and /scan_filtered (sensors.launch.py on the
car, sim.launch.py in the simulator). This file is the PERSISTENT part:

  scan_upright      /scan_filtered -> /scan_nav (frame laser_upright) + its static TF
  rf2o              laser odometry, odom -> base_link   (odom_source:=rf2o)
  adaptive_odom     rf2o + learned forward speed, odom -> base_link (odom_source:=adaptive, default)
  cmd_vel_to_control /cmd_vel -> "<left m/s>,<right m/s>" on /car/wheels/cmd (0 0 when /cmd_vel stops)
  ui:=true          rosbridge (ws :9090) + ui_bridge (web page http :8080)

The RESTARTABLE part (map_server + AMCL + pose_gate | slam_toolbox | flat_origin, Nav2,
PD follower, collision monitor, dynamic_scan, safety_guard) is nav_stack.launch.py. With
ui:=true ui_bridge starts it as a child process and restarts it when you change the map
mode in the web page; with ui:=false it is included here once.

Nothing here talks to the motors: on the car, the ugv-controller adapter
turns /car/wheels/cmd into the controller's own command.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

from car_nav import config_loader
from car_nav.actuation import WHEELS_TOPIC, drive_from_cfg
from car_nav.nav_modes import derive_mode

# arguments handed on to nav_stack.launch.py
STACK_ARGS = ("use_sim_time", "params_file", "slam_params_file", "controller", "bt_xml", "pose_gate",
              "max_pose_age_s", "autostart", "log_level")


def _upright_tf(context, *args, **kwargs):
    cfg = config_loader.load()
    laser = config_loader.laser_pose(cfg)
    drive = drive_from_cfg(cfg)
    msgs = [LogInfo(msg=(
        f"[car_nav] diff drive: track {drive.track_width_m:.3f} m "
        f"({'measured' if drive.measured else 'PLACEHOLDER, not measured'}), cruise {drive.cruise_speed_mps} m/s, "
        f"wheel limit {drive.max_wheel_speed_mps} m/s"
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


def _ui_and_stack(context, *args, **kwargs):
    """rosbridge + ui_bridge (which starts / restarts the stack), or the stack itself without the UI."""
    share = get_package_share_directory("car_nav")
    a = lambda n: context.launch_configurations[n]  # noqa: E731
    mode = derive_mode(a("mode"), a("slam"), a("map"), a("map_name"))
    use_sim_time = a("use_sim_time").lower() == "true"
    stack_args = [f"{k}:={a(k)}" for k in STACK_ARGS] + [f"data_dir:={a('ui_data_dir')}"]
    acts = []
    if a("ui").lower() == "true":
        acts += [
            Node(package="rosbridge_server", executable="rosbridge_websocket", name="rosbridge_websocket",
                 output="screen",
                 parameters=[{"port": int(a("ui_ws_port")), "use_sim_time": use_sim_time}],
                 arguments=["--ros-args", "--log-level", "warn"]),
            Node(package="car_nav", executable="ui_bridge", name="ui_bridge", output="screen",
                 remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
                 parameters=[{"use_sim_time": use_sim_time,
                              "http_port": int(a("ui_http_port")),
                              "rosbridge_port": int(a("ui_ws_port")),
                              "web_dir": a("ui_web_dir"),
                              "data_dir": a("ui_data_dir"),
                              "stack_args": stack_args,
                              "initial_mode": mode,
                              "initial_map": a("map"),
                              "initial_map_name": a("map_name"),
                              "seed_map_yaml": a("ui_seed_map"),
                              "seed_map_name": a("ui_seed_name"),
                              "seed_start": a("ui_seed_start")}]),
        ]
    elif mode == "idle":
        acts.append(LogInfo(msg="[car_nav] no map mode given and ui:=false: only sensors and odometry run "
                                "(use mode:=load map:=... | mode:=create | mode:=flat)"))
    else:
        args = {"mode": mode, "map": a("map"), "map_name": a("map_name"), "data_dir": a("ui_data_dir")}
        args.update({k: a(k) for k in STACK_ARGS})
        acts.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "nav_stack.launch.py")),
            launch_arguments=args.items()))
    return acts


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("car_nav")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    odom_source = LaunchConfiguration("odom_source")
    controller = LaunchConfiguration("controller")
    pose_gate = LaunchConfiguration("pose_gate")

    params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=os.environ.get(
            "CAR_NAV_NAV2_PARAMS", os.path.join(share, "config", "nav2_params.yaml"))),
        DeclareLaunchArgument("slam_params_file", default_value=os.path.join(share, "config", "slam_toolbox.yaml")),
        DeclareLaunchArgument("controller", default_value="pd",
                              description="pd (PD path follower) | rpp (Nav2 Regulated Pure Pursuit)"),
        DeclareLaunchArgument("bt_xml", default_value="", description="'' = chosen from controller"),
        DeclareLaunchArgument("mode", default_value="",
                              description="load | create | flat | idle ('' = from slam / map / map_name, else idle)"),
        DeclareLaunchArgument("map", default_value="", description="map yaml for mode load"),
        DeclareLaunchArgument("map_name", default_value="", description="a map of the UI's store (<ui_data_dir>/maps)"),
        DeclareLaunchArgument("slam", default_value="false", description="old spelling of mode:=create"),
        DeclareLaunchArgument("odom_source", default_value="adaptive",
                              description="adaptive (rf2o + learned speed) | rf2o | external"),
        DeclareLaunchArgument("pose_gate", default_value="true",
                              description="true: pose_gate holds back AMCL along-corridor corrections when blind"),
        DeclareLaunchArgument("max_pose_age_s", default_value="1.0",
                              description="pd_path_follower stops while the pose is older than this (s)"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("ui", default_value="true", description="web UI: rosbridge + ui_bridge"),
        DeclareLaunchArgument("ui_http_port", default_value="8080"),
        DeclareLaunchArgument("ui_ws_port", default_value="9090"),
        DeclareLaunchArgument("ui_web_dir", default_value="", description="'' = <car_nav share>/web"),
        DeclareLaunchArgument("ui_data_dir", default_value="/data/nav/ui",
                              description="web UI profiles, test records and maps (keep it on a volume)"),
        DeclareLaunchArgument("ui_seed_map", default_value="", description="simulator: map yaml to add to the store once"),
        DeclareLaunchArgument("ui_seed_name", default_value=""),
        DeclareLaunchArgument("ui_seed_start", default_value="", description="'x,y,yaw' default start pose of the seed"),
        DeclareLaunchArgument("log_level", default_value="info"),
        OpaqueFunction(function=_ui_and_stack),
        OpaqueFunction(function=_upright_tf),
        Node(package="car_nav", executable="scan_upright", name="scan_upright", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
             name="rf2o_laser_odometry", output="screen", parameters=[params],
             # rf2o logs every scan at INFO and "Waiting for laser_scans" at WARN (harmless)
             arguments=["--ros-args", "--log-level", "rf2o_laser_odometry:=error"],
             condition=IfCondition(PythonExpression(["'", odom_source, "' == 'rf2o'"]))),
        # odom_source:=adaptive: rf2o only measures (/odom_rf2o, no TF); adaptive_odom
        # adds the learned forward speed and owns /odom + odom -> base_link.
        Node(package="rf2o_laser_odometry", executable="rf2o_laser_odometry_node",
             name="rf2o_laser_odometry", output="screen",
             parameters=[params, {"publish_tf": False, "odom_topic": "/odom_rf2o"}],
             arguments=["--ros-args", "--log-level", "rf2o_laser_odometry:=error"],
             condition=IfCondition(PythonExpression(["'", odom_source, "' == 'adaptive'"]))),
        Node(package="car_nav", executable="adaptive_odom", name="adaptive_odom", output="screen",
             parameters=[params, {"localized_pose_topic": PythonExpression(
                 ["'/localized_pose' if '", pose_gate, "' == 'true' else '/amcl_pose'"])}],
             condition=IfCondition(PythonExpression(["'", odom_source, "' == 'adaptive'"]))),
        Node(package="car_nav", executable="cmd_vel_to_control", name="cmd_vel_to_control", output="screen",
             parameters=[params], remappings=[("control", WHEELS_TOPIC)]),
    ])
