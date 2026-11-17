"""World + car geometry -> Gazebo Harmonic SDF (no ROS imports).

The car is a kinematic body moved by the VelocityControl system at its
origin, and its origin IS base_link (the centre of the skid-steer car,
midway between the left and right wheels). Commanding (v, omega) there is
exactly differential-drive kinematics, turning on the spot included. The
LiDAR link takes car.yaml's roll (0: mounted upright), so Gazebo produces
scans in the same frame convention as the car.
"""
from __future__ import annotations

import math

from .world import World


def _box(name: str, x: float, y: float, sx: float, sy: float, yaw: float, h: float, color: str) -> str:
    geo = f"<geometry><box><size>{sx} {sy} {h}</size></box></geometry>"
    return f"""
    <model name="{name}"><static>true</static><pose>{x} {y} {h / 2} 0 0 {yaw}</pose>
      <link name="link"><collision name="c">{geo}</collision>
        <visual name="v">{geo}<material><diffuse>{color}</diffuse><ambient>{color}</ambient></material></visual></link></model>"""


def _cyl(name: str, x: float, y: float, r: float, h: float) -> str:
    geo = f"<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>"
    return f"""
    <model name="{name}"><static>true</static><pose>{x} {y} {h / 2} 0 0 0</pose>
      <link name="link"><collision name="c">{geo}</collision>
        <visual name="v">{geo}<material><diffuse>0.8 0.3 0.1 1</diffuse></material></visual></link></model>"""


def car_sdf(car_cfg: dict, sim_cfg: dict, start: tuple[float, float, float]) -> str:
    fp = car_cfg["footprint"]
    laser = car_cfg.get("laser") or {}
    drive = car_cfg.get("drive") or {}
    lidar = sim_cfg.get("lidar") or {}
    lx = float(laser.get("x") or 0.0)
    lyaw = float(laser.get("yaw") or 0.0)
    lz = float(laser.get("z") or 0.42)
    lroll = float(laser.get("roll") or 0.0)
    xf, xr, hw = float(fp["x_front"]), float(fp["x_rear"]), float(fp["half_width"])
    length, xc = xf - xr, (xf + xr) / 2.0
    track = float(drive.get("track_width_m") or drive.get("assumed_track_width_m") or 0.22)
    axle = 0.5 * (xf - xr) - 0.07          # front / rear axle x, wheels near the body corners
    samples = int(lidar.get("samples", 720))
    inc = 2 * math.pi / samples
    wheels = ""
    for i, (wx, wy) in enumerate([(xc + axle, track / 2), (xc + axle, -track / 2),
                                  (xc - axle, track / 2), (xc - axle, -track / 2)]):
        wheels += f"""
        <visual name="wheel{i}"><pose>{wx} {wy} 0.05 1.5708 0 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>0.04</length></cylinder></geometry>
          <material><diffuse>0.1 0.1 0.1 1</diffuse></material></visual>"""
    x, y, yaw = start
    return f"""
    <model name="car">
      <pose>{x} {y} 0 0 0 {yaw}</pose>
      <link name="base_link">
        <gravity>false</gravity>
        <inertial><pose>{xc} 0 0.12 0 0 0</pose><mass>3.0</mass>
          <inertia><ixx>0.03</ixx><iyy>0.08</iyy><izz>0.1</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="body"><pose>{xc} 0 0.13 0 0 0</pose>
          <geometry><box><size>{length} {2 * hw} 0.14</size></box></geometry></collision>
        <visual name="body"><pose>{xc} 0 0.13 0 0 0</pose>
          <geometry><box><size>{length} {2 * hw} 0.14</size></box></geometry>
          <material><diffuse>0.1 0.4 0.9 1</diffuse><ambient>0.1 0.4 0.9 1</ambient></material></visual>
        <visual name="centre_marker"><pose>0 0 0.21 0 0 0</pose>
          <geometry><box><size>0.03 {2 * hw} 0.01</size></box></geometry>
          <material><diffuse>1 0.9 0 1</diffuse></material></visual>{wheels}
      </link>
      <link name="laser_link">
        <pose>{lx} 0 {lz} {lroll} 0 {lyaw}</pose>
        <gravity>false</gravity>
        <inertial><mass>0.1</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <visual name="v"><geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
          <material><diffuse>0.2 0.2 0.2 1</diffuse></material></visual>
        <sensor name="lidar" type="gpu_lidar">
          <gz_frame_id>laser</gz_frame_id>
          <topic>scan</topic>
          <update_rate>{float(lidar.get("rate_hz", 7.0))}</update_rate>
          <always_on>1</always_on>
          <visualize>true</visualize>
          <lidar>
            <scan><horizontal><samples>{samples}</samples><resolution>1</resolution>
              <min_angle>{-math.pi}</min_angle><max_angle>{math.pi - inc}</max_angle></horizontal>
              <vertical><samples>1</samples><resolution>1</resolution><min_angle>0</min_angle><max_angle>0</max_angle></vertical></scan>
            <range><min>{float(lidar.get("range_min_m", 0.15))}</min><max>{float(lidar.get("range_max_m", 12.0))}</max><resolution>0.01</resolution></range>
            <noise><type>gaussian</type><mean>0</mean><stddev>{float(lidar.get("noise_std_m", 0.01))}</stddev></noise>
          </lidar>
        </sensor>
      </link>
      <joint name="laser_joint" type="fixed"><parent>base_link</parent><child>laser_link</child></joint>
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <odom_frame>world</odom_frame>
        <robot_base_frame>base_link</robot_base_frame>
        <odom_publish_frequency>50</odom_publish_frequency>
        <dimensions>2</dimensions>
      </plugin>
    </model>"""


def world_sdf(world: World, car_cfg: dict, sim_cfg: dict, start: tuple[float, float, float] | None = None) -> str:
    models = []
    for i, b in enumerate(world.boxes):
        models.append(_box(f"box{i}", b.cx, b.cy, b.sx, b.sy, b.yaw, b.height, "0.6 0.6 0.6 1"))
    for i, c in enumerate(world.cylinders):
        models.append(_cyl(f"cyl{i}", c.cx, c.cy, c.r, c.height))
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world.name}">
    <physics name="1ms" type="ignored"><max_step_size>0.004</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><direction>-0.4 0.2 -0.9</direction></light>
    <model name="ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        <material><diffuse>0.85 0.85 0.8 1</diffuse><ambient>0.85 0.85 0.8 1</ambient></material></visual></link></model>
    {''.join(models)}
    {car_sdf(car_cfg, sim_cfg, start or world.start)}
  </world>
</sdf>
"""
