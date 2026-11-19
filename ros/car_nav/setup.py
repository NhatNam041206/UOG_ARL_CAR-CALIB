from glob import glob

from setuptools import setup

package_name = "car_nav"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/behavior_trees", glob("behavior_trees/*.xml")),
        ("share/" + package_name + "/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="NAV2 mode sensor bring-up for the car-calib skid-steer car",
    license="MIT",
    entry_points={
        "console_scripts": [
            "scan_self_mask = car_nav.scan_self_mask:main",
            "front_range_logger = car_nav.front_range_logger:main",
            "scan_probe = car_nav.scan_probe:main",
            "scan_upright = car_nav.scan_upright:main",
            "cmd_vel_to_control = car_nav.cmd_vel_to_control:main",
            "pd_path_follower = car_nav.pd_path_follower:main",
            "adaptive_odom = car_nav.adaptive_odom:main",
            "pose_gate = car_nav.pose_gate:main",
            "ugv_serial_bridge = car_nav.ugv_serial_bridge:main",
            "dynamic_scan = car_nav.dynamic_scan:main",
            "safety_guard = car_nav.safety_guard:main",
            "depth_guard_source = car_nav.depth_guard_source:main",
            "ui_bridge = car_nav.ui_bridge:main",
            "flat_origin = car_nav.flat_origin:main",
        ],
    },
)
