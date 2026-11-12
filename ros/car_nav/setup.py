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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="NAV2 mode sensor bring-up for the car-calib rear-steer car",
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
        ],
    },
)
