from glob import glob

from setuptools import setup

package_name = "car_nav_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.yaml")),
        ("share/" + package_name + "/missions", glob("missions/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Simulator for the car_nav NAV2 mode",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lite_sim = car_nav_sim.lite_sim:main",
            "gz_actuator = car_nav_sim.gz_actuator:main",
            "truth_odom_tf = car_nav_sim.truth_odom_tf:main",
            "mission = car_nav_sim.mission:main",
            "make_world = car_nav_sim.make_world:main",
            "initial_pose = car_nav_sim.initial_pose:main",
        ],
    },
)
