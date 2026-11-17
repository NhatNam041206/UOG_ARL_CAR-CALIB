"""Put ros/car_nav and ros/car_nav_sim on sys.path for ROS-free tests."""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
for sub in ("ros/car_nav", "ros/car_nav_sim"):
    p = str(ROOT / sub)
    if p not in sys.path:
        sys.path.insert(0, p)

CAR_YAML = ROOT / "ros" / "car_nav" / "config" / "car.yaml"
NAV2_YAML = ROOT / "ros" / "car_nav" / "config" / "nav2_params.yaml"
BT_XML = ROOT / "ros" / "car_nav" / "behavior_trees" / "navigate_forward_only.xml"
SIM_DIR = ROOT / "ros" / "car_nav_sim"
