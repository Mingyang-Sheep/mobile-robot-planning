#!/usr/bin/env python3
"""Unified Python planner entry point.

Dynamically loads and runs the planner matching the ``algorithm`` ROS parameter,
analogous to the C++ pluginlib mechanism in planner_plugin_node.

Usage (launched by planner.launch with impl:=py):
    rosrun mr_traditional_planner python_planner_node.py _algorithm:=astar
"""

import importlib
import os
import sys

import rospy
from nav_msgs.msg import Path

# Ensure the scripts directory is on the module path so that
# ``optimal.astar_planner_py`` etc. can be imported by name.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from utils import debug_path

_ALGORITHM_MAP = {
    "astar":        ("optimal.astar_planner_py",             "AStarPlannerNode"),
    "dijkstra":     ("optimal.dijkstra_planner_py",          "DijkstraPlannerNode"),
    "dstar":        ("optimal.dstar_planner_py",             "DStarPlannerNode"),
    "dstar_lite":   ("optimal.dstar_lite_planner_py",        "DStarLitePlannerNode"),
    "theta_star":   ("optimal.theta_star_planner_py",        "ThetaStarPlannerNode"),
    "rrt_star":     ("optimal.rrt_star_planner_py",          "RRTStarPlannerNode"),
    "dwa":          ("optimal.dwa_planner_py",               "DynamicWindowApproachPlannerNode"),
    "cubic_spline": ("optimal.cubic_spline_planner_py",      "CubicSplinePlannerNode"),
    "bcd":          ("coverage.bcd_planner_py",              "BcdPlannerNode"),
    "stc":          ("coverage.stc_planner_py",              "StcPlannerNode"),
}


def main():
    rospy.init_node("python_planner_node", anonymous=False)

    algorithm = rospy.get_param("~algorithm", "")
    if algorithm not in _ALGORITHM_MAP:
        path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        rospy.sleep(0.2)
        debug_path.publish_empty(path_pub, "map", algorithm or "unknown", path_topic, "unsupported_algorithm")
        rospy.logfatal(
            "Unknown algorithm '%s'. Available: %s",
            algorithm, ", ".join(sorted(_ALGORITHM_MAP)),
        )
        sys.exit(1)

    module_name, class_name = _ALGORITHM_MAP[algorithm]

    rospy.loginfo(
        "[DebugPlanner] algorithm=%s impl=py loading=%s.%s",
        algorithm,
        module_name,
        class_name,
    )

    try:
        module = importlib.import_module(module_name)
        planner_class = getattr(module, class_name)
        planner_class()
    except Exception as exc:
        path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        path_pub = rospy.Publisher(path_topic, Path, queue_size=1, latch=True)
        rospy.sleep(0.2)
        debug_path.publish_empty(path_pub, "map", algorithm, path_topic, "exception")
        rospy.logfatal("[DebugPlanner] algorithm=%s impl=py exception=%s", algorithm, exc)
        raise

    rospy.spin()


if __name__ == "__main__":
    main()
