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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# Ensure the scripts directory is on the module path so that
# ``optimal.astar_planner_py`` etc. can be imported by name.
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

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


class DebugPathSupervisor:
    def __init__(self, algorithm):
        self.algorithm = algorithm
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.path_sub = rospy.Subscriber(self.path_topic, Path, self.path_callback, queue_size=10)
        self.goal_sub = rospy.Subscriber(self.goal_topic, PoseStamped, self.goal_callback, queue_size=1)
        self.publish_empty("map")
        rospy.loginfo("Cleared debug path topic: %s", self.path_topic)

    def goal_callback(self, msg):
        self.publish_empty(msg.header.frame_id or "map")

    def publish_empty(self, frame_id):
        empty_path = Path()
        empty_path.header.stamp = rospy.Time.now()
        empty_path.header.frame_id = frame_id
        self.path_pub.publish(empty_path)

    def path_callback(self, msg):
        rospy.loginfo(
            "[DebugPlanner] algorithm=%s topic=%s success=%s points=%d frame=%s",
            self.algorithm,
            self.path_topic,
            str(bool(msg.poses)).lower(),
            len(msg.poses),
            msg.header.frame_id,
        )


def main():
    rospy.init_node("python_planner_node", anonymous=False)

    algorithm = rospy.get_param("~algorithm", "")
    if algorithm not in _ALGORITHM_MAP:
        rospy.logfatal(
            "Unknown algorithm '%s'. Available: %s",
            algorithm, ", ".join(sorted(_ALGORITHM_MAP)),
        )
        sys.exit(1)

    module_name, class_name = _ALGORITHM_MAP[algorithm]
    debug_path_supervisor = DebugPathSupervisor(algorithm)

    rospy.loginfo("Loading planner: %s.%s", module_name, class_name)

    module = importlib.import_module(module_name)
    planner_class = getattr(module, class_name)
    planner_class()

    rospy.spin()


if __name__ == "__main__":
    main()
