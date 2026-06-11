#!/usr/bin/env python3
import sys

import rospy


GLOBAL_PLANNERS = {
    "navfn",
    "astar",
    "dijkstra",
    "dstar",
    "dstar_lite",
    "theta_star",
    "rrt_star",
}
PATH_SMOOTHERS = {"none", "cubic_spline"}
LOCAL_PLANNERS = {"dwa"}
COVERAGE_PLANNERS = {"bcd", "stc"}


def fail(message):
    rospy.logfatal("[PlannerCompatibility] %s", message)
    sys.exit(1)


def main():
    rospy.init_node("planner_compatibility_validator")

    planning_mode = rospy.get_param("~planning_mode", "normal")
    global_planner = rospy.get_param("~global_planner", "navfn")
    path_smoother = rospy.get_param("~path_smoother", "none")
    local_planner = rospy.get_param("~local_planner", "dwa")
    coverage_planner = rospy.get_param("~coverage_planner", "stc")

    if planning_mode not in {"normal", "coverage"}:
        fail("planning_mode must be normal or coverage, got '%s'." % planning_mode)

    if local_planner not in LOCAL_PLANNERS:
        fail(
            "local_planner '%s' is not supported. Only dwa maps to "
            "dwa_local_planner/DWAPlannerROS right now." % local_planner
        )

    if path_smoother not in PATH_SMOOTHERS:
        fail("path_smoother '%s' is invalid. Valid: none, cubic_spline." % path_smoother)

    if global_planner in {"cubic_spline", "bcd", "stc", "dwa"}:
        fail("'%s' cannot be used as a normal global planner." % global_planner)

    if planning_mode == "normal":
        if global_planner not in GLOBAL_PLANNERS:
            fail("global_planner '%s' is invalid for normal navigation." % global_planner)
        if coverage_planner not in COVERAGE_PLANNERS:
            fail("coverage_planner '%s' is invalid. Valid: bcd, stc." % coverage_planner)
    else:
        if coverage_planner not in COVERAGE_PLANNERS:
            fail("coverage_planner '%s' is invalid for coverage mode." % coverage_planner)
        if path_smoother != "none":
            fail("coverage mode does not apply path_smoother; use path_smoother:=none.")

    rospy.loginfo(
        "[PlannerCompatibility] mode=%s global=%s smoother=%s local=%s coverage=%s",
        planning_mode,
        global_planner,
        path_smoother,
        local_planner,
        coverage_planner,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
