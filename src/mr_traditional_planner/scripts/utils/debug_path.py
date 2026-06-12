#!/usr/bin/env python3
"""Small helpers for debug Path publication diagnostics."""

import functools
import traceback

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf


def log_node_start(algorithm, path_topic, executable, script_file, script_dir, module_file, cwd):
    rospy.loginfo(
        "[DebugPlanner] node_start script=%s node=%s algorithm=%s impl=py executable=%s script_dir=%s debug_path_module=%s cwd=%s path_topic=%s",
        script_file,
        rospy.get_name(),
        algorithm,
        executable,
        script_dir,
        module_file,
        cwd,
        path_topic,
    )


def log_algorithm_selected(algorithm):
    rospy.loginfo("[DebugPlanner] algorithm_selected=%s impl=py", algorithm)


def log_module_loaded(algorithm, module_name, class_name, module_file, object_id):
    rospy.loginfo(
        "[DebugPlanner] module_loaded path=%s class=%s object_id=%s algorithm=%s impl=py module=%s",
        module_file,
        class_name,
        object_id,
        algorithm,
        module_name,
    )


def log_node_ready(algorithm, planner):
    path_pub = getattr(planner, "path_pub", None)
    resolved_path_topic = getattr(path_pub, "resolved_name", getattr(planner, "path_topic", "unknown"))
    robot_frames = getattr(planner, "robot_frames", None)
    if isinstance(robot_frames, (list, tuple)):
        robot_frames = ",".join(str(frame) for frame in robot_frames)
    elif robot_frames is None:
        robot_frames = getattr(planner, "robot_frame", "unknown")

    rospy.loginfo(
        "[DebugPlanner] algorithm=%s impl=py node_ready=true map_topic=%s goal_topic=%s path_topic=%s map_frame=%s robot_frames=%s",
        algorithm,
        getattr(planner, "map_topic", "unknown"),
        getattr(planner, "goal_topic", "unknown"),
        resolved_path_topic,
        getattr(planner, "map_frame", "unknown"),
        robot_frames,
    )


def log_subscriptions_ready(planner):
    map_callback = getattr(getattr(planner, "map_callback", None), "__name__", "unknown")
    goal_callback = getattr(getattr(planner, "goal_callback", None), "__name__", "unknown")
    rospy.loginfo(
        "[DebugPlanner] subscriptions_ready map=%s goal=%s object_id=%s map_callback=%s.%s goal_callback=%s.%s",
        getattr(planner, "map_topic", "unknown"),
        getattr(planner, "goal_topic", "unknown"),
        id(planner),
        planner.__class__.__name__,
        map_callback,
        planner.__class__.__name__,
        goal_callback,
    )


def traced_callback(callback_name):
    def decorate(func):
        @functools.wraps(func)
        def wrapped(self, *args, **kwargs):
            algorithm = getattr(self, "algorithm", "unknown")
            rospy.loginfo(
                "[DebugPlanner] %s_enter object_id=%s algorithm=%s impl=py function=%s.%s",
                callback_name,
                id(self),
                algorithm,
                self.__class__.__name__,
                func.__name__,
            )
            try:
                return func(self, *args, **kwargs)
            except Exception as exc:
                rospy.logerr(
                    "[DebugPlanner] algorithm=%s impl=py %s_exception object_id=%s exception=%s",
                    algorithm,
                    callback_name,
                    id(self),
                    exc,
                )
                rospy.logerr(traceback.format_exc())
                if callback_name == "goal_callback" and hasattr(self, "publish_failure"):
                    self.publish_failure("exception")
                return None

        return wrapped

    return decorate


def publish_empty(path_pub, frame_id, algorithm, topic, reason):
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = frame_id or "map"
    path_pub.publish(path_msg)
    log_fn = rospy.loginfo if reason in ("startup_clear", "goal_not_received") else rospy.logwarn
    log_fn(
        "[DebugPlanner] publish topic=%s points=0 frame=%s algorithm=%s impl=py reason=%s",
        topic,
        path_msg.header.frame_id,
        algorithm,
        reason,
    )
    log_fn(
        "[DebugPlanner] algorithm=%s impl=py success=false points=0 reason=%s topic=%s",
        algorithm,
        reason,
        topic,
    )


def robot_frame_candidates(default_frame):
    configured = rospy.get_param("~robot_frame", default_frame)
    raw_candidates = rospy.get_param("~robot_frame_candidates", "")
    candidates = [configured]
    if isinstance(raw_candidates, str) and raw_candidates:
        candidates.extend(frame.strip() for frame in raw_candidates.split(",") if frame.strip())
    elif isinstance(raw_candidates, list):
        candidates.extend(str(frame).strip() for frame in raw_candidates if str(frame).strip())
    candidates.extend(["base_footprint", "base_link"])

    unique = []
    for frame in candidates:
        if frame and frame not in unique:
            unique.append(frame)
    return unique


def lookup_start_pose(tf_listener, map_frame, robot_frames, timeout, algorithm):
    errors = []
    for robot_frame in robot_frames:
        try:
            tf_listener.waitForTransform(
                map_frame, robot_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            translation, _ = tf_listener.lookupTransform(
                map_frame, robot_frame, rospy.Time(0)
            )
            rospy.loginfo(
                "[DebugPlanner] tf_success base_frame=%s start_x=%.3f start_y=%.3f algorithm=%s impl=py map_frame=%s",
                robot_frame,
                translation[0],
                translation[1],
                algorithm,
                map_frame,
            )
            return translation[0], translation[1], robot_frame
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            errors.append("%s: %s" % (robot_frame, exc))

    rospy.logwarn(
        "[DebugPlanner] algorithm=%s impl=py success=false points=0 reason=tf_lookup_failed map_frame=%s robot_frames=%s errors=%s",
        algorithm,
        map_frame,
        ",".join(robot_frames),
        " | ".join(errors),
    )
    return None


def log_map_ready(algorithm, topic, width, height, resolution, frame_id):
    rospy.loginfo_once(
        "[DebugPlanner] map_ready width=%d height=%d resolution=%.3f algorithm=%s impl=py topic=%s frame=%s",
        width,
        height,
        resolution,
        algorithm,
        topic,
        frame_id,
    )


def log_goal_received(algorithm, topic, frame_id, x, y, map_ready):
    rospy.loginfo(
        "[DebugPlanner] goal_received x=%.3f y=%.3f frame=%s algorithm=%s impl=py topic=%s map_ready=%s",
        x,
        y,
        frame_id,
        algorithm,
        topic,
        str(bool(map_ready)).lower(),
    )


def log_grid_points(algorithm, start_x, start_y, goal_x, goal_y):
    rospy.loginfo(
        "[DebugPlanner] grid_start=(%d,%d) grid_goal=(%d,%d) algorithm=%s impl=py",
        start_x,
        start_y,
        goal_x,
        goal_y,
        algorithm,
    )


def log_plan_call(algorithm):
    rospy.loginfo("[DebugPlanner] plan_call algorithm=%s impl=py", algorithm)


def log_plan_return(algorithm, points):
    rospy.loginfo("[DebugPlanner] plan_return points=%d algorithm=%s impl=py", points, algorithm)


def publish_world_path(path_pub, map_frame, algorithm, topic, path_points):
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = map_frame or "map"

    for world_x, world_y in path_points:
        pose = PoseStamped()
        pose.header.stamp = path_msg.header.stamp
        pose.header.frame_id = path_msg.header.frame_id
        pose.pose.position.x = world_x
        pose.pose.position.y = world_y
        pose.pose.orientation.w = 1.0
        path_msg.poses.append(pose)

    path_pub.publish(path_msg)
    rospy.loginfo(
        "[DebugPlanner] publish topic=%s points=%d frame=%s algorithm=%s impl=py",
        topic,
        len(path_msg.poses),
        path_msg.header.frame_id,
        algorithm,
    )
    log_success(algorithm, topic, len(path_msg.poses))


def publish_grid_path(path_pub, grid_map, path_indices, map_frame, algorithm, topic):
    path_points = []
    for linear_index in path_indices:
        grid_x, grid_y = grid_map.index_to_grid(linear_index)
        path_points.append(grid_map.grid_to_world(grid_x, grid_y))
    publish_world_path(path_pub, map_frame, algorithm, topic, path_points)


def log_success(algorithm, topic, points, reason="ok"):
    rospy.loginfo(
        "[DebugPlanner] algorithm=%s impl=py success=true points=%d reason=%s topic=%s",
        algorithm,
        points,
        reason,
        topic,
    )
