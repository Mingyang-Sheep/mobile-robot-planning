#!/usr/bin/env python3
"""Small helpers for debug Path publication diagnostics."""

import rospy
from nav_msgs.msg import Path
import tf


def publish_empty(path_pub, frame_id, algorithm, topic, reason):
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.header.frame_id = frame_id or "map"
    path_pub.publish(path_msg)
    log_fn = rospy.loginfo if reason in ("startup_clear", "goal_not_received") else rospy.logwarn
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
                "[DebugPlanner] algorithm=%s impl=py tf_lookup=true map_frame=%s robot_frame=%s x=%.3f y=%.3f",
                algorithm,
                map_frame,
                robot_frame,
                translation[0],
                translation[1],
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
        "[DebugPlanner] algorithm=%s impl=py map_ready=true topic=%s size=%dx%d resolution=%.3f frame=%s",
        algorithm,
        topic,
        width,
        height,
        resolution,
        frame_id,
    )


def log_goal_received(algorithm, topic, frame_id, x, y, map_ready):
    rospy.loginfo(
        "[DebugPlanner] algorithm=%s impl=py goal_received=true topic=%s frame=%s x=%.3f y=%.3f map_ready=%s",
        algorithm,
        topic,
        frame_id,
        x,
        y,
        str(bool(map_ready)).lower(),
    )


def log_success(algorithm, topic, points, reason="ok"):
    rospy.loginfo(
        "[DebugPlanner] algorithm=%s impl=py success=true points=%d reason=%s topic=%s",
        algorithm,
        points,
        reason,
        topic,
    )
