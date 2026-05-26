#!/usr/bin/env python3
"""Limit, smooth, and republish velocity commands for RL-SAR navigation."""

import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def approach(current, target, max_delta):
    return current + clamp(target - current, -max_delta, max_delta)


def approach_with_limits(current, target, max_acc_delta, max_decel_delta):
    slowing_down = abs(target) < abs(current) or current * target < 0.0
    max_delta = max_decel_delta if slowing_down else max_acc_delta
    return approach(current, target, max_delta)


def get_bool_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        return value.lower() in ("1", "true", "yes", "on")
    return bool(value)


class CmdVelFilter:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/move_base_cmd_vel")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.rate = float(rospy.get_param("~rate", 20.0))
        self.timeout = float(rospy.get_param("~timeout", 0.5))
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")

        self.min_x = float(rospy.get_param("~min_x", 0.0))
        self.max_x = float(rospy.get_param("~max_x", 0.20))
        self.max_y = float(rospy.get_param("~max_y", 0.0))
        self.max_yaw = float(rospy.get_param("~max_yaw", 0.45))
        self.max_acc_x = float(rospy.get_param("~max_acc_x", 0.25))
        self.max_acc_y = float(rospy.get_param("~max_acc_y", 0.0))
        self.max_acc_yaw = float(rospy.get_param("~max_acc_yaw", 0.6))
        self.max_decel_x = float(rospy.get_param("~max_decel_x", self.max_acc_x))
        self.max_decel_y = float(rospy.get_param("~max_decel_y", self.max_acc_y))
        self.max_decel_yaw = float(rospy.get_param("~max_decel_yaw", self.max_acc_yaw))
        self.scale_x = float(rospy.get_param("~scale_x", 1.0))
        self.scale_y = float(rospy.get_param("~scale_y", 1.0))
        self.scale_yaw = float(rospy.get_param("~scale_yaw", 1.0))

        self.deadband_x = float(rospy.get_param("~deadband_x", 0.01))
        self.deadband_y = float(rospy.get_param("~deadband_y", 0.01))
        self.deadband_yaw = float(rospy.get_param("~deadband_yaw", 0.02))
        self.turn_in_place_enabled = get_bool_param("~turn_in_place_enabled", False)
        self.turn_yaw_enter = float(rospy.get_param("~turn_yaw_enter", 0.14))
        self.turn_yaw_exit = float(rospy.get_param("~turn_yaw_exit", 0.06))
        self.forward_yaw_limit = float(rospy.get_param("~forward_yaw_limit", 0.0))
        self.goal_stop_enabled = get_bool_param("~goal_stop_enabled", True)
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/current_goal")
        self.goal_xy_tolerance = float(rospy.get_param("~goal_xy_tolerance", 0.22))
        self.goal_stop_latch = get_bool_param("~goal_stop_latch", True)

        self.target = Twist()
        self.current = Twist()
        self.last_msg_time = rospy.Time(0)
        self.current_goal = None
        self.goal_reached = False
        self.goal_distance = None
        self.turning_in_place = False
        self.tf_listener = tf.TransformListener() if self.goal_stop_enabled else None

        self.publisher = rospy.Publisher(self.output_topic, Twist, queue_size=10)
        rospy.Subscriber(self.input_topic, Twist, self._cmd_cb, queue_size=10)
        if self.goal_stop_enabled:
            rospy.Subscriber(self.goal_topic, PoseStamped, self._goal_cb, queue_size=1)

    def _limit_axis(self, value, limit, deadband):
        if abs(value) < deadband or limit <= 0.0:
            return 0.0
        return clamp(value, -limit, limit)

    def _limit_linear_x(self, value):
        if self.max_x <= 0.0:
            return 0.0
        if self.min_x >= 0.0:
            if value <= self.deadband_x:
                return 0.0
            return clamp(value, self.min_x, self.max_x)
        if abs(value) < self.deadband_x:
            return 0.0
        return clamp(value, self.min_x, self.max_x)

    def _cmd_cb(self, msg):
        self.last_msg_time = rospy.Time.now()
        if self.goal_reached:
            self.target = Twist()
            return

        self.target.linear.x = self._limit_linear_x(msg.linear.x * self.scale_x)
        self.target.linear.y = self._limit_axis(msg.linear.y * self.scale_y, self.max_y, self.deadband_y)
        self.target.angular.z = self._limit_axis(msg.angular.z * self.scale_yaw, self.max_yaw, self.deadband_yaw)
        self._apply_turn_in_place_mode()

    def _apply_turn_in_place_mode(self):
        if not self.turn_in_place_enabled:
            return

        yaw_abs = abs(self.target.angular.z)
        if self.turning_in_place:
            self.turning_in_place = yaw_abs > self.turn_yaw_exit
        else:
            self.turning_in_place = yaw_abs >= self.turn_yaw_enter

        if self.turning_in_place:
            self.target.linear.x = 0.0
            self.target.linear.y = 0.0
            return

        self.target.linear.y = 0.0
        self.target.angular.z = self._limit_axis(
            self.target.angular.z,
            self.forward_yaw_limit,
            self.deadband_yaw,
        )

    def _goal_cb(self, msg):
        self.current_goal = msg
        self.goal_reached = False
        self.goal_distance = None

    def _update_goal_reached(self):
        if not self.goal_stop_enabled or self.current_goal is None:
            return False
        if self.goal_stop_latch and self.goal_reached:
            return True

        goal_frame = self.current_goal.header.frame_id or "map"
        try:
            (translation, _) = self.tf_listener.lookupTransform(
                goal_frame,
                self.base_frame,
                rospy.Time(0),
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

        dx = translation[0] - self.current_goal.pose.position.x
        dy = translation[1] - self.current_goal.pose.position.y
        self.goal_distance = math.hypot(dx, dy)
        if self.goal_distance <= self.goal_xy_tolerance:
            self.goal_reached = True
            return True
        return False

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            dt = 1.0 / self.rate
            if (rospy.Time.now() - self.last_msg_time).to_sec() > self.timeout:
                self.target = Twist()
            if self._update_goal_reached():
                self.target = Twist()

            self.current.linear.x = approach_with_limits(
                self.current.linear.x,
                self.target.linear.x,
                self.max_acc_x * dt,
                self.max_decel_x * dt,
            )
            self.current.linear.y = approach_with_limits(
                self.current.linear.y,
                self.target.linear.y,
                self.max_acc_y * dt,
                self.max_decel_y * dt,
            )
            self.current.angular.z = approach_with_limits(
                self.current.angular.z,
                self.target.angular.z,
                self.max_acc_yaw * dt,
                self.max_decel_yaw * dt,
            )

            values = [self.current.linear.x, self.current.linear.y, self.current.angular.z]
            if not all(math.isfinite(value) for value in values):
                self.current = Twist()

            self.publisher.publish(self.current)
            rospy.loginfo_throttle(
                1.0,
                "cmd_vel_filter %s -> %s: target=(%.3f %.3f %.3f) current=(%.3f %.3f %.3f) goal_dist=%s reached=%s",
                self.input_topic,
                self.output_topic,
                self.target.linear.x,
                self.target.linear.y,
                self.target.angular.z,
                self.current.linear.x,
                self.current.linear.y,
                self.current.angular.z,
                "none" if self.goal_distance is None else "%.3f" % self.goal_distance,
                self.goal_reached,
            )
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("cmd_vel_filter")
    CmdVelFilter().spin()
