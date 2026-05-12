#!/usr/bin/env python3
import math
import os
import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils.math_utils import normalize_angle, GridMap, build_obstacle_set


class DwaState:
    __slots__ = ("x", "y", "yaw", "velocity", "yaw_rate")

    def __init__(self, x, y, yaw, velocity, yaw_rate):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity
        self.yaw_rate = yaw_rate


class DynamicWindowApproachPlannerNode:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/optimal_path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)

        self.max_speed = max(0.0, float(rospy.get_param("~max_speed", 0.22)))
        self.min_speed = min(float(rospy.get_param("~min_speed", 0.0)), self.max_speed)
        self.max_yaw_rate = max(1.0e-3, float(rospy.get_param("~max_yaw_rate", 1.82)))
        self.max_accel = max(1.0e-3, float(rospy.get_param("~max_accel", 0.2)))
        self.max_delta_yaw_rate = max(
            1.0e-3, float(rospy.get_param("~max_delta_yaw_rate", 2.0))
        )
        self.velocity_resolution = max(
            1.0e-3, float(rospy.get_param("~velocity_resolution", 0.02))
        )
        self.yaw_rate_resolution = max(
            1.0e-3, float(rospy.get_param("~yaw_rate_resolution", 0.1))
        )
        self.dt = max(1.0e-3, float(rospy.get_param("~dt", 0.1)))
        self.predict_time = max(self.dt, float(rospy.get_param("~predict_time", 2.0)))
        self.to_goal_cost_gain = float(rospy.get_param("~to_goal_cost_gain", 0.15))
        self.speed_cost_gain = float(rospy.get_param("~speed_cost_gain", 1.0))
        self.obstacle_cost_gain = float(rospy.get_param("~obstacle_cost_gain", 1.0))
        self.goal_tolerance = max(1.0e-3, float(rospy.get_param("~goal_tolerance", 0.2)))
        self.robot_stuck_flag_cons = 0.001
        control_frequency = max(1.0, float(rospy.get_param("~control_frequency", 10.0)))

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / control_frequency), self.control_timer_callback
        )

        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.latest_odom = None
        self.latest_goal = None
        self.goal_active = False
        self.obstacle_set = set()
        self.obstacle_points = []

    def map_callback(self, msg):
        self.grid_map = GridMap(msg)
        self.build_obstacle_lookup()

    def odom_callback(self, msg):
        self.latest_odom = msg

    def goal_callback(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "DWA Python: goal frame must be %s, got %s.",
                self.map_frame,
                msg.header.frame_id,
            )
            return

        self.latest_goal = msg
        self.goal_active = True

    def control_timer_callback(self, _):
        if not self.goal_active:
            return

        if self.grid_map is None:
            rospy.logwarn_throttle(1.0, "DWA Python: /map has not been received yet.")
            self.publish_stop()
            return

        if self.latest_odom is None:
            rospy.logwarn_throttle(1.0, "DWA Python: /odom has not been received yet.")
            self.publish_stop()
            return

        if self.latest_goal is None:
            self.publish_stop()
            return

        state = self.lookup_robot_state()
        if state is None:
            self.publish_stop()
            return

        goal_x = self.latest_goal.pose.position.x
        goal_y = self.latest_goal.pose.position.y
        if math.hypot(state.x - goal_x, state.y - goal_y) <= self.goal_tolerance:
            self.goal_active = False
            self.publish_stop()
            self.publish_trajectory([state])
            rospy.loginfo("DWA Python: goal reached.")
            return

        control, trajectory = self.select_control(state)
        if control is None:
            rospy.logwarn_throttle(1.0, "DWA Python: failed to find a collision-free control.")
            self.publish_stop()
            return

        self.publish_command(control[0], control[1])
        self.publish_trajectory(trajectory)

    def lookup_robot_state(self):
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.05)
            )
            translation, rotation = self.tf_listener.lookupTransform(
                self.map_frame, self.robot_frame, rospy.Time(0)
            )
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return DwaState(
                translation[0],
                translation[1],
                yaw,
                self.latest_odom.twist.twist.linear.x,
                self.latest_odom.twist.twist.angular.z,
            )
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn_throttle(
                1.0,
                "DWA Python: failed to lookup %s -> %s: %s",
                self.map_frame,
                self.robot_frame,
                exc,
            )
            return None

    def build_obstacle_lookup(self):
        self.obstacle_set = set()
        self.obstacle_points = []
        if self.grid_map is None or self.grid_map.resolution <= 0.0:
            return

        self.obstacle_set = build_obstacle_set(self.grid_map, self.robot_radius)

        gm = self.grid_map
        for linear_index in self.obstacle_set:
            grid_x = linear_index % gm.width
            grid_y = linear_index // gm.width
            self.obstacle_points.append(
                (
                    gm.origin_x + (grid_x + 0.5) * gm.resolution,
                    gm.origin_y + (grid_y + 0.5) * gm.resolution,
                )
            )

    def calc_dynamic_window(self, state):
        return (
            max(self.min_speed, state.velocity - self.max_accel * self.dt),
            min(self.max_speed, state.velocity + self.max_accel * self.dt),
            max(-self.max_yaw_rate, state.yaw_rate - self.max_delta_yaw_rate * self.dt),
            min(self.max_yaw_rate, state.yaw_rate + self.max_delta_yaw_rate * self.dt),
        )

    def select_control(self, state):
        velocity_min, velocity_max, yaw_rate_min, yaw_rate_max = self.calc_dynamic_window(state)
        if velocity_min > velocity_max or yaw_rate_min > yaw_rate_max:
            return None, []

        best_cost = float("inf")
        best_control = None
        best_trajectory = []
        velocity = velocity_min
        while velocity <= velocity_max + 1.0e-9:
            yaw_rate = yaw_rate_min
            while yaw_rate <= yaw_rate_max + 1.0e-9:
                trajectory = self.predict_trajectory(state, velocity, yaw_rate)
                obstacle_cost = self.calc_obstacle_cost(trajectory)
                if math.isfinite(obstacle_cost):
                    final_cost = (
                        self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory)
                        + self.speed_cost_gain * (self.max_speed - trajectory[-1].velocity)
                        + self.obstacle_cost_gain * obstacle_cost
                    )
                    if final_cost < best_cost:
                        best_cost = final_cost
                        best_control = [velocity, yaw_rate]
                        best_trajectory = trajectory
                yaw_rate += self.yaw_rate_resolution
            velocity += self.velocity_resolution

        if best_control is None:
            return None, []

        if (
            abs(best_control[0]) < self.robot_stuck_flag_cons
            and abs(state.velocity) < self.robot_stuck_flag_cons
        ):
            best_control[1] = -self.max_delta_yaw_rate
        return best_control, best_trajectory

    def predict_trajectory(self, state, velocity, yaw_rate):
        trajectory = [state]
        current = state
        time = 0.0
        while time <= self.predict_time:
            current = self.motion(current, velocity, yaw_rate, self.dt)
            trajectory.append(current)
            time += self.dt
        return trajectory

    def motion(self, state, velocity, yaw_rate, dt):
        next_yaw = normalize_angle(state.yaw + yaw_rate * dt)
        return DwaState(
            state.x + velocity * math.cos(next_yaw) * dt,
            state.y + velocity * math.sin(next_yaw) * dt,
            next_yaw,
            velocity,
            yaw_rate,
        )

    def calc_to_goal_cost(self, trajectory):
        final_state = trajectory[-1]
        dx = self.latest_goal.pose.position.x - final_state.x
        dy = self.latest_goal.pose.position.y - final_state.y
        goal_angle = math.atan2(dy, dx)
        return abs(normalize_angle(goal_angle - final_state.yaw))

    def calc_obstacle_cost(self, trajectory):
        min_distance = float("inf")
        for state in trajectory:
            if not self.is_world_point_free(state.x, state.y):
                return float("inf")

            for obstacle_x, obstacle_y in self.obstacle_points:
                min_distance = min(min_distance, math.hypot(state.x - obstacle_x, state.y - obstacle_y))

        if not math.isfinite(min_distance):
            return 0.0
        return 1.0 / max(min_distance, 1.0e-6)

    def publish_command(self, velocity, yaw_rate):
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity
        cmd_vel.angular.z = yaw_rate
        self.cmd_vel_pub.publish(cmd_vel)

    def publish_stop(self):
        self.publish_command(0.0, 0.0)

    def publish_trajectory(self, trajectory):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame

        for state in trajectory:
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, state.yaw)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = state.x
            pose.pose.position.y = state.y
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def is_world_point_free(self, world_x, world_y):
        grid_x, grid_y = self.grid_map.world_to_grid(world_x, world_y)
        return self.grid_map.in_bounds(grid_x, grid_y) and self.grid_map.to_index(grid_x, grid_y) not in self.obstacle_set


if __name__ == "__main__":
    rospy.init_node("dwa_planner_py")
    DynamicWindowApproachPlannerNode()
    rospy.spin()
