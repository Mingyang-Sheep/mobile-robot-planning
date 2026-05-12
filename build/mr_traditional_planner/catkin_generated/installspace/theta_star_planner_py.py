#!/usr/bin/env python3
import heapq
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils.math_utils import GridMap, build_obstacle_set, euclidean_distance


class ThetaStarNode:
    __slots__ = ("x", "y", "g", "h", "parent_index")

    def __init__(self, x, y, g, h, parent_index):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent_index = parent_index


class ThetaStarPlannerNode:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/optimal_path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)

        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.obstacle_set = set()

        diagonal_cost = math.sqrt(2.0)
        self.motion_model = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
            (1, 1, diagonal_cost),
            (-1, 1, diagonal_cost),
            (-1, -1, diagonal_cost),
            (1, -1, diagonal_cost),
        ]

    def map_callback(self, msg):
        self.grid_map = GridMap(msg)
        self.obstacle_set = build_obstacle_set(self.grid_map, self.robot_radius)

    def goal_callback(self, msg):
        if self.grid_map is None:
            rospy.logwarn("Theta* Python: /map has not been received yet.")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "Theta* Python: goal frame must be %s, got %s.",
                self.map_frame,
                msg.header.frame_id,
            )
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        start_x, start_y = self.grid_map.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.grid_map.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.grid_map.in_bounds(start_x, start_y):
            rospy.logwarn("Theta* Python: start is outside the map.")
            return
        if not self.grid_map.in_bounds(goal_x, goal_y):
            rospy.logwarn("Theta* Python: goal is outside the map.")
            return

        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("Theta* Python: start is inside an inflated obstacle.")
            return
        if goal_index in self.obstacle_set:
            rospy.logwarn("Theta* Python: goal is inside an inflated obstacle.")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("Theta* Python: no path found.")
            return

        self.publish_path(path_indices)

    def lookup_start_pose(self):
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.2)
            )
            translation, _ = self.tf_listener.lookupTransform(
                self.map_frame, self.robot_frame, rospy.Time(0)
            )
            return translation[0], translation[1]
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn(
                "Theta* Python: failed to lookup %s -> %s: %s",
                self.map_frame,
                self.robot_frame,
                exc,
            )
            return None

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)
        start_h = euclidean_distance(start_x, start_y, goal_x, goal_y)

        node_lookup = {start_index: ThetaStarNode(start_x, start_y, 0.0, start_h, -1)}
        closed_set = set()
        open_heap = [(start_h, start_h, start_index)]

        while open_heap:
            _, _, current_index = heapq.heappop(open_heap)
            if current_index in closed_set:
                continue

            current_node = node_lookup[current_index]
            if current_index == goal_index:
                return self.reconstruct_path(goal_index, node_lookup)

            closed_set.add(current_index)

            for step_x, step_y, step_cost in self.motion_model:
                next_x = current_node.x + step_x
                next_y = current_node.y + step_y
                if not self.grid_map.in_bounds(next_x, next_y):
                    continue

                next_index = self.grid_map.to_index(next_x, next_y)
                if next_index in self.obstacle_set or next_index in closed_set:
                    continue

                tentative_g = current_node.g + step_cost
                parent_index = current_index
                if current_node.parent_index >= 0 and self.line_of_sight(
                    current_node.parent_index, next_x, next_y
                ):
                    parent_node = node_lookup[current_node.parent_index]
                    tentative_g = parent_node.g + math.hypot(
                        next_x - parent_node.x, next_y - parent_node.y
                    )
                    parent_index = current_node.parent_index

                existing_node = node_lookup.get(next_index)
                if existing_node is not None and tentative_g >= existing_node.g:
                    continue

                heuristic_cost = euclidean_distance(next_x, next_y, goal_x, goal_y)
                node_lookup[next_index] = ThetaStarNode(
                    next_x, next_y, tentative_g, heuristic_cost, parent_index
                )
                heapq.heappush(
                    open_heap,
                    (tentative_g + heuristic_cost, heuristic_cost, next_index),
                )

        return []

    def line_of_sight(self, from_index, to_x, to_y):
        x0, y0 = self.grid_map.index_to_grid(from_index)
        x1 = to_x
        y1 = to_y
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx - dy

        while True:
            if self.grid_map.to_index(x0, y0) in self.obstacle_set:
                return False
            if x0 == x1 and y0 == y1:
                return True

            twice_error = 2 * error
            if twice_error > -dy:
                error -= dy
                x0 += step_x
            if twice_error < dx:
                error += dx
                y0 += step_y

    def reconstruct_path(self, goal_index, node_lookup):
        path_indices = []
        current_index = goal_index
        while current_index != -1:
            path_indices.append(current_index)
            current_index = node_lookup[current_index].parent_index

        path_indices.reverse()
        return path_indices

    def publish_path(self, path_indices):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame

        for linear_index in path_indices:
            grid_x, grid_y = self.grid_map.index_to_grid(linear_index)
            world_x, world_y = self.grid_map.grid_to_world(grid_x, grid_y)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


if __name__ == "__main__":
    rospy.init_node("theta_star_planner_py")
    ThetaStarPlannerNode()
    rospy.spin()
