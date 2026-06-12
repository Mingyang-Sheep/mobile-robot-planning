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
from utils import debug_path
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
        self.algorithm = "theta_star"
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_frames = debug_path.robot_frame_candidates(self.robot_frame)
        self.tf_timeout = max(0.1, float(rospy.get_param("~tf_timeout", 1.0)))
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)

        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)
        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.obstacle_set = set()

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        debug_path.log_subscriptions_ready(self)
        self.publish_failure("goal_not_received")

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

    @debug_path.traced_callback("map_callback")
    def map_callback(self, msg):
        grid_map = GridMap(msg)
        obstacle_set = build_obstacle_set(grid_map, self.robot_radius)
        self.grid_map = grid_map
        self.obstacle_set = obstacle_set
        debug_path.log_map_ready(
            self.algorithm,
            self.map_topic,
            self.grid_map.width,
            self.grid_map.height,
            self.grid_map.resolution,
            msg.header.frame_id,
        )

    @debug_path.traced_callback("goal_callback")
    def goal_callback(self, msg):
        debug_path.log_goal_received(
            self.algorithm,
            self.goal_topic,
            msg.header.frame_id or self.map_frame,
            msg.pose.position.x,
            msg.pose.position.y,
            self.grid_map is not None,
        )
        if self.grid_map is None:
            rospy.logwarn("Theta* Python: /map has not been received yet.")
            self.publish_failure("map_not_ready")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "Theta* Python: goal frame must be %s, got %s.",
                self.map_frame,
                msg.header.frame_id,
            )
            self.publish_failure("goal_frame_mismatch")
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            self.publish_failure("tf_lookup_failed")
            return

        start_x, start_y = self.grid_map.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.grid_map.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        debug_path.log_grid_points(self.algorithm, start_x, start_y, goal_x, goal_y)

        if not self.grid_map.in_bounds(start_x, start_y):
            rospy.logwarn("Theta* Python: start is outside the map.")
            self.publish_failure("start_out_of_bounds")
            return
        if not self.grid_map.in_bounds(goal_x, goal_y):
            rospy.logwarn("Theta* Python: goal is outside the map.")
            self.publish_failure("goal_out_of_bounds")
            return

        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("Theta* Python: start is inside an inflated obstacle.")
            self.publish_failure("start_blocked")
            return
        if goal_index in self.obstacle_set:
            rospy.logwarn("Theta* Python: goal is inside an inflated obstacle.")
            self.publish_failure("goal_blocked")
            return

        debug_path.log_plan_call(self.algorithm)
        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        debug_path.log_plan_return(self.algorithm, len(path_indices))
        if not path_indices:
            rospy.logwarn("Theta* Python: no path found.")
            self.publish_failure("no_path")
            return

        self.publish_path(path_indices)

    def lookup_start_pose(self):
        result = debug_path.lookup_start_pose(
            self.tf_listener, self.map_frame, self.robot_frames, self.tf_timeout, self.algorithm
        )
        if result is None:
            return None
        return result[0], result[1]

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
        debug_path.publish_grid_path(
            self.path_pub,
            self.grid_map,
            path_indices,
            self.map_frame,
            self.algorithm,
            self.path_topic,
        )

    def publish_failure(self, reason):
        debug_path.publish_empty(
            self.path_pub, self.map_frame, self.algorithm, self.path_topic, reason
        )


if __name__ == "__main__":
    rospy.init_node("theta_star_planner_py")
    ThetaStarPlannerNode()
    rospy.spin()
