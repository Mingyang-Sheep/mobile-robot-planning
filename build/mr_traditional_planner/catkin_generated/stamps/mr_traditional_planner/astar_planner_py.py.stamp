#!/usr/bin/env python3
import heapq
import math
import os
import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils import debug_path
from utils.math_utils import GridMap, build_obstacle_set, euclidean_distance


class Node:
    __slots__ = ("x", "y", "g", "h", "parent_index")

    def __init__(self, x, y, g, h, parent_index):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent_index = parent_index


class AStarPlannerNode:
    def __init__(self):
        self.algorithm = "astar"
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frames = debug_path.robot_frame_candidates("base_footprint")
        self.tf_timeout = max(0.1, float(rospy.get_param("~tf_timeout", 1.0)))
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        self.path_pub = rospy.Publisher(
            self.path_topic,
            Path,
            queue_size=1,
            latch=True,
        )
        self.publish_failure("goal_not_received")

        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.obstacle_set = set()

        self.robot_radius = 0.15

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
        debug_path.log_map_ready(
            self.algorithm,
            self.map_topic,
            self.grid_map.width,
            self.grid_map.height,
            self.grid_map.resolution,
            msg.header.frame_id,
        )

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
            rospy.logwarn("A* Python: /map 尚未收到，无法开始规划。")
            self.publish_failure("map_not_ready")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn("A* Python: 仅支持 map 坐标系目标点，当前收到的是 %s。", msg.header.frame_id)
            self.publish_failure("goal_frame_mismatch")
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            self.publish_failure("tf_lookup_failed")
            return

        start_x, start_y = self.grid_map.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.grid_map.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.grid_map.in_bounds(start_x, start_y):
            rospy.logwarn("A* Python: 起点超出地图范围，停止规划。")
            self.publish_failure("start_out_of_bounds")
            return

        if not self.grid_map.in_bounds(goal_x, goal_y):
            rospy.logwarn("A* Python: 终点超出地图范围，停止规划。")
            self.publish_failure("goal_out_of_bounds")
            return

        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)

        if start_index in self.obstacle_set:
            rospy.logwarn("A* Python: 起点位于障碍物膨胀区内，停止规划。")
            self.publish_failure("start_blocked")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("A* Python: 终点位于障碍物膨胀区内，停止规划。")
            self.publish_failure("goal_blocked")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("A* Python: 未找到可行路径。")
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
        gm = self.grid_map
        start_index = gm.to_index(start_x, start_y)
        goal_index = gm.to_index(goal_x, goal_y)

        start_h = euclidean_distance(start_x, start_y, goal_x, goal_y)
        node_lookup = {
            start_index: Node(start_x, start_y, 0.0, start_h, -1),
        }
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

                if not gm.in_bounds(next_x, next_y):
                    continue

                next_index = gm.to_index(next_x, next_y)
                if next_index in self.obstacle_set or next_index in closed_set:
                    continue

                tentative_g = current_node.g + step_cost
                heuristic_cost = euclidean_distance(next_x, next_y, goal_x, goal_y)

                existing_node = node_lookup.get(next_index)
                if existing_node is not None and tentative_g >= existing_node.g:
                    continue

                node_lookup[next_index] = Node(
                    next_x, next_y, tentative_g, heuristic_cost, current_index
                )
                heapq.heappush(
                    open_heap,
                    (tentative_g + heuristic_cost, heuristic_cost, next_index),
                )

        return []

    def reconstruct_path(self, goal_index, node_lookup):
        path_indices = []
        current_index = goal_index

        while current_index != -1:
            path_indices.append(current_index)
            current_index = node_lookup[current_index].parent_index

        path_indices.reverse()
        return path_indices

    def publish_path(self, path_indices):
        gm = self.grid_map
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame

        for linear_index in path_indices:
            grid_x, grid_y = gm.index_to_grid(linear_index)
            world_x, world_y = gm.grid_to_world(grid_x, grid_y)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        debug_path.log_success(self.algorithm, self.path_topic, len(path_msg.poses))

    def publish_failure(self, reason):
        debug_path.publish_empty(
            self.path_pub, self.map_frame, self.algorithm, self.path_topic, reason
        )


if __name__ == "__main__":
    rospy.init_node("astar_planner_py")
    AStarPlannerNode()
    rospy.spin()
