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
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1
        )
        self.path_pub = rospy.Publisher(
            "/mr_traditional_planner/optimal_path", Path, queue_size=1, latch=True
        )

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

    def goal_callback(self, msg):
        if self.grid_map is None:
            rospy.logwarn("A* Python: /map 尚未收到，无法开始规划。")
            return

        if msg.header.frame_id and msg.header.frame_id != "map":
            rospy.logwarn("A* Python: 仅支持 map 坐标系目标点，当前收到的是 %s。", msg.header.frame_id)
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        start_x, start_y = self.grid_map.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.grid_map.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.grid_map.in_bounds(start_x, start_y):
            rospy.logwarn("A* Python: 起点超出地图范围，停止规划。")
            return

        if not self.grid_map.in_bounds(goal_x, goal_y):
            rospy.logwarn("A* Python: 终点超出地图范围，停止规划。")
            return

        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)

        if start_index in self.obstacle_set:
            rospy.logwarn("A* Python: 起点位于障碍物膨胀区内，停止规划。")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("A* Python: 终点位于障碍物膨胀区内，停止规划。")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("A* Python: 未找到可行路径。")
            return

        self.publish_path(path_indices)

    def lookup_start_pose(self):
        try:
            self.tf_listener.waitForTransform(
                "map", "base_footprint", rospy.Time(0), rospy.Duration(0.2)
            )
            translation, _ = self.tf_listener.lookupTransform(
                "map", "base_footprint", rospy.Time(0)
            )
            return translation[0], translation[1]
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn("A* Python: 获取 map -> base_footprint 失败，停止规划。%s", exc)
            return None

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
        path_msg.header.frame_id = "map"

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


if __name__ == "__main__":
    rospy.init_node("astar_planner_py")
    AStarPlannerNode()
    rospy.spin()
