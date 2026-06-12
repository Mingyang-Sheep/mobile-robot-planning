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


class DStarLitePlannerNode:
    def __init__(self):
        self.algorithm = "dstar_lite"
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_frames = debug_path.robot_frame_candidates(self.robot_frame)
        self.tf_timeout = max(0.1, float(rospy.get_param("~tf_timeout", 1.0)))
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)

        self.path_pub = rospy.Publisher(
            self.path_topic, Path, queue_size=1, latch=True
        )
        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.obstacle_set = set()
        self.g_values = []
        self.rhs_values = []
        self.open_heap = []
        self.open_lookup = {}
        self.search_start_index = -1
        self.search_goal_index = -1
        self.km = 0.0

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        debug_path.log_subscriptions_ready(self)
        self.publish_failure("goal_not_received")

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
            rospy.logwarn("D* Lite Python: /map 尚未收到，无法开始规划。")
            self.publish_failure("map_not_ready")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "D* Lite Python: 仅支持 %s 坐标系目标点，当前收到的是 %s。",
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
            rospy.logwarn("D* Lite Python: 起点超出地图范围，停止规划。")
            self.publish_failure("start_out_of_bounds")
            return

        if not self.grid_map.in_bounds(goal_x, goal_y):
            rospy.logwarn("D* Lite Python: 终点超出地图范围，停止规划。")
            self.publish_failure("goal_out_of_bounds")
            return

        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("D* Lite Python: 起点位于障碍物膨胀区内，停止规划。")
            self.publish_failure("start_blocked")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("D* Lite Python: 终点位于障碍物膨胀区内，停止规划。")
            self.publish_failure("goal_blocked")
            return

        debug_path.log_plan_call(self.algorithm)
        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        debug_path.log_plan_return(self.algorithm, len(path_indices))
        if not path_indices:
            rospy.logwarn("D* Lite Python: 未找到可行路径。")
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
        map_size = self.grid_map.width * self.grid_map.height
        self.search_start_index = self.grid_map.to_index(start_x, start_y)
        self.search_goal_index = self.grid_map.to_index(goal_x, goal_y)
        if self.search_start_index == self.search_goal_index:
            return [self.search_start_index]

        self.g_values = [float("inf")] * map_size
        self.rhs_values = [float("inf")] * map_size
        self.open_heap = []
        self.open_lookup = {}
        self.km = 0.0

        self.rhs_values[self.search_goal_index] = 0.0
        self.push_open(self.search_goal_index)

        if not self.compute_shortest_path():
            return []

        path_indices = []
        current_index = self.search_start_index
        for _ in range(map_size):
            path_indices.append(current_index)
            if current_index == self.search_goal_index:
                return path_indices

            best_cost = float("inf")
            best_index = -1
            for neighbor_index in self.neighbors(current_index):
                candidate_cost = self.move_cost(current_index, neighbor_index) + self.g_values[
                    neighbor_index
                ]
                if candidate_cost < best_cost:
                    best_cost = candidate_cost
                    best_index = neighbor_index

            if best_index < 0 or not math.isfinite(best_cost):
                return []
            current_index = best_index

        return []

    def compute_shortest_path(self):
        map_size = self.grid_map.width * self.grid_map.height
        for _ in range(max(1, map_size * 32)):
            top_key = self.top_key()
            start_key = self.calculate_key(self.search_start_index)
            if not self.compare_keys(top_key, start_key) and self.nearly_equal(
                self.rhs_values[self.search_start_index], self.g_values[self.search_start_index]
            ):
                return True

            popped = self.pop_open()
            if popped is None:
                return False

            current_index, item_key = popped
            current_key = self.calculate_key(current_index)

            if self.compare_keys(item_key, current_key):
                self.push_open(current_index)
                continue

            if self.g_values[current_index] > self.rhs_values[current_index]:
                self.g_values[current_index] = self.rhs_values[current_index]
                for predecessor_index in self.neighbors(current_index):
                    self.update_vertex(predecessor_index)
            else:
                self.g_values[current_index] = float("inf")
                self.update_vertex(current_index)
                for predecessor_index in self.neighbors(current_index):
                    self.update_vertex(predecessor_index)

        rospy.logwarn("D* Lite Python: 搜索迭代次数超过上限，停止规划。")
        return False

    def push_open(self, linear_index):
        key = self.calculate_key(linear_index)
        self.open_lookup[linear_index] = key
        heapq.heappush(self.open_heap, (*key, linear_index))

    def remove_open(self, linear_index):
        self.open_lookup.pop(linear_index, None)

    def top_key(self):
        while self.open_heap:
            key_first, key_second, linear_index = self.open_heap[0]
            key = (key_first, key_second)
            if self.open_lookup.get(linear_index) == key:
                return key
            heapq.heappop(self.open_heap)
        return float("inf"), float("inf")

    def pop_open(self):
        while self.open_heap:
            key_first, key_second, linear_index = heapq.heappop(self.open_heap)
            key = (key_first, key_second)
            if self.open_lookup.get(linear_index) == key:
                self.remove_open(linear_index)
                return linear_index, key
        return None

    def update_vertex(self, linear_index):
        if linear_index != self.search_goal_index:
            best_rhs = float("inf")
            for successor_index in self.neighbors(linear_index):
                best_rhs = min(
                    best_rhs,
                    self.move_cost(linear_index, successor_index) + self.g_values[successor_index],
                )
            self.rhs_values[linear_index] = best_rhs

        self.remove_open(linear_index)
        if not self.nearly_equal(self.g_values[linear_index], self.rhs_values[linear_index]):
            self.push_open(linear_index)

    def calculate_key(self, linear_index):
        min_value = min(self.g_values[linear_index], self.rhs_values[linear_index])
        sx, sy = self.grid_map.index_to_grid(self.search_start_index)
        gx, gy = self.grid_map.index_to_grid(linear_index)
        return min_value + euclidean_distance(sx, sy, gx, gy) + self.km, min_value

    @staticmethod
    def compare_keys(lhs, rhs):
        return lhs[0] < rhs[0] or (abs(lhs[0] - rhs[0]) <= 1.0e-9 and lhs[1] < rhs[1])

    @staticmethod
    def nearly_equal(lhs, rhs):
        return abs(lhs - rhs) <= 1.0e-9

    def neighbors(self, linear_index):
        grid_x, grid_y = self.grid_map.index_to_grid(linear_index)
        result = []
        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue
                next_x = grid_x + offset_x
                next_y = grid_y + offset_y
                if self.grid_map.in_bounds(next_x, next_y):
                    result.append(self.grid_map.to_index(next_x, next_y))
        return result

    def move_cost(self, from_index, to_index):
        from_x, from_y = self.grid_map.index_to_grid(from_index)
        to_x, to_y = self.grid_map.index_to_grid(to_index)
        if from_index in self.obstacle_set or to_index in self.obstacle_set:
            return float("inf")
        return math.hypot(from_x - to_x, from_y - to_y)

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
    rospy.init_node("dstar_lite_planner_py")
    DStarLitePlannerNode()
    rospy.spin()
