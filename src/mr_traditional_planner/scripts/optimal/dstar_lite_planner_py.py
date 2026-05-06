#!/usr/bin/env python3
import heapq
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DStarLitePlannerNode:
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
        self.path_pub = rospy.Publisher(
            self.path_topic, Path, queue_size=1, latch=True
        )

        self.tf_listener = tf.TransformListener()
        self.latest_map = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.inflation_offsets = []
        self.obstacle_set = set()
        self.g_values = []
        self.rhs_values = []
        self.open_heap = []
        self.search_start_index = -1
        self.search_goal_index = -1
        self.km = 0.0

    def map_callback(self, msg):
        self.latest_map = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.build_obstacle_lookup()

    def goal_callback(self, msg):
        if self.latest_map is None:
            rospy.logwarn("D* Lite Python: /map 尚未收到，无法开始规划。")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "D* Lite Python: 仅支持 %s 坐标系目标点，当前收到的是 %s。",
                self.map_frame,
                msg.header.frame_id,
            )
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.in_bounds(start_x, start_y):
            rospy.logwarn("D* Lite Python: 起点超出地图范围，停止规划。")
            return

        if not self.in_bounds(goal_x, goal_y):
            rospy.logwarn("D* Lite Python: 终点超出地图范围，停止规划。")
            return

        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("D* Lite Python: 起点位于障碍物膨胀区内，停止规划。")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("D* Lite Python: 终点位于障碍物膨胀区内，停止规划。")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("D* Lite Python: 未找到可行路径。")
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
                "D* Lite Python: 获取 %s -> %s 失败，停止规划。%s",
                self.map_frame,
                self.robot_frame,
                exc,
            )
            return None

    def build_obstacle_lookup(self):
        self.obstacle_set.clear()
        if self.latest_map is None or self.resolution <= 0.0:
            return

        self.precompute_inflation_offsets()
        for linear_index, occupancy in enumerate(self.latest_map.data):
            if occupancy < 0 or occupancy >= 50:
                obstacle_x = linear_index % self.map_width
                obstacle_y = linear_index // self.map_width
                for offset_x, offset_y in self.inflation_offsets:
                    inflated_x = obstacle_x + offset_x
                    inflated_y = obstacle_y + offset_y
                    if self.in_bounds(inflated_x, inflated_y):
                        self.obstacle_set.add(self.to_index(inflated_x, inflated_y))

    def precompute_inflation_offsets(self):
        self.inflation_offsets = []
        inflation_radius_in_cells = int(math.ceil(self.robot_radius / self.resolution))
        for offset_y in range(-inflation_radius_in_cells, inflation_radius_in_cells + 1):
            for offset_x in range(-inflation_radius_in_cells, inflation_radius_in_cells + 1):
                if math.hypot(offset_x, offset_y) * self.resolution <= self.robot_radius:
                    self.inflation_offsets.append((offset_x, offset_y))

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        map_size = self.map_width * self.map_height
        self.search_start_index = self.to_index(start_x, start_y)
        self.search_goal_index = self.to_index(goal_x, goal_y)
        if self.search_start_index == self.search_goal_index:
            return [self.search_start_index]

        self.g_values = [float("inf")] * map_size
        self.rhs_values = [float("inf")] * map_size
        self.open_heap = []
        self.km = 0.0

        self.rhs_values[self.search_goal_index] = 0.0
        heapq.heappush(
            self.open_heap,
            (*self.calculate_key(self.search_goal_index), self.search_goal_index),
        )

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
        map_size = self.map_width * self.map_height
        for _ in range(max(1, map_size * 32)):
            if not self.open_heap and self.nearly_equal(
                self.rhs_values[self.search_start_index], self.g_values[self.search_start_index]
            ):
                return True

            if not self.open_heap:
                return False

            key_first, key_second, current_index = self.open_heap[0]
            start_key = self.calculate_key(self.search_start_index)
            if not self.compare_keys((key_first, key_second), start_key) and self.nearly_equal(
                self.rhs_values[self.search_start_index], self.g_values[self.search_start_index]
            ):
                return True

            heapq.heappop(self.open_heap)
            current_key = self.calculate_key(current_index)
            item_key = (key_first, key_second)
            if self.compare_keys(current_key, item_key):
                continue

            if self.compare_keys(item_key, current_key):
                heapq.heappush(self.open_heap, (*current_key, current_index))
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

    def update_vertex(self, linear_index):
        if linear_index != self.search_goal_index:
            best_rhs = float("inf")
            for successor_index in self.neighbors(linear_index):
                best_rhs = min(
                    best_rhs,
                    self.move_cost(linear_index, successor_index) + self.g_values[successor_index],
                )
            self.rhs_values[linear_index] = best_rhs

        if not self.nearly_equal(self.g_values[linear_index], self.rhs_values[linear_index]):
            heapq.heappush(self.open_heap, (*self.calculate_key(linear_index), linear_index))

    def calculate_key(self, linear_index):
        min_value = min(self.g_values[linear_index], self.rhs_values[linear_index])
        return min_value + self.heuristic(self.search_start_index, linear_index) + self.km, min_value

    @staticmethod
    def compare_keys(lhs, rhs):
        return lhs[0] < rhs[0] or (abs(lhs[0] - rhs[0]) <= 1.0e-9 and lhs[1] < rhs[1])

    @staticmethod
    def nearly_equal(lhs, rhs):
        return abs(lhs - rhs) <= 1.0e-9

    def neighbors(self, linear_index):
        grid_x, grid_y = self.index_to_grid(linear_index)
        result = []
        for offset_y in (-1, 0, 1):
            for offset_x in (-1, 0, 1):
                if offset_x == 0 and offset_y == 0:
                    continue
                next_x = grid_x + offset_x
                next_y = grid_y + offset_y
                if self.in_bounds(next_x, next_y):
                    result.append(self.to_index(next_x, next_y))
        return result

    def move_cost(self, from_index, to_index):
        from_x, from_y = self.index_to_grid(from_index)
        to_x, to_y = self.index_to_grid(to_index)
        if from_index in self.obstacle_set or to_index in self.obstacle_set:
            return float("inf")
        return math.hypot(from_x - to_x, from_y - to_y)

    def heuristic(self, from_index, to_index):
        from_x, from_y = self.index_to_grid(from_index)
        to_x, to_y = self.index_to_grid(to_index)
        dx = abs(from_x - to_x)
        dy = abs(from_y - to_y)
        diagonal = min(dx, dy)
        straight = max(dx, dy) - diagonal
        return math.sqrt(2.0) * diagonal + straight

    def publish_path(self, path_indices):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame
        for linear_index in path_indices:
            grid_x, grid_y = self.index_to_grid(linear_index)
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def world_to_grid(self, world_x, world_y):
        return int((world_x - self.origin_x) / self.resolution), int(
            (world_y - self.origin_y) / self.resolution
        )

    def grid_to_world(self, grid_x, grid_y):
        return (
            self.origin_x + (grid_x + 0.5) * self.resolution,
            self.origin_y + (grid_y + 0.5) * self.resolution,
        )

    def to_index(self, grid_x, grid_y):
        return grid_y * self.map_width + grid_x

    def index_to_grid(self, linear_index):
        return linear_index % self.map_width, linear_index // self.map_width

    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height


if __name__ == "__main__":
    rospy.init_node("dstar_lite_planner_py")
    DStarLitePlannerNode()
    rospy.spin()
