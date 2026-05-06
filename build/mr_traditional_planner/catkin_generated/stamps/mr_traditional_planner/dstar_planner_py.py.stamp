#!/usr/bin/env python3
import heapq
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class SearchState:
    __slots__ = ("x", "y", "h", "k", "parent_index", "tag")

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.h = 0.0
        self.k = 0.0
        self.parent_index = -1
        self.tag = "new"


class DStarPlannerNode:
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
        self.search_states = []
        self.open_heap = []

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
            rospy.logwarn("D* Python: /map 尚未收到，无法开始规划。")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "D* Python: 仅支持 %s 坐标系目标点，当前收到的是 %s。",
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
            rospy.logwarn("D* Python: 起点超出地图范围，停止规划。")
            return

        if not self.in_bounds(goal_x, goal_y):
            rospy.logwarn("D* Python: 终点超出地图范围，停止规划。")
            return

        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("D* Python: 起点位于障碍物膨胀区内，停止规划。")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("D* Python: 终点位于障碍物膨胀区内，停止规划。")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("D* Python: 未找到可行路径。")
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
                "D* Python: 获取 %s -> %s 失败，停止规划。%s",
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
        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)
        if start_index == goal_index:
            return [start_index]

        self.search_states = []
        for linear_index in range(map_size):
            grid_x, grid_y = self.index_to_grid(linear_index)
            self.search_states.append(SearchState(grid_x, grid_y))
        self.open_heap = []

        self.insert_state(goal_index, 0.0)
        for _ in range(max(1, map_size * 16)):
            if self.search_states[start_index].tag == "closed":
                break
            if self.process_state() < 0.0:
                return []
        else:
            rospy.logwarn("D* Python: 搜索迭代次数超过上限，停止规划。")
            return []

        path_indices = []
        current_index = start_index
        for _ in range(map_size):
            path_indices.append(current_index)
            if current_index == goal_index:
                return path_indices

            parent_index = self.search_states[current_index].parent_index
            if parent_index < 0 or not math.isfinite(self.move_cost(current_index, parent_index)):
                return []
            current_index = parent_index

        return []

    def process_state(self):
        popped = self.pop_min_open()
        if popped is None:
            return -1.0

        current_index, old_key = popped
        current = self.search_states[current_index]
        if old_key < current.h:
            for neighbor_index in self.neighbors(current_index):
                neighbor = self.search_states[neighbor_index]
                cost = self.move_cost(current_index, neighbor_index)
                if math.isfinite(cost) and neighbor.h <= old_key and current.h > neighbor.h + cost:
                    current.parent_index = neighbor_index
                    current.h = neighbor.h + cost

        if self.nearly_equal(old_key, current.h):
            for neighbor_index in self.neighbors(current_index):
                neighbor = self.search_states[neighbor_index]
                cost = self.move_cost(current_index, neighbor_index)
                if not math.isfinite(cost):
                    continue
                should_insert = (
                    neighbor.tag == "new"
                    or (
                        neighbor.parent_index == current_index
                        and not self.nearly_equal(neighbor.h, current.h + cost)
                    )
                    or (neighbor.parent_index != current_index and neighbor.h > current.h + cost)
                )
                if should_insert:
                    neighbor.parent_index = current_index
                    self.insert_state(neighbor_index, current.h + cost)
        else:
            for neighbor_index in self.neighbors(current_index):
                neighbor = self.search_states[neighbor_index]
                cost = self.move_cost(current_index, neighbor_index)
                if not math.isfinite(cost):
                    continue
                if neighbor.tag == "new" or (
                    neighbor.parent_index == current_index
                    and not self.nearly_equal(neighbor.h, current.h + cost)
                ):
                    neighbor.parent_index = current_index
                    self.insert_state(neighbor_index, current.h + cost)
                elif neighbor.parent_index != current_index and neighbor.h > current.h + cost:
                    self.insert_state(current_index, current.h)
                elif (
                    neighbor.parent_index != current_index
                    and current.h > neighbor.h + cost
                    and neighbor.tag == "closed"
                    and neighbor.h > old_key
                ):
                    self.insert_state(neighbor_index, neighbor.h)

        return self.get_kmin()

    def insert_state(self, linear_index, new_h):
        state = self.search_states[linear_index]
        if state.tag == "new":
            state.k = new_h
        elif state.tag == "open":
            state.k = min(state.k, new_h)
        else:
            state.k = min(state.h, new_h)
        state.h = new_h
        state.tag = "open"
        heapq.heappush(self.open_heap, (state.k, linear_index))

    def pop_min_open(self):
        while self.open_heap:
            key, linear_index = heapq.heappop(self.open_heap)
            state = self.search_states[linear_index]
            if state.tag == "open" and self.nearly_equal(state.k, key):
                state.tag = "closed"
                return linear_index, key
        return None

    def get_kmin(self):
        while self.open_heap:
            key, linear_index = self.open_heap[0]
            state = self.search_states[linear_index]
            if state.tag == "open" and self.nearly_equal(state.k, key):
                return key
            heapq.heappop(self.open_heap)
        return -1.0

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

    @staticmethod
    def nearly_equal(lhs, rhs):
        return abs(lhs - rhs) <= 1.0e-9

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
    rospy.init_node("dstar_planner_py")
    DStarPlannerNode()
    rospy.spin()
