#!/usr/bin/env python3
import heapq
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class Node:
    __slots__ = ("x", "y", "g", "h", "parent_index")

    def __init__(self, x, y, g, h, parent_index):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent_index = parent_index


class DijkstraPlannerNode:
    def __init__(self):
        # 最优路径类算法统一监听静态地图输入。
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        # 最优路径类算法统一监听 RViz 2D Goal 目标点输入。
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1
        )
        # 最优路径统一输出到固定 Path 话题。
        self.path_pub = rospy.Publisher(
            "/mr_traditional_planner/optimal_path", Path, queue_size=1, latch=True
        )

        self.tf_listener = tf.TransformListener()
        self.latest_map = None
        self.latest_goal = None

        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        self.robot_radius = 0.15
        self.inflation_offsets = []
        self.obstacle_set = set()

        # 与 A* / C++ Dijkstra 严格对齐的 8 邻域扩展顺序和步进代价。
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
        self.latest_map = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.build_obstacle_lookup()

    def goal_callback(self, msg):
        self.latest_goal = msg

        if self.latest_map is None:
            rospy.logwarn("Dijkstra Python: /map 尚未收到，无法开始规划。")
            return

        if msg.header.frame_id and msg.header.frame_id != "map":
            rospy.logwarn(
                "Dijkstra Python: 仅支持 map 坐标系目标点，当前收到的是 %s。", msg.header.frame_id
            )
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        # 关键步骤：严格按公式 index_x = int((world_x - origin_x) / resolution) 做世界坐标到栅格坐标映射。
        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        # 关键步骤：终点同样使用完全一致的公式映射到 OccupancyGrid 的栅格索引。
        goal_x, goal_y = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.in_bounds(start_x, start_y):
            rospy.logwarn("Dijkstra Python: 起点超出地图范围，停止规划。")
            return

        if not self.in_bounds(goal_x, goal_y):
            rospy.logwarn("Dijkstra Python: 终点超出地图范围，停止规划。")
            return

        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)

        if start_index in self.obstacle_set:
            rospy.logwarn("Dijkstra Python: 起点位于障碍物膨胀区内，停止规划。")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("Dijkstra Python: 终点位于障碍物膨胀区内，停止规划。")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("Dijkstra Python: 未找到可行路径。")
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
            rospy.logwarn("Dijkstra Python: 获取 map -> base_footprint 失败，停止规划。%s", exc)
            return None

    def build_obstacle_lookup(self):
        self.obstacle_set.clear()
        if self.latest_map is None or self.resolution <= 0.0:
            return

        self.precompute_inflation_offsets()

        for linear_index, occupancy in enumerate(self.latest_map.data):
            # 与 C++ 版严格一致：未知栅格和占用概率 >= 50 的栅格都按障碍处理。
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
                # 与 C++ 版严格一致：用欧式距离判断该偏移是否落在机器人半径膨胀圈内。
                if math.hypot(offset_x, offset_y) * self.resolution <= self.robot_radius:
                    self.inflation_offsets.append((offset_x, offset_y))

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)

        node_lookup = {
            start_index: Node(start_x, start_y, 0.0, 0.0, -1),
        }
        closed_set = set()
        open_heap = [(0.0, start_index)]

        while open_heap:
            _, current_index = heapq.heappop(open_heap)

            if current_index in closed_set:
                continue

            current_node = node_lookup[current_index]
            if current_index == goal_index:
                return self.reconstruct_path(goal_index, node_lookup)

            closed_set.add(current_index)

            for step_x, step_y, step_cost in self.motion_model:
                next_x = current_node.x + step_x
                next_y = current_node.y + step_y

                if not self.in_bounds(next_x, next_y):
                    continue

                next_index = self.to_index(next_x, next_y)
                if next_index in self.obstacle_set or next_index in closed_set:
                    continue

                tentative_g = current_node.g + step_cost

                existing_node = node_lookup.get(next_index)
                if existing_node is not None and tentative_g >= existing_node.g:
                    continue

                node_lookup[next_index] = Node(next_x, next_y, tentative_g, 0.0, current_index)
                heapq.heappush(open_heap, (tentative_g, next_index))

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
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for linear_index in path_indices:
            grid_x = linear_index % self.map_width
            grid_y = linear_index // self.map_width

            # 关键步骤：将离散栅格索引还原为世界坐标时，取栅格中心点而不是左下角顶点。
            world_x, world_y = self.grid_to_world(grid_x, grid_y)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def world_to_grid(self, world_x, world_y):
        grid_x = int((world_x - self.origin_x) / self.resolution)
        grid_y = int((world_y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    def grid_to_world(self, grid_x, grid_y):
        world_x = self.origin_x + (grid_x + 0.5) * self.resolution
        world_y = self.origin_y + (grid_y + 0.5) * self.resolution
        return world_x, world_y

    def to_index(self, grid_x, grid_y):
        return grid_y * self.map_width + grid_x

    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height


if __name__ == "__main__":
    rospy.init_node("dijkstra_planner_py")
    DijkstraPlannerNode()
    rospy.spin()
