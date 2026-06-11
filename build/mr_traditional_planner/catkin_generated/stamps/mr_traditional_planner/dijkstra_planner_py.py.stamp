#!/usr/bin/env python3
import heapq
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils import debug_path
from utils.math_utils import GridMap, build_obstacle_set, euclidean_distance

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
        self.algorithm = "dijkstra"
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frames = debug_path.robot_frame_candidates("base_footprint")
        self.tf_timeout = max(0.1, float(rospy.get_param("~tf_timeout", 1.0)))
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        # 最优路径类算法统一监听静态地图输入。
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        # 最优路径类算法统一监听 RViz 2D Goal 目标点输入。
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        # 最优路径统一输出到固定 Path 话题。
        self.path_pub = rospy.Publisher(
            self.path_topic,
            Path,
            queue_size=1,
            latch=True,
        )
        self.publish_failure("goal_not_received")

        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.latest_goal = None

        self.robot_radius = 0.15
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
        self.latest_goal = msg
        debug_path.log_goal_received(
            self.algorithm,
            self.goal_topic,
            msg.header.frame_id or self.map_frame,
            msg.pose.position.x,
            msg.pose.position.y,
            self.grid_map is not None,
        )

        if self.grid_map is None:
            rospy.logwarn("Dijkstra Python: /map 尚未收到，无法开始规划。")
            self.publish_failure("map_not_ready")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "Dijkstra Python: 仅支持 map 坐标系目标点，当前收到的是 %s。", msg.header.frame_id
            )
            self.publish_failure("goal_frame_mismatch")
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            self.publish_failure("tf_lookup_failed")
            return

        # 关键步骤：严格按公式 index_x = int((world_x - origin_x) / resolution) 做世界坐标到栅格坐标映射。
        start_x, start_y = self.grid_map.world_to_grid(start_world[0], start_world[1])
        # 关键步骤：终点同样使用完全一致的公式映射到 OccupancyGrid 的栅格索引。
        goal_x, goal_y = self.grid_map.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.grid_map.in_bounds(start_x, start_y):
            rospy.logwarn("Dijkstra Python: 起点超出地图范围，停止规划。")
            self.publish_failure("start_out_of_bounds")
            return

        if not self.grid_map.in_bounds(goal_x, goal_y):
            rospy.logwarn("Dijkstra Python: 终点超出地图范围，停止规划。")
            self.publish_failure("goal_out_of_bounds")
            return

        start_index = self.grid_map.to_index(start_x, start_y)
        goal_index = self.grid_map.to_index(goal_x, goal_y)

        if start_index in self.obstacle_set:
            rospy.logwarn("Dijkstra Python: 起点位于障碍物膨胀区内，停止规划。")
            self.publish_failure("start_blocked")
            return

        if goal_index in self.obstacle_set:
            rospy.logwarn("Dijkstra Python: 终点位于障碍物膨胀区内，停止规划。")
            self.publish_failure("goal_blocked")
            return

        path_indices = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_indices:
            rospy.logwarn("Dijkstra Python: 未找到可行路径。")
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

                if not self.grid_map.in_bounds(next_x, next_y):
                    continue

                next_index = self.grid_map.to_index(next_x, next_y)
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
        path_msg.header.frame_id = self.map_frame

        for linear_index in path_indices:
            grid_x = linear_index % self.grid_map.width
            grid_y = linear_index // self.grid_map.width

            # 关键步骤：将离散栅格索引还原为世界坐标时，取栅格中心点而不是左下角顶点。
            world_x, world_y = self.grid_map.grid_to_world(grid_x, grid_y)

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
    rospy.init_node("dijkstra_planner_py")
    DijkstraPlannerNode()
    rospy.spin()
