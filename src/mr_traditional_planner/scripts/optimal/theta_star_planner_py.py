#!/usr/bin/env python3
import heapq
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


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
        self.latest_map = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.inflation_offsets = []
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
        self.latest_map = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.build_obstacle_lookup()

    def goal_callback(self, msg):
        if self.latest_map is None:
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

        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        goal_x, goal_y = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)

        if not self.in_bounds(start_x, start_y):
            rospy.logwarn("Theta* Python: start is outside the map.")
            return
        if not self.in_bounds(goal_x, goal_y):
            rospy.logwarn("Theta* Python: goal is outside the map.")
            return

        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)
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
        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)
        start_h = self.heuristic(start_x, start_y, goal_x, goal_y)

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
                if not self.in_bounds(next_x, next_y):
                    continue

                next_index = self.to_index(next_x, next_y)
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

                heuristic_cost = self.heuristic(next_x, next_y, goal_x, goal_y)
                node_lookup[next_index] = ThetaStarNode(
                    next_x, next_y, tentative_g, heuristic_cost, parent_index
                )
                heapq.heappush(
                    open_heap,
                    (tentative_g + heuristic_cost, heuristic_cost, next_index),
                )

        return []

    def line_of_sight(self, from_index, to_x, to_y):
        x0, y0 = self.index_to_grid(from_index)
        x1 = to_x
        y1 = to_y
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        step_x = 1 if x0 < x1 else -1
        step_y = 1 if y0 < y1 else -1
        error = dx - dy

        while True:
            if self.to_index(x0, y0) in self.obstacle_set:
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
    def heuristic(x, y, goal_x, goal_y):
        return math.hypot(goal_x - x, goal_y - y)

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
    rospy.init_node("theta_star_planner_py")
    ThetaStarPlannerNode()
    rospy.spin()
