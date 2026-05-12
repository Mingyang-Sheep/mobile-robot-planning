#!/usr/bin/env python3
import math
import os
import random
import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils.math_utils import GridMap, build_obstacle_set, euclidean_distance


class RRTStarNode:
    __slots__ = ("x", "y", "cost", "parent_index")

    def __init__(self, x, y, cost=0.0, parent_index=-1):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index


class RRTStarPlannerNode:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/optimal_path")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)
        self.max_iterations = max(1, int(rospy.get_param("~max_iterations", 1000)))
        self.expand_distance = max(1.0e-3, float(rospy.get_param("~expand_distance", 0.5)))
        self.path_resolution = max(1.0e-3, float(rospy.get_param("~path_resolution", 0.05)))
        self.goal_sample_rate = min(100, max(0, int(rospy.get_param("~goal_sample_rate", 20))))
        self.connect_circle_distance = max(
            1.0e-3, float(rospy.get_param("~connect_circle_distance", 2.0))
        )
        self.search_until_max_iter = bool(rospy.get_param("~search_until_max_iter", False))
        self.random = random.Random(int(rospy.get_param("~random_seed", 0)))

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)

        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.obstacle_set = set()

    def map_callback(self, msg):
        self.grid_map = GridMap(msg)
        self.obstacle_set = build_obstacle_set(self.grid_map, self.robot_radius)

    def goal_callback(self, msg):
        if self.grid_map is None:
            rospy.logwarn("RRT* Python: /map has not been received yet.")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "RRT* Python: goal frame must be %s, got %s.",
                self.map_frame,
                msg.header.frame_id,
            )
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        start_x, start_y = start_world
        goal_x = msg.pose.position.x
        goal_y = msg.pose.position.y
        if not self.is_world_point_free(start_x, start_y):
            rospy.logwarn("RRT* Python: start is outside the map or inside an inflated obstacle.")
            return
        if not self.is_world_point_free(goal_x, goal_y):
            rospy.logwarn("RRT* Python: goal is outside the map or inside an inflated obstacle.")
            return

        path_points = self.plan_path(start_x, start_y, goal_x, goal_y)
        if not path_points:
            rospy.logwarn("RRT* Python: no path found.")
            return

        self.publish_path(path_points)

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
                "RRT* Python: failed to lookup %s -> %s: %s",
                self.map_frame,
                self.robot_frame,
                exc,
            )
            return None

    def plan_path(self, start_x, start_y, goal_x, goal_y):
        if euclidean_distance(start_x, start_y, goal_x, goal_y) <= self.path_resolution and self.is_segment_free(
            start_x, start_y, goal_x, goal_y
        ):
            return [(start_x, start_y), (goal_x, goal_y)]

        nodes = [RRTStarNode(start_x, start_y, 0.0, -1)]
        for _ in range(self.max_iterations):
            sampled_node = self.sample_node(goal_x, goal_y)
            nearest_index = self.nearest_node_index(nodes, sampled_node)
            nearest_node = nodes[nearest_index]
            new_node = self.steer(nearest_node, sampled_node)

            if not self.world_in_bounds(new_node.x, new_node.y) or not self.is_segment_free(
                nearest_node.x, nearest_node.y, new_node.x, new_node.y
            ):
                continue

            new_node.parent_index = nearest_index
            new_node.cost = nearest_node.cost + euclidean_distance(
                nearest_node.x, nearest_node.y, new_node.x, new_node.y
            )
            near_indices = self.near_node_indices(nodes, new_node)
            self.choose_parent(nodes, near_indices, new_node)

            nodes.append(new_node)
            new_index = len(nodes) - 1
            self.rewire(nodes, new_index, near_indices)

            if not self.search_until_max_iter:
                best_index = self.best_goal_node_index(nodes, goal_x, goal_y)
                if best_index >= 0:
                    return self.reconstruct_path(nodes, best_index, goal_x, goal_y)

        best_index = self.best_goal_node_index(nodes, goal_x, goal_y)
        if best_index >= 0:
            return self.reconstruct_path(nodes, best_index, goal_x, goal_y)
        return []

    def sample_node(self, goal_x, goal_y):
        if self.random.randint(0, 100) < self.goal_sample_rate:
            return RRTStarNode(goal_x, goal_y)

        gm = self.grid_map
        max_x = gm.origin_x + gm.width * gm.resolution
        max_y = gm.origin_y + gm.height * gm.resolution
        return RRTStarNode(
            self.random.uniform(gm.origin_x, max_x),
            self.random.uniform(gm.origin_y, max_y),
        )

    def steer(self, from_node, to_node):
        node_distance = euclidean_distance(from_node.x, from_node.y, to_node.x, to_node.y)
        if node_distance <= self.expand_distance:
            return RRTStarNode(to_node.x, to_node.y)

        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        return RRTStarNode(
            from_node.x + self.expand_distance * math.cos(theta),
            from_node.y + self.expand_distance * math.sin(theta),
        )

    def nearest_node_index(self, nodes, target_node):
        distances = [
            (node.x - target_node.x) * (node.x - target_node.x)
            + (node.y - target_node.y) * (node.y - target_node.y)
            for node in nodes
        ]
        return distances.index(min(distances))

    def near_node_indices(self, nodes, new_node):
        n_nodes = len(nodes) + 1
        radius = self.connect_circle_distance
        if n_nodes > 1:
            radius *= math.sqrt(math.log(n_nodes) / n_nodes)
        radius = min(radius, self.expand_distance)
        radius_squared = radius * radius

        result = []
        for index, node in enumerate(nodes):
            squared_distance = (node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2
            if squared_distance <= radius_squared:
                result.append(index)
        return result

    def choose_parent(self, nodes, near_indices, new_node):
        for near_index in near_indices:
            near_node = nodes[near_index]
            if not self.is_segment_free(near_node.x, near_node.y, new_node.x, new_node.y):
                continue

            candidate_cost = near_node.cost + euclidean_distance(
                near_node.x, near_node.y, new_node.x, new_node.y
            )
            if candidate_cost < new_node.cost:
                new_node.cost = candidate_cost
                new_node.parent_index = near_index

    def rewire(self, nodes, new_index, near_indices):
        new_node = nodes[new_index]
        for near_index in near_indices:
            if near_index == new_node.parent_index:
                continue

            near_node = nodes[near_index]
            if not self.is_segment_free(new_node.x, new_node.y, near_node.x, near_node.y):
                continue

            candidate_cost = new_node.cost + euclidean_distance(
                new_node.x, new_node.y, near_node.x, near_node.y
            )
            if candidate_cost < near_node.cost:
                near_node.cost = candidate_cost
                near_node.parent_index = new_index
                self.propagate_cost_to_children(nodes, near_index)

    def propagate_cost_to_children(self, nodes, parent_index):
        parent_node = nodes[parent_index]
        for index, node in enumerate(nodes):
            if node.parent_index != parent_index:
                continue

            node.cost = parent_node.cost + euclidean_distance(
                parent_node.x, parent_node.y, node.x, node.y
            )
            self.propagate_cost_to_children(nodes, index)

    def best_goal_node_index(self, nodes, goal_x, goal_y):
        best_index = -1
        best_cost = float("inf")
        for index, node in enumerate(nodes):
            goal_distance = euclidean_distance(node.x, node.y, goal_x, goal_y)
            if goal_distance > self.expand_distance:
                continue
            if not self.is_segment_free(node.x, node.y, goal_x, goal_y):
                continue

            candidate_cost = node.cost + goal_distance
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_index = index
        return best_index

    def reconstruct_path(self, nodes, goal_parent_index, goal_x, goal_y):
        path_points = [(goal_x, goal_y)]
        current_index = goal_parent_index
        while current_index >= 0:
            node = nodes[current_index]
            path_points.append((node.x, node.y))
            current_index = node.parent_index

        path_points.reverse()
        return path_points

    def is_world_point_free(self, world_x, world_y):
        if not self.world_in_bounds(world_x, world_y):
            return False
        grid_x, grid_y = self.grid_map.world_to_grid(world_x, world_y)
        return self.grid_map.to_index(grid_x, grid_y) not in self.obstacle_set

    def is_segment_free(self, from_x, from_y, to_x, to_y):
        segment_length = euclidean_distance(from_x, from_y, to_x, to_y)
        collision_step = max(1.0e-6, min(self.path_resolution, self.grid_map.resolution * 0.5))
        steps = max(1, int(math.ceil(segment_length / collision_step)))

        for step in range(steps + 1):
            ratio = float(step) / float(steps)
            check_x = from_x + (to_x - from_x) * ratio
            check_y = from_y + (to_y - from_y) * ratio
            if not self.is_world_point_free(check_x, check_y):
                return False
        return True

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame

        for world_x, world_y in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def world_in_bounds(self, world_x, world_y):
        gm = self.grid_map
        return (
            gm.origin_x <= world_x < gm.origin_x + gm.width * gm.resolution
            and gm.origin_y <= world_y < gm.origin_y + gm.height * gm.resolution
        )


if __name__ == "__main__":
    rospy.init_node("rrt_star_planner_py")
    RRTStarPlannerNode()
    rospy.spin()
