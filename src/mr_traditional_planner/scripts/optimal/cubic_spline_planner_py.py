#!/usr/bin/env python3
import bisect
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


class CubicSpline1D:
    def __init__(self, x_values, y_values):
        self.x = list(x_values)
        self.a = list(y_values)
        point_count = len(self.x)
        self.b = [0.0] * max(0, point_count - 1)
        self.c = [0.0] * point_count
        self.d = [0.0] * max(0, point_count - 1)
        if point_count < 2:
            return

        h = [self.x[i + 1] - self.x[i] for i in range(point_count - 1)]
        alpha = [0.0] * point_count
        for i in range(1, point_count - 1):
            alpha[i] = (
                3.0 * (self.a[i + 1] - self.a[i]) / h[i]
                - 3.0 * (self.a[i] - self.a[i - 1]) / h[i - 1]
            )

        lower = [1.0] * point_count
        mu = [0.0] * point_count
        z = [0.0] * point_count
        for i in range(1, point_count - 1):
            lower[i] = 2.0 * (self.x[i + 1] - self.x[i - 1]) - h[i - 1] * mu[i - 1]
            mu[i] = h[i] / lower[i]
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / lower[i]

        for j in range(point_count - 2, -1, -1):
            self.c[j] = z[j] - mu[j] * self.c[j + 1]
            self.b[j] = (
                (self.a[j + 1] - self.a[j]) / h[j]
                - h[j] * (self.c[j + 1] + 2.0 * self.c[j]) / 3.0
            )
            self.d[j] = (self.c[j + 1] - self.c[j]) / (3.0 * h[j])

    def calc_position(self, x_value):
        index = self.search_index(x_value)
        dx = x_value - self.x[index]
        return (
            self.a[index]
            + self.b[index] * dx
            + self.c[index] * dx * dx
            + self.d[index] * dx * dx * dx
        )

    def calc_first_derivative(self, x_value):
        index = self.search_index(x_value)
        dx = x_value - self.x[index]
        return self.b[index] + 2.0 * self.c[index] * dx + 3.0 * self.d[index] * dx * dx

    def search_index(self, x_value):
        index = bisect.bisect(self.x, x_value) - 1
        return min(max(index, 0), len(self.b) - 1)


class CubicSpline2D:
    def __init__(self, points):
        self.s = self.calc_s(points)
        self.sx = CubicSpline1D(self.s, [point[0] for point in points])
        self.sy = CubicSpline1D(self.s, [point[1] for point in points])

    @staticmethod
    def calc_s(points):
        distances = [0.0]
        for index in range(1, len(points)):
            distances.append(
                distances[-1]
                + math.hypot(
                    points[index][0] - points[index - 1][0],
                    points[index][1] - points[index - 1][1],
                )
            )
        return distances

    def calc_position(self, s_value):
        return self.sx.calc_position(s_value), self.sy.calc_position(s_value)


class CubicSplinePlannerNode:
    def __init__(self):
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/debug_optimal_path")
        self.input_path_topic = rospy.get_param("~input_path_topic", "")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)
        self.spline_resolution = max(1.0e-3, float(rospy.get_param("~spline_resolution", 0.05)))
        self.control_point_ratio = min(
            1.0, max(0.05, float(rospy.get_param("~control_point_ratio", 0.35)))
        )
        self.collision_check = bool(rospy.get_param("~collision_check", True))

        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.goal_sub = rospy.Subscriber(
            self.goal_topic, PoseStamped, self.goal_callback, queue_size=1
        )
        self.input_path_sub = None
        if self.input_path_topic:
            self.input_path_sub = rospy.Subscriber(
                self.input_path_topic, Path, self.input_path_callback, queue_size=1
            )
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1, latch=True)

        self.tf_listener = tf.TransformListener()
        self.grid_map = None
        self.obstacle_set = set()

    def map_callback(self, msg):
        self.grid_map = GridMap(msg)
        self.obstacle_set = build_obstacle_set(self.grid_map, self.robot_radius)

    def goal_callback(self, msg):
        if self.collision_check and self.grid_map is None:
            rospy.logwarn("CubicSpline Python: /map has not been received yet.")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "CubicSpline Python: goal frame must be %s, got %s.",
                self.map_frame,
                msg.header.frame_id,
            )
            return

        start_pose = self.lookup_start_pose()
        if start_pose is None:
            return

        if self.collision_check:
            raw_path = self.build_grid_path(
                start_pose[0], start_pose[1], msg.pose.position.x, msg.pose.position.y
            )
            if not raw_path:
                rospy.logwarn("CubicSpline Python: failed to generate collision-free raw path.")
                return
        else:
            goal_yaw = self.yaw_from_pose(msg)
            raw_path = self.build_goal_anchors(
                start_pose[0],
                start_pose[1],
                start_pose[2],
                msg.pose.position.x,
                msg.pose.position.y,
                goal_yaw,
            )

        raw_path = self.sanitize_path_points(raw_path)
        path_points = self.smooth_anchors(raw_path)
        if not path_points:
            rospy.logwarn("CubicSpline Python: failed to generate spline path.")
            return

        if self.collision_check and not self.path_is_free(path_points):
            rospy.logwarn("[CubicSpline] collision detected, fallback to raw path")
            self.publish_path(raw_path)
            return

        self.publish_path(path_points)

    def input_path_callback(self, msg):
        if self.collision_check and self.grid_map is None:
            rospy.logwarn("CubicSpline Python: /map has not been received yet.")
            return

        if len(msg.poses) < 2:
            rospy.logwarn("CubicSpline Python: input path needs at least two poses.")
            return

        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            rospy.logwarn(
                "CubicSpline Python: input path frame must be %s, got %s.",
                self.map_frame,
                msg.header.frame_id,
            )
            return

        anchors = self.sanitize_path_points(
            [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        )
        if len(anchors) < 2:
            rospy.logwarn("CubicSpline Python: input path has fewer than two valid poses after cleanup.")
            return

        path_points = self.smooth_anchors(anchors)
        if not path_points:
            rospy.logwarn("CubicSpline Python: failed to smooth input path.")
            return

        if self.collision_check and not self.path_is_free(path_points):
            rospy.logwarn("[CubicSpline] collision detected, fallback to raw path")
            self.publish_path(anchors)
            return

        self.publish_path(path_points)

    def lookup_start_pose(self):
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(0.2)
            )
            translation, rotation = self.tf_listener.lookupTransform(
                self.map_frame, self.robot_frame, rospy.Time(0)
            )
            yaw = tf.transformations.euler_from_quaternion(rotation)[2]
            return translation[0], translation[1], yaw
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn(
                "CubicSpline Python: failed to lookup %s -> %s: %s",
                self.map_frame,
                self.robot_frame,
                exc,
            )
            return None

    def build_goal_anchors(self, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw):
        path_length = math.hypot(goal_x - start_x, goal_y - start_y)
        if path_length < 1.0e-6:
            return [(start_x, start_y)]

        tangent_length = path_length * self.control_point_ratio
        return self.deduplicate_anchors(
            [
                (start_x, start_y),
                (
                    start_x + tangent_length * math.cos(start_yaw),
                    start_y + tangent_length * math.sin(start_yaw),
                ),
                (
                    goal_x - tangent_length * math.cos(goal_yaw),
                    goal_y - tangent_length * math.sin(goal_yaw),
                ),
                (goal_x, goal_y),
            ]
        )

    def build_grid_path(self, start_x, start_y, goal_x, goal_y):
        if self.grid_map is None or self.grid_map.resolution <= 0.0:
            return []

        start_grid = self.world_to_grid(start_x, start_y)
        goal_grid = self.world_to_grid(goal_x, goal_y)
        if self.grid_map.is_obstacle(start_grid[0], start_grid[1], self.obstacle_set):
            return []
        if self.grid_map.is_obstacle(goal_grid[0], goal_grid[1], self.obstacle_set):
            return []

        start_index = self.grid_map.to_index(start_grid[0], start_grid[1])
        goal_index = self.grid_map.to_index(goal_grid[0], goal_grid[1])
        total_cells = self.grid_map.width * self.grid_map.height
        costs = [float("inf")] * total_cells
        parents = [-1] * total_cells
        closed = [False] * total_cells

        def heuristic(grid_x, grid_y):
            return (
                math.hypot(goal_grid[0] - grid_x, goal_grid[1] - grid_y)
                * self.grid_map.resolution
            )

        costs[start_index] = 0.0
        parents[start_index] = start_index
        open_set = [(heuristic(start_grid[0], start_grid[1]), start_index)]
        directions = (
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        )

        while open_set:
            _, current_index = heapq.heappop(open_set)
            if closed[current_index]:
                continue
            closed[current_index] = True
            if current_index == goal_index:
                break

            current_x, current_y = self.grid_map.index_to_grid(current_index)
            for dx, dy in directions:
                next_x = current_x + dx
                next_y = current_y + dy
                if self.grid_map.is_obstacle(next_x, next_y, self.obstacle_set):
                    continue
                if dx != 0 and dy != 0 and (
                    self.grid_map.is_obstacle(current_x + dx, current_y, self.obstacle_set)
                    or self.grid_map.is_obstacle(current_x, current_y + dy, self.obstacle_set)
                ):
                    continue

                next_index = self.grid_map.to_index(next_x, next_y)
                step_cost = math.hypot(dx, dy) * self.grid_map.resolution
                next_cost = costs[current_index] + step_cost
                if next_cost < costs[next_index]:
                    costs[next_index] = next_cost
                    parents[next_index] = current_index
                    heapq.heappush(open_set, (next_cost + heuristic(next_x, next_y), next_index))

        if parents[goal_index] < 0:
            return []

        reversed_indices = []
        current_index = goal_index
        while current_index != start_index:
            reversed_indices.append(current_index)
            current_index = parents[current_index]
        reversed_indices.append(start_index)
        reversed_indices.reverse()

        path_points = []
        for index in reversed_indices:
            grid_x, grid_y = self.grid_map.index_to_grid(index)
            path_points.append(self.grid_map.grid_to_world(grid_x, grid_y))
        if path_points:
            path_points[0] = (start_x, start_y)
            path_points[-1] = (goal_x, goal_y)
        return self.sanitize_path_points(path_points)

    def smooth_anchors(self, anchors):
        clean_anchors = self.sanitize_path_points(anchors)
        if len(clean_anchors) < 2:
            return clean_anchors

        spline = CubicSpline2D(clean_anchors)
        max_s = spline.s[-1]
        if max_s < 1.0e-6:
            return clean_anchors

        path_points = []
        s_value = 0.0
        while s_value < max_s:
            path_points.append(spline.calc_position(s_value))
            s_value += self.spline_resolution
        path_points.append(spline.calc_position(max_s))
        return self.sanitize_path_points(path_points)

    def path_cleanup_distance(self):
        fallback_distance = max(1.0e-6, self.spline_resolution * 0.5)
        if self.grid_map is None or self.grid_map.resolution <= 0.0:
            return fallback_distance
        return max(1.0e-6, min(fallback_distance, self.grid_map.resolution * 0.25))

    def sanitize_path_points(self, path_points):
        clean_points = []
        min_distance = self.path_cleanup_distance()

        for point in path_points:
            if not math.isfinite(point[0]) or not math.isfinite(point[1]):
                continue
            if clean_points and math.hypot(
                point[0] - clean_points[-1][0], point[1] - clean_points[-1][1]
            ) < min_distance:
                continue
            if len(clean_points) >= 2 and math.hypot(
                point[0] - clean_points[-2][0], point[1] - clean_points[-2][1]
            ) < min_distance:
                clean_points.pop()
                continue
            clean_points.append(point)

        return clean_points

    def path_is_free(self, path_points):
        if not path_points:
            return True

        if not self.is_world_point_free(path_points[0][0], path_points[0][1]):
            return False

        sample_step = self.collision_check_step()
        for index in range(1, len(path_points)):
            start = path_points[index - 1]
            end = path_points[index]
            distance = math.hypot(end[0] - start[0], end[1] - start[1])
            sample_count = max(1, int(math.ceil(distance / sample_step)))
            for sample in range(1, sample_count + 1):
                ratio = float(sample) / float(sample_count)
                world_x = start[0] + (end[0] - start[0]) * ratio
                world_y = start[1] + (end[1] - start[1]) * ratio
                if not self.is_world_point_free(world_x, world_y):
                    return False
        return True

    def collision_check_step(self):
        fallback_step = max(1.0e-6, self.spline_resolution)
        if self.grid_map is None or self.grid_map.resolution <= 0.0:
            return fallback_step
        return max(1.0e-6, min(fallback_step, self.grid_map.resolution * 0.5))

    def is_world_point_free(self, world_x, world_y):
        if not math.isfinite(world_x) or not math.isfinite(world_y):
            return False
        if self.grid_map is None:
            return True
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        return self.grid_map.in_bounds(grid_x, grid_y) and self.grid_map.to_index(grid_x, grid_y) not in self.obstacle_set

    def world_to_grid(self, world_x, world_y):
        return (
            int(math.floor((world_x - self.grid_map.origin_x) / self.grid_map.resolution)),
            int(math.floor((world_y - self.grid_map.origin_y) / self.grid_map.resolution)),
        )

    def publish_path(self, path_points):
        clean_points = self.sanitize_path_points(path_points)
        if not clean_points:
            return

        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame

        for index, point in enumerate(clean_points):
            yaw = 0.0
            if index + 1 < len(clean_points):
                yaw = math.atan2(clean_points[index + 1][1] - point[1], clean_points[index + 1][0] - point[0])
            elif index > 0:
                yaw = math.atan2(point[1] - clean_points[index - 1][1], point[0] - clean_points[index - 1][0])

            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    @staticmethod
    def deduplicate_anchors(anchors):
        result = []
        for point in anchors:
            if result and math.hypot(point[0] - result[-1][0], point[1] - result[-1][1]) < 1.0e-6:
                continue
            result.append(point)
        return result

    @staticmethod
    def yaw_from_pose(pose_msg):
        quaternion = (
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        )
        return tf.transformations.euler_from_quaternion(quaternion)[2]


if __name__ == "__main__":
    rospy.init_node("cubic_spline_planner_py")
    CubicSplinePlannerNode()
    rospy.spin()
