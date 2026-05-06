#!/usr/bin/env python3
import bisect
import math

import rospy
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


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
        self.path_topic = rospy.get_param("~path_topic", "/mr_traditional_planner/optimal_path")
        self.input_path_topic = rospy.get_param("~input_path_topic", "")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_radius = rospy.get_param("~robot_radius", 0.15)
        self.spline_resolution = max(1.0e-3, float(rospy.get_param("~spline_resolution", 0.05)))
        self.control_point_ratio = min(
            1.0, max(0.05, float(rospy.get_param("~control_point_ratio", 0.35)))
        )
        self.collision_check = bool(rospy.get_param("~collision_check", False))

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
        self.latest_map = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.inflation_offsets = []
        self.obstacle_set = set()

    def map_callback(self, msg):
        self.latest_map = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.build_obstacle_lookup()

    def goal_callback(self, msg):
        if self.collision_check and self.latest_map is None:
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

        goal_yaw = self.yaw_from_pose(msg)
        anchors = self.build_goal_anchors(
            start_pose[0],
            start_pose[1],
            start_pose[2],
            msg.pose.position.x,
            msg.pose.position.y,
            goal_yaw,
        )
        path_points = self.smooth_anchors(anchors)
        if not path_points:
            rospy.logwarn("CubicSpline Python: failed to generate spline path.")
            return

        if self.collision_check and not self.path_is_free(path_points):
            rospy.logwarn("CubicSpline Python: generated spline path intersects an inflated obstacle.")
            return

        self.publish_path(path_points)

    def input_path_callback(self, msg):
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

        anchors = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        path_points = self.smooth_anchors(anchors)
        if not path_points:
            rospy.logwarn("CubicSpline Python: failed to smooth input path.")
            return

        if self.collision_check and not self.path_is_free(path_points):
            rospy.logwarn("CubicSpline Python: smoothed path intersects an inflated obstacle.")
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

    def smooth_anchors(self, anchors):
        clean_anchors = self.deduplicate_anchors(anchors)
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
        return path_points

    def path_is_free(self, path_points):
        return all(self.is_world_point_free(world_x, world_y) for world_x, world_y in path_points)

    def is_world_point_free(self, world_x, world_y):
        if self.latest_map is None:
            return True
        grid_x, grid_y = self.world_to_grid(world_x, world_y)
        return self.in_bounds(grid_x, grid_y) and self.to_index(grid_x, grid_y) not in self.obstacle_set

    def publish_path(self, path_points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.map_frame

        for index, point in enumerate(path_points):
            yaw = 0.0
            if index + 1 < len(path_points):
                yaw = math.atan2(path_points[index + 1][1] - point[1], path_points[index + 1][0] - point[0])
            elif index > 0:
                yaw = math.atan2(point[1] - path_points[index - 1][1], point[0] - path_points[index - 1][0])

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

    def world_to_grid(self, world_x, world_y):
        return int((world_x - self.origin_x) / self.resolution), int(
            (world_y - self.origin_y) / self.resolution
        )

    def to_index(self, grid_x, grid_y):
        return grid_y * self.map_width + grid_x

    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height


if __name__ == "__main__":
    rospy.init_node("cubic_spline_planner_py")
    CubicSplinePlannerNode()
    rospy.spin()
