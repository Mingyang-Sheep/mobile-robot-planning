#!/usr/bin/env python3
import heapq
import math

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path


class GridNode:
    __slots__ = ("x", "y", "g", "h", "parent_index")

    def __init__(self, x, y, g, h, parent_index):
        self.x = x
        self.y = y
        self.g = g
        self.h = h
        self.parent_index = parent_index


class StcPlannerNode:
    def __init__(self):
        # 全覆盖算法统一监听地图输入。
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        # 与 A* 完全对齐：覆盖算法也统一使用 RViz 的 2D Nav Goal 作为触发入口。
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback, queue_size=1
        )
        # 全覆盖路径统一输出到固定 Path 话题。
        self.path_pub = rospy.Publisher(
            "/mr_traditional_planner/coverage_path", Path, queue_size=1, latch=True
        )
        # 统一以 /move_base 为动作客户端接口，覆盖算法只负责编排 Waypoints。
        self.move_base_client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.tf_listener = tf.TransformListener()
        self.latest_map = None
        self.map_width = 0
        self.map_height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0

        # 0.15m 与其它传统算法保持一致，确保所有算法共用同一套碰撞边界。
        self.robot_radius = 0.15
        # 0.2m 用作 STC 粗栅格间距，控制生成树节点的采样密度。
        self.tree_spacing = 0.2
        # 单个 waypoint 超时后直接跳过，避免整条覆盖链路被单点卡住。
        self.goal_timeout = 30.0
        self.inflation_offsets = []
        self.obstacle_set = set()
        self.is_executing = False

        # STC 的安全连接也统一采用 4 邻域 A*，保证连接段绝对不穿障碍。
        self.motion_model = [
            (1, 0, 1.0),
            (0, 1, 1.0),
            (-1, 0, 1.0),
            (0, -1, 1.0),
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
        if msg.header.frame_id and msg.header.frame_id != "map":
            rospy.logwarn(
                "STC Python: 仅支持 map 坐标系触发，当前收到的是 %s。", msg.header.frame_id
            )
            return

        self.trigger_coverage()

    def trigger_coverage(self):
        if self.is_executing:
            rospy.logwarn("STC Python: 当前仍在执行覆盖任务，忽略重复触发。")
            return

        if self.latest_map is None:
            rospy.logwarn("STC Python: /map 尚未收到，无法开始覆盖规划。")
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        # 关键步骤：STC 规划也沿用与 A* 完全一致的 world -> grid 映射公式。
        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        if not self.in_bounds(start_x, start_y):
            rospy.logwarn("STC Python: 清扫起点超出地图范围，停止执行。")
            return

        start_index = self.to_index(start_x, start_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("STC Python: 清扫起点位于障碍物膨胀区内，停止执行。")
            return

        path_indices = self.build_coverage_path(start_x, start_y)
        if not path_indices:
            rospy.logwarn("STC Python: 未生成有效覆盖路径。")
            return

        self.publish_path(path_indices)

        self.is_executing = True
        try:
            if not self.wait_for_move_base_server():
                return

            # RViz 的 2D Nav Goal 在这里仅作为统一触发入口，不保留其原始单点导航目标。
            self.move_base_client.cancel_all_goals()
            self.execute_coverage_path(path_indices)
        finally:
            self.is_executing = False

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
            rospy.logwarn("STC Python: 获取 map -> base_footprint 失败，停止规划。%s", exc)
            return None

    def wait_for_move_base_server(self):
        while not rospy.is_shutdown():
            if self.move_base_client.wait_for_server(rospy.Duration(2.0)):
                return True
            rospy.logwarn("STC Python: 等待 /move_base Action Server 启动。")
        return False

    def build_obstacle_lookup(self):
        self.obstacle_set.clear()
        if self.latest_map is None or self.resolution <= 0.0:
            return

        self.precompute_inflation_offsets()

        for linear_index, occupancy in enumerate(self.latest_map.data):
            # 与 A* / Dijkstra / BCD 保持一致：未知区与占用概率 >= 50 的栅格都视为障碍。
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
                # 关键步骤：仍使用欧式距离做圆形膨胀，确保所有传统算法的碰撞边界一致。
                if math.hypot(offset_x, offset_y) * self.resolution <= self.robot_radius:
                    self.inflation_offsets.append((offset_x, offset_y))

    def build_coverage_path(self, start_x, start_y):
        if self.latest_map is None or self.resolution <= 0.0:
            return []

        # 0.2m 统一换算为 STC 粗栅格步长，既控制树节点密度，也保证不同实现的采样尺度一致。
        coarse_step_cells = max(1, int(round(self.tree_spacing / self.resolution)))
        coarse_width = int(math.ceil(float(self.map_width) / coarse_step_cells))
        coarse_height = int(math.ceil(float(self.map_height) / coarse_step_cells))

        def coarse_key(coarse_x, coarse_y):
            return coarse_y * coarse_width + coarse_x

        def coarse_center(coarse_x, coarse_y):
            anchor_x = coarse_x * coarse_step_cells
            anchor_y = coarse_y * coarse_step_cells
            block_width = min(coarse_step_cells, self.map_width - anchor_x)
            block_height = min(coarse_step_cells, self.map_height - anchor_y)
            return (
                anchor_x + max(0, block_width - 1) // 2,
                anchor_y + max(0, block_height - 1) // 2,
            )

        def coarse_cell_is_free(coarse_x, coarse_y):
            anchor_x = coarse_x * coarse_step_cells
            anchor_y = coarse_y * coarse_step_cells
            end_x = min(anchor_x + coarse_step_cells, self.map_width)
            end_y = min(anchor_y + coarse_step_cells, self.map_height)

            for grid_y in range(anchor_y, end_y):
                for grid_x in range(anchor_x, end_x):
                    if self.is_obstacle(grid_x, grid_y):
                        return False
            return True

        free_nodes = set()
        for coarse_y in range(coarse_height):
            for coarse_x in range(coarse_width):
                if coarse_cell_is_free(coarse_x, coarse_y):
                    free_nodes.add(coarse_key(coarse_x, coarse_y))

        if not free_nodes:
            return []

        start_coarse_x = max(0, min(coarse_width - 1, start_x // coarse_step_cells))
        start_coarse_y = max(0, min(coarse_height - 1, start_y // coarse_step_cells))

        current_key = None
        best_start_distance = float("inf")
        for node_key in free_nodes:
            coarse_x = node_key % coarse_width
            coarse_y = node_key // coarse_width
            center_x, center_y = coarse_center(coarse_x, coarse_y)
            distance = math.hypot(center_x - start_x, center_y - start_y)

            # 优先从机器人所在粗栅格附近开始，找不到时退化到最近自由粗栅格。
            if coarse_x == start_coarse_x and coarse_y == start_coarse_y:
                current_key = node_key
                break

            if distance < best_start_distance:
                best_start_distance = distance
                current_key = node_key

        if current_key is None:
            return []

        center_sequence = []
        globally_visited = set()
        neighbor_order = [(1, 0), (0, 1), (-1, 0), (0, -1)]

        def append_center(node_key):
            coarse_x = node_key % coarse_width
            coarse_y = node_key // coarse_width
            center_x, center_y = coarse_center(coarse_x, coarse_y)
            linear_index = self.to_index(center_x, center_y)
            if not center_sequence or center_sequence[-1] != linear_index:
                center_sequence.append(linear_index)

        def dfs_visit(node_key):
            globally_visited.add(node_key)
            append_center(node_key)

            coarse_x = node_key % coarse_width
            coarse_y = node_key // coarse_width
            for step_x, step_y in neighbor_order:
                neighbor_x = coarse_x + step_x
                neighbor_y = coarse_y + step_y
                if not (0 <= neighbor_x < coarse_width and 0 <= neighbor_y < coarse_height):
                    continue

                neighbor_key = coarse_key(neighbor_x, neighbor_y)
                if neighbor_key not in free_nodes or neighbor_key in globally_visited:
                    continue

                dfs_visit(neighbor_key)
                # 关键步骤：STC 在树边回溯时再次写入父节点中心，形成“沿生成树往返”的覆盖轨迹。
                append_center(node_key)

        while len(globally_visited) < len(free_nodes):
            if current_key not in free_nodes or current_key in globally_visited:
                best_distance = float("inf")
                best_key = None

                current_grid_x, current_grid_y = start_x, start_y
                if center_sequence:
                    current_grid_x, current_grid_y = self.index_to_grid(center_sequence[-1])

                for candidate_key in free_nodes:
                    if candidate_key in globally_visited:
                        continue

                    candidate_x = candidate_key % coarse_width
                    candidate_y = candidate_key // coarse_width
                    center_x, center_y = coarse_center(candidate_x, candidate_y)
                    distance = math.hypot(center_x - current_grid_x, center_y - current_grid_y)
                    if distance < best_distance:
                        best_distance = distance
                        best_key = candidate_key

                if best_key is None:
                    break
                current_key = best_key

            dfs_visit(current_key)

        dense_path = []
        current_x, current_y = start_x, start_y

        for linear_index in center_sequence:
            target_x, target_y = self.index_to_grid(linear_index)
            connector = self.plan_safe_path(current_x, current_y, target_x, target_y)
            if not connector:
                rospy.logwarn("STC Python: 无法安全连接到下一个生成树节点，跳过该节点。")
                continue

            self.append_indices(dense_path, connector)
            current_x, current_y = self.index_to_grid(dense_path[-1])

        return self.compress_path(dense_path)

    def plan_safe_path(self, start_x, start_y, goal_x, goal_y):
        if not self.in_bounds(start_x, start_y) or not self.in_bounds(goal_x, goal_y):
            return []

        start_index = self.to_index(start_x, start_y)
        goal_index = self.to_index(goal_x, goal_y)

        if start_index in self.obstacle_set or goal_index in self.obstacle_set:
            return []

        if start_index == goal_index:
            return [start_index]

        start_h = self.heuristic(start_x, start_y, goal_x, goal_y)
        node_lookup = {start_index: GridNode(start_x, start_y, 0.0, start_h, -1)}
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
                heuristic_cost = self.heuristic(next_x, next_y, goal_x, goal_y)

                existing_node = node_lookup.get(next_index)
                if existing_node is not None and tentative_g >= existing_node.g:
                    continue

                node_lookup[next_index] = GridNode(
                    next_x, next_y, tentative_g, heuristic_cost, current_index
                )
                heapq.heappush(open_heap, (tentative_g + heuristic_cost, heuristic_cost, next_index))

        return []

    def reconstruct_path(self, goal_index, node_lookup):
        path_indices = []
        current_index = goal_index

        while current_index != -1:
            path_indices.append(current_index)
            current_index = node_lookup[current_index].parent_index

        path_indices.reverse()
        return path_indices

    def append_indices(self, target_path, extension):
        for linear_index in extension:
            if not target_path or target_path[-1] != linear_index:
                target_path.append(linear_index)

    def compress_path(self, dense_path):
        if len(dense_path) < 3:
            return list(dense_path)

        compressed_path = [dense_path[0]]
        previous_direction = None

        for waypoint_index in range(1, len(dense_path)):
            prev_x, prev_y = self.index_to_grid(dense_path[waypoint_index - 1])
            curr_x, curr_y = self.index_to_grid(dense_path[waypoint_index])
            direction = (self.sign(curr_x - prev_x), self.sign(curr_y - prev_y))

            if previous_direction is None:
                previous_direction = direction
                continue

            if direction != previous_direction:
                compressed_path.append(dense_path[waypoint_index - 1])
                previous_direction = direction

        if compressed_path[-1] != dense_path[-1]:
            compressed_path.append(dense_path[-1])

        return compressed_path

    def sign(self, value):
        if value > 0:
            return 1
        if value < 0:
            return -1
        return 0

    def heuristic(self, grid_x, grid_y, goal_x, goal_y):
        # 关键步骤：安全连接统一使用曼哈顿启发式，匹配 4 邻域搜索并避免不必要的斜穿。
        return abs(goal_x - grid_x) + abs(goal_y - grid_y)

    def compute_waypoint_yaw(self, path_indices, waypoint_index):
        if len(path_indices) < 2:
            return 0.0

        from_index = path_indices[waypoint_index]
        to_index = path_indices[waypoint_index]

        if waypoint_index + 1 < len(path_indices):
            to_index = path_indices[waypoint_index + 1]
        else:
            from_index = path_indices[waypoint_index - 1]

        from_grid_x, from_grid_y = self.index_to_grid(from_index)
        to_grid_x, to_grid_y = self.index_to_grid(to_index)
        from_world_x, from_world_y = self.grid_to_world(from_grid_x, from_grid_y)
        to_world_x, to_world_y = self.grid_to_world(to_grid_x, to_grid_y)
        return math.atan2(to_world_y - from_world_y, to_world_x - from_world_x)

    def publish_path(self, path_indices):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for waypoint_index, linear_index in enumerate(path_indices):
            grid_x, grid_y = self.index_to_grid(linear_index)
            # 关键步骤：发布可视化路径时仍取栅格中心点，保持与其它传统算法一致的坐标定义。
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            yaw = self.compute_waypoint_yaw(path_indices, waypoint_index)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def execute_coverage_path(self, path_indices):
        for waypoint_index, linear_index in enumerate(path_indices):
            grid_x, grid_y = self.index_to_grid(linear_index)
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            yaw = self.compute_waypoint_yaw(path_indices, waypoint_index)
            quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = world_x
            goal.target_pose.pose.position.y = world_y
            goal.target_pose.pose.orientation.x = quaternion[0]
            goal.target_pose.pose.orientation.y = quaternion[1]
            goal.target_pose.pose.orientation.z = quaternion[2]
            goal.target_pose.pose.orientation.w = quaternion[3]

            self.move_base_client.send_goal(goal)
            finished = self.move_base_client.wait_for_result(rospy.Duration(self.goal_timeout))
            if not finished:
                rospy.logwarn("STC Python: 第 %d 个覆盖点等待超时，跳过该点。", waypoint_index)
                self.move_base_client.cancel_goal()
                continue

            if self.move_base_client.get_state() != GoalStatus.SUCCEEDED:
                rospy.logwarn(
                    "STC Python: 第 %d 个覆盖点执行失败，状态码 %d，跳过该点。",
                    waypoint_index,
                    self.move_base_client.get_state(),
                )

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

    def index_to_grid(self, linear_index):
        return linear_index % self.map_width, linear_index // self.map_width

    def in_bounds(self, grid_x, grid_y):
        return 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height

    def is_obstacle(self, grid_x, grid_y):
        if not self.in_bounds(grid_x, grid_y):
            return True
        return self.to_index(grid_x, grid_y) in self.obstacle_set


if __name__ == "__main__":
    rospy.init_node("stc_planner_py")
    StcPlannerNode()
    rospy.spin()
