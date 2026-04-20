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


class Segment:
    __slots__ = ("y", "x_start", "x_end", "cell_id")

    def __init__(self, y, x_start, x_end, cell_id=-1):
        self.y = y
        self.x_start = x_start
        self.x_end = x_end
        self.cell_id = cell_id


class BcdPlannerNode:
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

        # 0.15m 与前面 A* / Dijkstra 保持一致，保证覆盖路径和最优路径共用同一套碰撞边界。
        self.robot_radius = 0.15
        # 0.2m 是牛耕式相邻扫掠线的间距，直接决定覆盖密度和路径长度。
        self.sweep_spacing = 0.2
        # 单个 waypoint 超时后直接跳过，避免某个坏点拖死整条覆盖链路。
        self.goal_timeout = 30.0
        self.inflation_offsets = []
        self.obstacle_set = set()
        self.is_executing = False

        # 覆盖算法的安全连接统一采用 4 邻域 A*，避免对角穿角导致的碰撞风险。
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
                "BCD Python: 仅支持 map 坐标系触发，当前收到的是 %s。", msg.header.frame_id
            )
            return

        self.trigger_coverage()

    def trigger_coverage(self):
        if self.is_executing:
            rospy.logwarn("BCD Python: 当前仍在执行覆盖任务，忽略重复触发。")
            return

        if self.latest_map is None:
            rospy.logwarn("BCD Python: /map 尚未收到，无法开始覆盖规划。")
            return

        start_world = self.lookup_start_pose()
        if start_world is None:
            return

        # 关键步骤：覆盖规划沿用与 A* 完全相同的 world -> grid 映射公式。
        start_x, start_y = self.world_to_grid(start_world[0], start_world[1])
        if not self.in_bounds(start_x, start_y):
            rospy.logwarn("BCD Python: 清扫起点超出地图范围，停止执行。")
            return

        start_index = self.to_index(start_x, start_y)
        if start_index in self.obstacle_set:
            rospy.logwarn("BCD Python: 清扫起点位于障碍物膨胀区内，停止执行。")
            return

        path_indices = self.build_coverage_path(start_x, start_y)
        if not path_indices:
            rospy.logwarn("BCD Python: 未生成有效覆盖路径。")
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
            rospy.logwarn("BCD Python: 获取 map -> base_footprint 失败，停止规划。%s", exc)
            return None

    def wait_for_move_base_server(self):
        while not rospy.is_shutdown():
            if self.move_base_client.wait_for_server(rospy.Duration(2.0)):
                return True
            rospy.logwarn("BCD Python: 等待 /move_base Action Server 启动。")
        return False

    def build_obstacle_lookup(self):
        self.obstacle_set.clear()
        if self.latest_map is None or self.resolution <= 0.0:
            return

        self.precompute_inflation_offsets()

        for linear_index, occupancy in enumerate(self.latest_map.data):
            # 与 A* / Dijkstra 保持一致：未知区与占用概率 >= 50 的栅格都视为障碍。
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
                # 关键步骤：仍使用欧式距离做圆形膨胀，保证与前面最优路径算法的碰撞边界一致。
                if math.hypot(offset_x, offset_y) * self.resolution <= self.robot_radius:
                    self.inflation_offsets.append((offset_x, offset_y))

    def build_coverage_path(self, start_x, start_y):
        if self.latest_map is None or self.resolution <= 0.0:
            return []

        # 0.2m 的扫掠间距先转成栅格步长，再在离散网格上生成弓字形路径。
        sweep_step_cells = max(1, int(round(self.sweep_spacing / self.resolution)))
        sampled_rows = list(range(0, self.map_height, sweep_step_cells))
        if not sampled_rows or sampled_rows[-1] != self.map_height - 1:
            sampled_rows.append(self.map_height - 1)

        all_segments = []
        cells = {}
        previous_row_segment_indices = []
        next_cell_id = 0

        for row_y in sampled_rows:
            current_row_segment_indices = []
            for segment in self.extract_free_segments(row_y):
                parent_cell_ids = set()
                for previous_segment_index in previous_row_segment_indices:
                    previous_segment = all_segments[previous_segment_index]
                    if self.segments_overlap(previous_segment, segment):
                        parent_cell_ids.add(previous_segment.cell_id)

                # 简化版 BCD：若扫掠线拓扑发生分裂或合并，则开启新 Cell；否则沿用唯一父 Cell。
                if len(parent_cell_ids) == 1:
                    segment.cell_id = next(iter(parent_cell_ids))
                else:
                    segment.cell_id = next_cell_id
                    next_cell_id += 1

                all_segments.append(segment)
                segment_index = len(all_segments) - 1
                cells.setdefault(segment.cell_id, []).append(segment_index)
                current_row_segment_indices.append(segment_index)

            previous_row_segment_indices = current_row_segment_indices

        cell_paths = {}
        for cell_id, segment_indices in cells.items():
            ordered_segments = sorted(
                (all_segments[segment_index] for segment_index in segment_indices),
                key=lambda segment: (segment.y, segment.x_start),
            )
            cell_path = self.generate_cell_path(ordered_segments)
            if cell_path:
                cell_paths[cell_id] = cell_path

        dense_path = []
        current_x, current_y = start_x, start_y

        while cell_paths:
            candidates = []
            for cell_id, cell_path in cell_paths.items():
                entry_x, entry_y = self.index_to_grid(cell_path[0])
                exit_x, exit_y = self.index_to_grid(cell_path[-1])

                candidates.append((math.hypot(entry_x - current_x, entry_y - current_y), cell_id, False))
                candidates.append((math.hypot(exit_x - current_x, exit_y - current_y), cell_id, True))

            candidates.sort(key=lambda item: item[0])

            selected = None
            for _, cell_id, reverse_cell in candidates:
                cell_path = cell_paths[cell_id]
                ordered_cell_path = list(reversed(cell_path)) if reverse_cell else cell_path
                entry_x, entry_y = self.index_to_grid(ordered_cell_path[0])
                connector = self.plan_safe_path(current_x, current_y, entry_x, entry_y)
                if connector:
                    selected = (cell_id, connector, ordered_cell_path)
                    break

            if selected is None:
                rospy.logwarn("BCD Python: 剩余 Cell 无法与当前路径安全连接，提前结束覆盖。")
                break

            cell_id, connector, ordered_cell_path = selected
            self.append_indices(dense_path, connector)
            self.append_indices(dense_path, ordered_cell_path)
            cell_paths.pop(cell_id)

            current_x, current_y = self.index_to_grid(dense_path[-1])

        return self.compress_path(dense_path)

    def extract_free_segments(self, row_y):
        segments = []
        grid_x = 0

        while grid_x < self.map_width:
            while grid_x < self.map_width and self.is_obstacle(grid_x, row_y):
                grid_x += 1

            if grid_x >= self.map_width:
                break

            segment_start = grid_x
            while grid_x + 1 < self.map_width and not self.is_obstacle(grid_x + 1, row_y):
                grid_x += 1

            segments.append(Segment(row_y, segment_start, grid_x))
            grid_x += 1

        return segments

    def segments_overlap(self, lhs, rhs):
        return lhs.x_start <= rhs.x_end and rhs.x_start <= lhs.x_end

    def generate_cell_path(self, cell_segments):
        dense_path = []
        left_to_right = True

        for segment in cell_segments:
            segment_path = self.trace_scan_segment(segment, left_to_right)
            left_to_right = not left_to_right

            if not segment_path:
                continue

            if dense_path:
                current_x, current_y = self.index_to_grid(dense_path[-1])
                segment_start_x, segment_start_y = self.index_to_grid(segment_path[0])
                connector = self.plan_safe_path(
                    current_x, current_y, segment_start_x, segment_start_y
                )
                if not connector:
                    rospy.logwarn("BCD Python: Cell 内部扫描段之间无法安全连接，终止当前 Cell。")
                    break
                self.append_indices(dense_path, connector)

            self.append_indices(dense_path, segment_path)

        return dense_path

    def trace_scan_segment(self, segment, left_to_right):
        traced_indices = []
        start_x = segment.x_start if left_to_right else segment.x_end
        end_x = segment.x_end if left_to_right else segment.x_start
        step_x = 1 if end_x >= start_x else -1
        grid_x = start_x

        while True:
            # 关键步骤：牛耕扫描段沿着整条线逐栅格检查，一旦撞到障碍立即截断，不允许穿墙。
            if self.is_obstacle(grid_x, segment.y):
                break

            traced_indices.append(self.to_index(grid_x, segment.y))
            if grid_x == end_x:
                break
            grid_x += step_x

        return traced_indices

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

    def publish_path(self, path_indices):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for waypoint_index, linear_index in enumerate(path_indices):
            grid_x, grid_y = self.index_to_grid(linear_index)
            # 关键步骤：发布可视化路径时仍取栅格中心点，保持与 A* / Dijkstra 一致的索引还原逻辑。
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
                rospy.logwarn("BCD Python: 第 %d 个覆盖点等待超时，跳过该点。", waypoint_index)
                self.move_base_client.cancel_goal()
                continue

            if self.move_base_client.get_state() != GoalStatus.SUCCEEDED:
                rospy.logwarn(
                    "BCD Python: 第 %d 个覆盖点执行失败，状态码 %d，跳过该点。",
                    waypoint_index,
                    self.move_base_client.get_state(),
                )

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
    rospy.init_node("bcd_planner_py")
    BcdPlannerNode()
    rospy.spin()
