#include "mr_traditional_planner/coverage/stc_planner.h"

#include <actionlib/client/simple_client_goal_state.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace coverage {

namespace {

struct OpenItem {
  double f;
  double h;
  int index;

  bool operator<(const OpenItem& other) const {
    if (f != other.f) {
      return f > other.f;
    }

    if (h != other.h) {
      return h > other.h;
    }

    return index > other.index;
  }
};

}  // namespace

StcPlanner::StcPlanner()
    : move_base_client_("/move_base", true),
      map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15),
      tree_spacing_(0.2),
      goal_timeout_(30.0),
      is_executing_(false) {
  // 全覆盖算法统一以地图为输入。
  map_sub_ = nh_.subscribe("/map", 1, &StcPlanner::mapCallback, this);
  // 与 A* 完全对齐：覆盖算法也统一使用 RViz 的 2D Nav Goal 作为触发入口。
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &StcPlanner::goalCallback, this);
  // 全覆盖路径统一输出到专用 Path 话题，便于可视化与评测。
  path_pub_ = nh_.advertise<nav_msgs::Path>("/mr_traditional_planner/coverage_path", 1, true);
}

void StcPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void StcPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
    ROS_WARN_STREAM("STC C++: 仅支持 map 坐标系触发，当前收到的是 "
                    << msg->header.frame_id << "。");
    return;
  }

  if (is_executing_) {
    ROS_WARN("STC C++: 当前仍在执行覆盖任务，忽略重复触发。");
    return;
  }

  if (!latest_map_) {
    ROS_WARN("STC C++: /map 尚未收到，无法开始覆盖规划。");
    return;
  }

  double start_world_x = 0.0;
  double start_world_y = 0.0;
  if (!lookupStartPose(start_world_x, start_world_y)) {
    return;
  }

  // 关键步骤：STC 规划也沿用与 A* 完全一致的世界坐标到栅格索引转换公式。
  const std::pair<int, int> start = worldToGrid(start_world_x, start_world_y);
  if (!inBounds(start.first, start.second)) {
    ROS_WARN("STC C++: 清扫起点超出地图范围，停止执行。");
    return;
  }

  if (isObstacle(start.first, start.second)) {
    ROS_WARN("STC C++: 清扫起点位于障碍物膨胀区内，停止执行。");
    return;
  }

  const std::vector<int> path_indices = buildCoveragePath(start.first, start.second);
  if (path_indices.empty()) {
    ROS_WARN("STC C++: 未生成有效覆盖路径。");
    return;
  }

  publishPath(path_indices);

  is_executing_ = true;
  if (!waitForMoveBaseServer()) {
    is_executing_ = false;
    return;
  }

  // RViz 的 2D Nav Goal 在这里仅作为统一触发入口，不保留其原始单点导航目标。
  move_base_client_.cancelAllGoals();
  executeCoveragePath(path_indices);
  is_executing_ = false;
}

void StcPlanner::buildObstacleLookup() {
  obstacle_grid_.assign(static_cast<std::size_t>(map_width_ * map_height_), 0U);
  if (!latest_map_ || resolution_ <= 0.0) {
    return;
  }

  precomputeInflationOffsets();

  for (std::size_t linear_index = 0; linear_index < latest_map_->data.size(); ++linear_index) {
    const int occupancy = latest_map_->data[linear_index];
    // 与 A* / Dijkstra / BCD 保持一致：未知区与占用概率 >= 50 的栅格都视为障碍。
    if (occupancy < 0 || occupancy >= 50) {
      const int obstacle_x = static_cast<int>(linear_index % static_cast<std::size_t>(map_width_));
      const int obstacle_y = static_cast<int>(linear_index / static_cast<std::size_t>(map_width_));

      for (const std::pair<int, int>& offset : inflation_offsets_) {
        const int inflated_x = obstacle_x + offset.first;
        const int inflated_y = obstacle_y + offset.second;

        if (inBounds(inflated_x, inflated_y)) {
          obstacle_grid_[static_cast<std::size_t>(toIndex(inflated_x, inflated_y))] = 1U;
        }
      }
    }
  }
}

void StcPlanner::precomputeInflationOffsets() {
  inflation_offsets_.clear();
  const int inflation_radius_in_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));

  for (int offset_y = -inflation_radius_in_cells; offset_y <= inflation_radius_in_cells; ++offset_y) {
    for (int offset_x = -inflation_radius_in_cells; offset_x <= inflation_radius_in_cells; ++offset_x) {
      // 关键步骤：仍使用欧式距离做圆形膨胀，确保所有传统算法的碰撞边界一致。
      if (std::hypot(static_cast<double>(offset_x), static_cast<double>(offset_y)) * resolution_ <=
          robot_radius_) {
        inflation_offsets_.emplace_back(offset_x, offset_y);
      }
    }
  }
}

bool StcPlanner::lookupStartPose(double& start_world_x, double& start_world_y) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.2));
    tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM("STC C++: 获取 map -> base_footprint 失败，停止规划。" << ex.what());
    return false;
  }

  start_world_x = transform.getOrigin().x();
  start_world_y = transform.getOrigin().y();
  return true;
}

bool StcPlanner::waitForMoveBaseServer() {
  while (ros::ok()) {
    if (move_base_client_.waitForServer(ros::Duration(2.0))) {
      return true;
    }

    ROS_WARN("STC C++: 等待 /move_base Action Server 启动。");
  }

  return false;
}

bool StcPlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool StcPlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }

  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

int StcPlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> StcPlanner::indexToGrid(int linear_index) const {
  return std::make_pair(linear_index % map_width_, linear_index / map_width_);
}

std::pair<int, int> StcPlanner::worldToGrid(double world_x, double world_y) const {
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
}

std::pair<double, double> StcPlanner::gridToWorld(int grid_x, int grid_y) const {
  return std::make_pair(origin_x_ + (static_cast<double>(grid_x) + 0.5) * resolution_,
                        origin_y_ + (static_cast<double>(grid_y) + 0.5) * resolution_);
}

std::vector<int> StcPlanner::planSafePath(int start_x, int start_y, int goal_x, int goal_y) const {
  if (!inBounds(start_x, start_y) || !inBounds(goal_x, goal_y)) {
    return std::vector<int>();
  }

  const int start_index = toIndex(start_x, start_y);
  const int goal_index = toIndex(goal_x, goal_y);
  if (isObstacle(start_x, start_y) || isObstacle(goal_x, goal_y)) {
    return std::vector<int>();
  }

  if (start_index == goal_index) {
    return std::vector<int>(1, start_index);
  }

  static const std::vector<std::pair<int, int>> kMotionModel = {
      std::make_pair(1, 0), std::make_pair(0, 1), std::make_pair(-1, 0), std::make_pair(0, -1)};

  const double start_h = heuristic(start_x, start_y, goal_x, goal_y);
  std::priority_queue<OpenItem> open_set;
  std::unordered_map<int, GridNode> node_lookup;
  std::unordered_set<int> closed_set;
  node_lookup.reserve(static_cast<std::size_t>(map_width_ * map_height_ / 4));
  closed_set.reserve(static_cast<std::size_t>(map_width_ * map_height_ / 4));

  node_lookup.emplace(start_index, GridNode{start_x, start_y, 0.0, start_h, -1});
  open_set.push(OpenItem{start_h, start_h, start_index});

  while (!open_set.empty()) {
    const OpenItem current_item = open_set.top();
    open_set.pop();

    if (closed_set.find(current_item.index) != closed_set.end()) {
      continue;
    }

    const auto current_it = node_lookup.find(current_item.index);
    if (current_it == node_lookup.end()) {
      continue;
    }

    const GridNode& current_node = current_it->second;
    if (current_item.index == goal_index) {
      return reconstructPath(goal_index, node_lookup);
    }

    closed_set.insert(current_item.index);

    for (const std::pair<int, int>& motion : kMotionModel) {
      const int next_x = current_node.x + motion.first;
      const int next_y = current_node.y + motion.second;

      if (!inBounds(next_x, next_y) || isObstacle(next_x, next_y)) {
        continue;
      }

      const int next_index = toIndex(next_x, next_y);
      if (closed_set.find(next_index) != closed_set.end()) {
        continue;
      }

      const double tentative_g = current_node.g + 1.0;
      const double heuristic_cost = heuristic(next_x, next_y, goal_x, goal_y);

      const auto existing_it = node_lookup.find(next_index);
      if (existing_it != node_lookup.end() && tentative_g >= existing_it->second.g) {
        continue;
      }

      node_lookup[next_index] = GridNode{next_x, next_y, tentative_g, heuristic_cost, current_item.index};
      open_set.push(OpenItem{tentative_g + heuristic_cost, heuristic_cost, next_index});
    }
  }

  return std::vector<int>();
}

std::vector<int> StcPlanner::reconstructPath(
    int goal_index, const std::unordered_map<int, GridNode>& node_lookup) const {
  std::vector<int> path_indices;
  int current_index = goal_index;

  while (current_index != -1) {
    path_indices.push_back(current_index);
    const auto node_it = node_lookup.find(current_index);
    if (node_it == node_lookup.end()) {
      break;
    }

    current_index = node_it->second.parent_index;
  }

  std::reverse(path_indices.begin(), path_indices.end());
  return path_indices;
}

void StcPlanner::appendIndices(const std::vector<int>& extension, std::vector<int>& path_indices) const {
  for (const int linear_index : extension) {
    if (path_indices.empty() || path_indices.back() != linear_index) {
      path_indices.push_back(linear_index);
    }
  }
}

std::vector<int> StcPlanner::compressPath(const std::vector<int>& path_indices) const {
  if (path_indices.size() < 3U) {
    return path_indices;
  }

  auto sign = [](int value) {
    if (value > 0) {
      return 1;
    }

    if (value < 0) {
      return -1;
    }

    return 0;
  };

  std::vector<int> compressed_path;
  compressed_path.reserve(path_indices.size());
  compressed_path.push_back(path_indices.front());

  bool has_previous_direction = false;
  std::pair<int, int> previous_direction = std::make_pair(0, 0);

  for (std::size_t waypoint_index = 1; waypoint_index < path_indices.size(); ++waypoint_index) {
    const std::pair<int, int> previous_grid = indexToGrid(path_indices[waypoint_index - 1U]);
    const std::pair<int, int> current_grid = indexToGrid(path_indices[waypoint_index]);
    const std::pair<int, int> current_direction =
        std::make_pair(sign(current_grid.first - previous_grid.first),
                       sign(current_grid.second - previous_grid.second));

    if (!has_previous_direction) {
      previous_direction = current_direction;
      has_previous_direction = true;
      continue;
    }

    if (current_direction != previous_direction) {
      if (compressed_path.back() != path_indices[waypoint_index - 1U]) {
        compressed_path.push_back(path_indices[waypoint_index - 1U]);
      }

      previous_direction = current_direction;
    }
  }

  if (compressed_path.back() != path_indices.back()) {
    compressed_path.push_back(path_indices.back());
  }

  return compressed_path;
}

std::vector<int> StcPlanner::buildCoveragePath(int start_x, int start_y) const {
  if (!latest_map_ || resolution_ <= 0.0) {
    return std::vector<int>();
  }

  // 0.2m 统一换算为 STC 粗栅格步长，既控制树节点密度，也保证不同实现的采样尺度一致。
  const int coarse_step_cells = std::max(1, static_cast<int>(std::round(tree_spacing_ / resolution_)));
  const int coarse_width = (map_width_ + coarse_step_cells - 1) / coarse_step_cells;
  const int coarse_height = (map_height_ + coarse_step_cells - 1) / coarse_step_cells;

  auto coarseKey = [coarse_width](int coarse_x, int coarse_y) {
    return coarse_y * coarse_width + coarse_x;
  };

  auto coarseCenter = [this, coarse_step_cells](int coarse_x, int coarse_y) {
    const int anchor_x = coarse_x * coarse_step_cells;
    const int anchor_y = coarse_y * coarse_step_cells;
    const int block_width = std::min(coarse_step_cells, map_width_ - anchor_x);
    const int block_height = std::min(coarse_step_cells, map_height_ - anchor_y);
    return std::make_pair(anchor_x + std::max(0, block_width - 1) / 2,
                          anchor_y + std::max(0, block_height - 1) / 2);
  };

  auto coarseCellIsFree = [this, coarse_step_cells](int coarse_x, int coarse_y) {
    const int anchor_x = coarse_x * coarse_step_cells;
    const int anchor_y = coarse_y * coarse_step_cells;
    const int end_x = std::min(anchor_x + coarse_step_cells, map_width_);
    const int end_y = std::min(anchor_y + coarse_step_cells, map_height_);

    for (int grid_y = anchor_y; grid_y < end_y; ++grid_y) {
      for (int grid_x = anchor_x; grid_x < end_x; ++grid_x) {
        if (isObstacle(grid_x, grid_y)) {
          return false;
        }
      }
    }

    return true;
  };

  std::unordered_set<int> free_nodes;
  free_nodes.reserve(static_cast<std::size_t>(coarse_width * coarse_height / 2));

  for (int coarse_y = 0; coarse_y < coarse_height; ++coarse_y) {
    for (int coarse_x = 0; coarse_x < coarse_width; ++coarse_x) {
      if (coarseCellIsFree(coarse_x, coarse_y)) {
        free_nodes.insert(coarseKey(coarse_x, coarse_y));
      }
    }
  }

  if (free_nodes.empty()) {
    return std::vector<int>();
  }

  const int start_coarse_x = std::max(0, std::min(coarse_width - 1, start_x / coarse_step_cells));
  const int start_coarse_y = std::max(0, std::min(coarse_height - 1, start_y / coarse_step_cells));

  int current_key = -1;
  double best_start_distance = std::numeric_limits<double>::infinity();
  for (const int node_key : free_nodes) {
    const int coarse_x = node_key % coarse_width;
    const int coarse_y = node_key / coarse_width;
    const std::pair<int, int> center = coarseCenter(coarse_x, coarse_y);
    const double distance = std::hypot(static_cast<double>(center.first - start_x),
                                       static_cast<double>(center.second - start_y));

    // 优先从机器人所在粗栅格附近开始，找不到时退化到最近自由粗栅格。
    if (coarse_x == start_coarse_x && coarse_y == start_coarse_y) {
      current_key = node_key;
      break;
    }

    if (distance < best_start_distance) {
      best_start_distance = distance;
      current_key = node_key;
    }
  }

  if (current_key < 0) {
    return std::vector<int>();
  }

  std::vector<int> center_sequence;
  std::unordered_set<int> globally_visited;
  globally_visited.reserve(free_nodes.size());
  static const std::vector<std::pair<int, int>> kNeighborOrder = {
      std::make_pair(1, 0), std::make_pair(0, 1), std::make_pair(-1, 0), std::make_pair(0, -1)};

  auto appendCenter = [&center_sequence, &coarseCenter, this, coarse_width](int node_key) {
    const int coarse_x = node_key % coarse_width;
    const int coarse_y = node_key / coarse_width;
    const std::pair<int, int> center = coarseCenter(coarse_x, coarse_y);
    const int linear_index = toIndex(center.first, center.second);
    if (center_sequence.empty() || center_sequence.back() != linear_index) {
      center_sequence.push_back(linear_index);
    }
  };

  while (globally_visited.size() < free_nodes.size()) {
    if (free_nodes.find(current_key) == free_nodes.end() ||
        globally_visited.find(current_key) != globally_visited.end()) {
      double best_distance = std::numeric_limits<double>::infinity();
      int best_key = -1;

      int current_grid_x = start_x;
      int current_grid_y = start_y;
      if (!center_sequence.empty()) {
        const std::pair<int, int> current_grid = indexToGrid(center_sequence.back());
        current_grid_x = current_grid.first;
        current_grid_y = current_grid.second;
      }

      for (const int candidate_key : free_nodes) {
        if (globally_visited.find(candidate_key) != globally_visited.end()) {
          continue;
        }

        const int candidate_x = candidate_key % coarse_width;
        const int candidate_y = candidate_key / coarse_width;
        const std::pair<int, int> center = coarseCenter(candidate_x, candidate_y);
        const double distance = std::hypot(static_cast<double>(center.first - current_grid_x),
                                           static_cast<double>(center.second - current_grid_y));
        if (distance < best_distance) {
          best_distance = distance;
          best_key = candidate_key;
        }
      }

      if (best_key < 0) {
        break;
      }

      current_key = best_key;
    }

    std::function<void(int)> dfsVisit = [&](int node_key) {
      globally_visited.insert(node_key);
      appendCenter(node_key);

      const int coarse_x = node_key % coarse_width;
      const int coarse_y = node_key / coarse_width;

      for (const std::pair<int, int>& step : kNeighborOrder) {
        const int neighbor_x = coarse_x + step.first;
        const int neighbor_y = coarse_y + step.second;
        if (neighbor_x < 0 || neighbor_x >= coarse_width || neighbor_y < 0 || neighbor_y >= coarse_height) {
          continue;
        }

        const int neighbor_key = coarseKey(neighbor_x, neighbor_y);
        if (free_nodes.find(neighbor_key) == free_nodes.end() ||
            globally_visited.find(neighbor_key) != globally_visited.end()) {
          continue;
        }

        dfsVisit(neighbor_key);
        // 关键步骤：STC 在树边回溯时再次写入父节点中心，形成“沿生成树往返”的覆盖轨迹。
        appendCenter(node_key);
      }
    };

    dfsVisit(current_key);
  }

  std::vector<int> dense_path;
  std::pair<int, int> current = std::make_pair(start_x, start_y);

  for (const int linear_index : center_sequence) {
    const std::pair<int, int> target = indexToGrid(linear_index);
    const std::vector<int> connector = planSafePath(current.first, current.second, target.first, target.second);
    if (connector.empty()) {
      ROS_WARN("STC C++: 无法安全连接到下一个生成树节点，跳过该节点。");
      continue;
    }

    appendIndices(connector, dense_path);
    current = indexToGrid(dense_path.back());
  }

  return compressPath(dense_path);
}

double StcPlanner::heuristic(int grid_x, int grid_y, int goal_x, int goal_y) const {
  // 关键步骤：安全连接统一使用曼哈顿启发式，匹配 4 邻域搜索并避免不必要的斜穿。
  return std::abs(goal_x - grid_x) + std::abs(goal_y - grid_y);
}

double StcPlanner::computeWaypointYaw(const std::vector<int>& path_indices,
                                      std::size_t waypoint_index) const {
  if (path_indices.size() < 2U) {
    return 0.0;
  }

  int from_index = path_indices[waypoint_index];
  int to_index = path_indices[waypoint_index];

  if (waypoint_index + 1U < path_indices.size()) {
    to_index = path_indices[waypoint_index + 1U];
  } else {
    from_index = path_indices[waypoint_index - 1U];
  }

  const std::pair<int, int> from_grid = indexToGrid(from_index);
  const std::pair<int, int> to_grid = indexToGrid(to_index);
  const std::pair<double, double> from_world = gridToWorld(from_grid.first, from_grid.second);
  const std::pair<double, double> to_world = gridToWorld(to_grid.first, to_grid.second);
  return std::atan2(to_world.second - from_world.second, to_world.first - from_world.first);
}

void StcPlanner::publishPath(const std::vector<int>& path_indices) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "map";
  path_msg.poses.reserve(path_indices.size());

  for (std::size_t waypoint_index = 0; waypoint_index < path_indices.size(); ++waypoint_index) {
    const std::pair<int, int> grid = indexToGrid(path_indices[waypoint_index]);
    // 关键步骤：发布可视化路径时仍取栅格中心点，保持与其它传统算法一致的坐标定义。
    const std::pair<double, double> world = gridToWorld(grid.first, grid.second);

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = world.first;
    pose.pose.position.y = world.second;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(computeWaypointYaw(path_indices, waypoint_index));
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
}

void StcPlanner::executeCoveragePath(const std::vector<int>& path_indices) {
  for (std::size_t waypoint_index = 0; waypoint_index < path_indices.size() && ros::ok(); ++waypoint_index) {
    const std::pair<int, int> grid = indexToGrid(path_indices[waypoint_index]);
    const std::pair<double, double> world = gridToWorld(grid.first, grid.second);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.pose.position.x = world.first;
    goal.target_pose.pose.position.y = world.second;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        computeWaypointYaw(path_indices, waypoint_index));

    move_base_client_.sendGoal(goal);
    const bool finished = move_base_client_.waitForResult(ros::Duration(goal_timeout_));
    if (!finished) {
      ROS_WARN_STREAM("STC C++: 第 " << waypoint_index << " 个覆盖点等待超时，跳过该点。");
      move_base_client_.cancelGoal();
      continue;
    }

    const actionlib::SimpleClientGoalState state = move_base_client_.getState();
    if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_WARN_STREAM("STC C++: 第 " << waypoint_index << " 个覆盖点执行失败，状态为 "
                                     << state.toString() << "，跳过该点。");
      continue;
    }
  }
}

}  // namespace coverage
}  // namespace mr_traditional_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "stc_planner_cpp");
  mr_traditional_planner::coverage::StcPlanner planner;
  ros::spin();
  return 0;
}
