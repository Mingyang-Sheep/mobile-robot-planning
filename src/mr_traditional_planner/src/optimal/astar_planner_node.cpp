#include "mr_traditional_planner/optimal/astar_planner.h"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

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

AStarPlanner::AStarPlanner()
    : map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15) {
  // 最优路径类算法统一监听静态地图输入。
  map_sub_ = nh_.subscribe("/map", 1, &AStarPlanner::mapCallback, this);
  // 最优路径类算法统一监听 RViz 2D Goal 目标点输入。
  goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AStarPlanner::goalCallback, this);
  // 统一输出最优路径消息。
  path_pub_ = nh_.advertise<nav_msgs::Path>("/mr_traditional_planner/optimal_path", 1, true);
}

void AStarPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void AStarPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  latest_goal_ = msg;

  if (!latest_map_) {
    ROS_WARN("A* C++: /map 尚未收到，无法开始规划。");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != "map") {
    ROS_WARN_STREAM("A* C++: 仅支持 map 坐标系目标点，当前收到的是 " << msg->header.frame_id << "。");
    return;
  }

  double start_world_x = 0.0;
  double start_world_y = 0.0;
  if (!lookupStartPose(start_world_x, start_world_y)) {
    return;
  }

  // 关键步骤：严格按公式 index_x = int((world_x - origin_x) / resolution) 做世界坐标到栅格坐标映射。
  const std::pair<int, int> start = worldToGrid(start_world_x, start_world_y);
  // 关键步骤：终点使用完全一致的映射公式，保证与 Python 版栅格索引定义一致。
  const std::pair<int, int> goal = worldToGrid(msg->pose.position.x, msg->pose.position.y);

  if (!inBounds(start.first, start.second)) {
    ROS_WARN("A* C++: 起点超出地图范围，停止规划。");
    return;
  }

  if (!inBounds(goal.first, goal.second)) {
    ROS_WARN("A* C++: 终点超出地图范围，停止规划。");
    return;
  }

  if (isObstacle(start.first, start.second)) {
    ROS_WARN("A* C++: 起点位于障碍物膨胀区内，停止规划。");
    return;
  }

  if (isObstacle(goal.first, goal.second)) {
    ROS_WARN("A* C++: 终点位于障碍物膨胀区内，停止规划。");
    return;
  }

  const std::vector<int> path_indices = planPath(start.first, start.second, goal.first, goal.second);
  if (path_indices.empty()) {
    ROS_WARN("A* C++: 未找到可行路径。");
    return;
  }

  publishPath(path_indices);
}

void AStarPlanner::buildObstacleLookup() {
  obstacle_grid_.assign(static_cast<std::size_t>(map_width_ * map_height_), 0U);
  if (!latest_map_ || resolution_ <= 0.0) {
    return;
  }

  precomputeInflationOffsets();

  for (std::size_t linear_index = 0; linear_index < latest_map_->data.size(); ++linear_index) {
    const int occupancy = latest_map_->data[linear_index];
    // 与 Python 版严格一致：未知栅格和占用概率 >= 50 的栅格都按障碍处理。
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

void AStarPlanner::precomputeInflationOffsets() {
  inflation_offsets_.clear();
  const int inflation_radius_in_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));

  for (int offset_y = -inflation_radius_in_cells; offset_y <= inflation_radius_in_cells; ++offset_y) {
    for (int offset_x = -inflation_radius_in_cells; offset_x <= inflation_radius_in_cells; ++offset_x) {
      // 与 Python 版严格一致：用欧式距离判断该偏移是否落在机器人半径膨胀圈内。
      if (std::hypot(static_cast<double>(offset_x), static_cast<double>(offset_y)) * resolution_ <=
          robot_radius_) {
        inflation_offsets_.emplace_back(offset_x, offset_y);
      }
    }
  }
}

bool AStarPlanner::lookupStartPose(double& start_world_x, double& start_world_y) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.2));
    tf_listener_.lookupTransform("map", "base_footprint", ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM("A* C++: 获取 map -> base_footprint 失败，停止规划。" << ex.what());
    return false;
  }

  start_world_x = transform.getOrigin().x();
  start_world_y = transform.getOrigin().y();
  return true;
}

bool AStarPlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool AStarPlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }

  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

int AStarPlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> AStarPlanner::worldToGrid(double world_x, double world_y) const {
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
}

std::pair<double, double> AStarPlanner::gridToWorld(int grid_x, int grid_y) const {
  return std::make_pair(origin_x_ + (static_cast<double>(grid_x) + 0.5) * resolution_,
                        origin_y_ + (static_cast<double>(grid_y) + 0.5) * resolution_);
}

double AStarPlanner::heuristic(int grid_x, int grid_y, int goal_x, int goal_y) const {
  // 与 Python 版严格一致：启发式 H 代价统一采用欧式距离。
  return std::hypot(static_cast<double>(goal_x - grid_x), static_cast<double>(goal_y - grid_y));
}

std::vector<int> AStarPlanner::planPath(int start_x, int start_y, int goal_x, int goal_y) {
  static const double kDiagonalCost = std::sqrt(2.0);
  static const std::vector<std::tuple<int, int, double>> kMotionModel = {
      std::make_tuple(1, 0, 1.0),          std::make_tuple(0, 1, 1.0),
      std::make_tuple(-1, 0, 1.0),         std::make_tuple(0, -1, 1.0),
      std::make_tuple(1, 1, kDiagonalCost), std::make_tuple(-1, 1, kDiagonalCost),
      std::make_tuple(-1, -1, kDiagonalCost), std::make_tuple(1, -1, kDiagonalCost),
  };

  const int start_index = toIndex(start_x, start_y);
  const int goal_index = toIndex(goal_x, goal_y);
  const double start_h = heuristic(start_x, start_y, goal_x, goal_y);

  std::priority_queue<OpenItem> open_set;
  std::unordered_map<int, Node> node_lookup;
  std::unordered_map<int, bool> closed_set;
  node_lookup.reserve(static_cast<std::size_t>(map_width_ * map_height_ / 4));
  closed_set.reserve(static_cast<std::size_t>(map_width_ * map_height_ / 4));

  node_lookup.emplace(start_index, Node{start_x, start_y, 0.0, start_h, -1});
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

    const Node& current_node = current_it->second;
    if (current_item.index == goal_index) {
      return reconstructPath(goal_index, node_lookup);
    }

    closed_set.emplace(current_item.index, true);

    for (const auto& motion : kMotionModel) {
      const int next_x = current_node.x + std::get<0>(motion);
      const int next_y = current_node.y + std::get<1>(motion);
      const double step_cost = std::get<2>(motion);

      if (!inBounds(next_x, next_y) || isObstacle(next_x, next_y)) {
        continue;
      }

      const int next_index = toIndex(next_x, next_y);
      if (closed_set.find(next_index) != closed_set.end()) {
        continue;
      }

      const double tentative_g = current_node.g + step_cost;
      const double heuristic_cost = heuristic(next_x, next_y, goal_x, goal_y);

      const auto existing_it = node_lookup.find(next_index);
      if (existing_it != node_lookup.end() && tentative_g >= existing_it->second.g) {
        continue;
      }

      node_lookup[next_index] = Node{next_x, next_y, tentative_g, heuristic_cost, current_item.index};
      open_set.push(OpenItem{tentative_g + heuristic_cost, heuristic_cost, next_index});
    }
  }

  return std::vector<int>();
}

std::vector<int> AStarPlanner::reconstructPath(int goal_index,
                                               const std::unordered_map<int, Node>& node_lookup) const {
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

void AStarPlanner::publishPath(const std::vector<int>& path_indices) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = "map";

  path_msg.poses.reserve(path_indices.size());
  for (const int linear_index : path_indices) {
    const int grid_x = linear_index % map_width_;
    const int grid_y = linear_index / map_width_;

    // 关键步骤：将离散栅格索引还原为世界坐标时，取栅格中心点而不是左下角顶点。
    const std::pair<double, double> world = gridToWorld(grid_x, grid_y);

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = world.first;
    pose.pose.position.y = world.second;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
}

}  // namespace optimal
}  // namespace mr_traditional_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar_planner_cpp");
  mr_traditional_planner::optimal::AStarPlanner planner;
  ros::spin();
  return 0;
}
