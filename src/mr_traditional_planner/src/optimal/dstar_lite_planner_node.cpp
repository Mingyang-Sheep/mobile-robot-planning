#include "mr_traditional_planner/optimal/dstar_lite_planner.h"

#include "mr_traditional_planner/debug_path_tools.h"

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

DStarLitePlanner::DStarLitePlanner()
    : map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15),
      map_frame_("map"),
      robot_frame_("base_footprint"),
      search_start_index_(-1),
      search_goal_index_(-1),
      km_(0.0) {}

void DStarLitePlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_ = nh;
  private_nh_ = private_nh;

  std::string map_topic;
  std::string goal_topic;
  private_nh_.param<std::string>("map_topic", map_topic, std::string("/map"));
  private_nh_.param<std::string>("goal_topic", goal_topic, std::string("/move_base_simple/goal"));
  private_nh_.param<std::string>("path_topic", path_topic_,
                                 std::string("/mr_traditional_planner/debug_optimal_path"));
  private_nh_.param<double>("robot_radius", robot_radius_, robot_radius_);
  private_nh_.param<std::string>("map_frame", map_frame_, map_frame_);
  private_nh_.param<std::string>("robot_frame", robot_frame_, robot_frame_);

  map_sub_ = nh_.subscribe(map_topic, 1, &DStarLitePlanner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic, 1, &DStarLitePlanner::goalCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  publishFailure("startup_clear");
}

void DStarLitePlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void DStarLitePlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  latest_goal_ = msg;

  if (!latest_map_) {
    ROS_WARN("D* Lite C++: /map 尚未收到，无法开始规划。");
    publishFailure("map_missing");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    ROS_WARN_STREAM("D* Lite C++: 仅支持 " << map_frame_ << " 坐标系目标点，当前收到的是 "
                                          << msg->header.frame_id << "。");
    publishFailure("goal_frame_mismatch");
    return;
  }

  double start_world_x = 0.0;
  double start_world_y = 0.0;
  if (!lookupStartPose(start_world_x, start_world_y)) {
    publishFailure("tf_lookup_failed");
    return;
  }

  const std::pair<int, int> start = worldToGrid(start_world_x, start_world_y);
  const std::pair<int, int> goal = worldToGrid(msg->pose.position.x, msg->pose.position.y);

  if (!inBounds(start.first, start.second)) {
    ROS_WARN("D* Lite C++: 起点超出地图范围，停止规划。");
    publishFailure("start_out_of_bounds");
    return;
  }

  if (!inBounds(goal.first, goal.second)) {
    ROS_WARN("D* Lite C++: 终点超出地图范围，停止规划。");
    publishFailure("goal_out_of_bounds");
    return;
  }

  if (isObstacle(start.first, start.second)) {
    ROS_WARN("D* Lite C++: 起点位于障碍物膨胀区内，停止规划。");
    publishFailure("start_blocked");
    return;
  }

  if (isObstacle(goal.first, goal.second)) {
    ROS_WARN("D* Lite C++: 终点位于障碍物膨胀区内，停止规划。");
    publishFailure("goal_blocked");
    return;
  }

  const std::vector<int> path_indices = planPath(start.first, start.second, goal.first, goal.second);
  if (path_indices.empty()) {
    ROS_WARN("D* Lite C++: 未找到可行路径。");
    publishFailure("no_path");
    return;
  }

  publishPath(path_indices);
}

void DStarLitePlanner::buildObstacleLookup() {
  obstacle_grid_.assign(static_cast<std::size_t>(map_width_ * map_height_), 0U);
  if (!latest_map_ || resolution_ <= 0.0) {
    return;
  }

  precomputeInflationOffsets();

  for (std::size_t linear_index = 0; linear_index < latest_map_->data.size(); ++linear_index) {
    const int occupancy = latest_map_->data[linear_index];
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

void DStarLitePlanner::precomputeInflationOffsets() {
  inflation_offsets_.clear();
  const int inflation_radius_in_cells = static_cast<int>(std::ceil(robot_radius_ / resolution_));

  for (int offset_y = -inflation_radius_in_cells; offset_y <= inflation_radius_in_cells; ++offset_y) {
    for (int offset_x = -inflation_radius_in_cells; offset_x <= inflation_radius_in_cells; ++offset_x) {
      if (std::hypot(static_cast<double>(offset_x), static_cast<double>(offset_y)) * resolution_ <=
          robot_radius_) {
        inflation_offsets_.emplace_back(offset_x, offset_y);
      }
    }
  }
}

bool DStarLitePlanner::lookupStartPose(double& start_world_x, double& start_world_y) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.2));
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM("D* Lite C++: 获取 " << map_frame_ << " -> " << robot_frame_
                                         << " 失败，停止规划。" << ex.what());
    return false;
  }

  start_world_x = transform.getOrigin().x();
  start_world_y = transform.getOrigin().y();
  return true;
}

bool DStarLitePlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool DStarLitePlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }
  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

int DStarLitePlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> DStarLitePlanner::indexToGrid(int linear_index) const {
  return std::make_pair(linear_index % map_width_, linear_index / map_width_);
}

std::pair<int, int> DStarLitePlanner::worldToGrid(double world_x, double world_y) const {
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
}

std::pair<double, double> DStarLitePlanner::gridToWorld(int grid_x, int grid_y) const {
  return std::make_pair(origin_x_ + (static_cast<double>(grid_x) + 0.5) * resolution_,
                        origin_y_ + (static_cast<double>(grid_y) + 0.5) * resolution_);
}

std::vector<int> DStarLitePlanner::planPath(int start_x, int start_y, int goal_x, int goal_y) {
  const int map_size = map_width_ * map_height_;
  search_start_index_ = toIndex(start_x, start_y);
  search_goal_index_ = toIndex(goal_x, goal_y);
  if (search_start_index_ == search_goal_index_) {
    return std::vector<int>(1, search_start_index_);
  }

  g_values_.assign(static_cast<std::size_t>(map_size), std::numeric_limits<double>::infinity());
  rhs_values_.assign(static_cast<std::size_t>(map_size), std::numeric_limits<double>::infinity());
  open_queue_ = std::priority_queue<OpenItem>();
  km_ = 0.0;

  rhs_values_[static_cast<std::size_t>(search_goal_index_)] = 0.0;
  open_queue_.push(OpenItem{calculateKey(search_goal_index_), search_goal_index_});

  if (!computeShortestPath()) {
    return std::vector<int>();
  }

  std::vector<int> path_indices;
  path_indices.reserve(static_cast<std::size_t>(map_width_ + map_height_));
  int current_index = search_start_index_;
  for (int step_count = 0; step_count < map_size; ++step_count) {
    path_indices.push_back(current_index);
    if (current_index == search_goal_index_) {
      return path_indices;
    }

    double best_cost = std::numeric_limits<double>::infinity();
    int best_index = -1;
    for (const int neighbor_index : neighbors(current_index)) {
      const double candidate_cost =
          moveCost(current_index, neighbor_index) + g_values_[static_cast<std::size_t>(neighbor_index)];
      if (candidate_cost < best_cost) {
        best_cost = candidate_cost;
        best_index = neighbor_index;
      }
    }

    if (best_index < 0 || !std::isfinite(best_cost)) {
      return std::vector<int>();
    }
    current_index = best_index;
  }

  return std::vector<int>();
}

std::vector<int> DStarLitePlanner::neighbors(int linear_index) const {
  std::vector<int> neighbor_indices;
  neighbor_indices.reserve(8U);
  const std::pair<int, int> grid = indexToGrid(linear_index);

  for (int offset_y = -1; offset_y <= 1; ++offset_y) {
    for (int offset_x = -1; offset_x <= 1; ++offset_x) {
      if (offset_x == 0 && offset_y == 0) {
        continue;
      }

      const int next_x = grid.first + offset_x;
      const int next_y = grid.second + offset_y;
      if (inBounds(next_x, next_y)) {
        neighbor_indices.push_back(toIndex(next_x, next_y));
      }
    }
  }

  return neighbor_indices;
}

double DStarLitePlanner::moveCost(int from_index, int to_index) const {
  const std::pair<int, int> from_grid = indexToGrid(from_index);
  const std::pair<int, int> to_grid = indexToGrid(to_index);
  if (isObstacle(from_grid.first, from_grid.second) || isObstacle(to_grid.first, to_grid.second)) {
    return std::numeric_limits<double>::infinity();
  }

  return std::hypot(static_cast<double>(from_grid.first - to_grid.first),
                    static_cast<double>(from_grid.second - to_grid.second));
}

double DStarLitePlanner::heuristic(int from_index, int to_index) const {
  const std::pair<int, int> from_grid = indexToGrid(from_index);
  const std::pair<int, int> to_grid = indexToGrid(to_index);
  const double dx = std::abs(from_grid.first - to_grid.first);
  const double dy = std::abs(from_grid.second - to_grid.second);
  const double diagonal = std::min(dx, dy);
  const double straight = std::max(dx, dy) - diagonal;
  return std::sqrt(2.0) * diagonal + straight;
}

DStarLitePlanner::Key DStarLitePlanner::calculateKey(int linear_index) const {
  const double min_value =
      std::min(g_values_[static_cast<std::size_t>(linear_index)],
               rhs_values_[static_cast<std::size_t>(linear_index)]);
  return Key{min_value + heuristic(search_start_index_, linear_index) + km_, min_value};
}

bool DStarLitePlanner::compareKeys(const Key& lhs, const Key& rhs) const {
  if (!nearlyEqual(lhs.first, rhs.first)) {
    return lhs.first < rhs.first;
  }
  return lhs.second < rhs.second && !nearlyEqual(lhs.second, rhs.second);
}

bool DStarLitePlanner::nearlyEqual(double lhs, double rhs) const {
  return std::fabs(lhs - rhs) <= 1.0e-9;
}

void DStarLitePlanner::updateVertex(int linear_index) {
  if (linear_index != search_goal_index_) {
    double best_rhs = std::numeric_limits<double>::infinity();
    for (const int successor_index : neighbors(linear_index)) {
      const double candidate_rhs =
          moveCost(linear_index, successor_index) +
          g_values_[static_cast<std::size_t>(successor_index)];
      best_rhs = std::min(best_rhs, candidate_rhs);
    }
    rhs_values_[static_cast<std::size_t>(linear_index)] = best_rhs;
  }

  if (!nearlyEqual(g_values_[static_cast<std::size_t>(linear_index)],
                   rhs_values_[static_cast<std::size_t>(linear_index)])) {
    open_queue_.push(OpenItem{calculateKey(linear_index), linear_index});
  }
}

bool DStarLitePlanner::computeShortestPath() {
  const int map_size = map_width_ * map_height_;
  const int max_iterations = std::max(1, map_size * 32);

  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    if (open_queue_.empty() &&
        nearlyEqual(rhs_values_[static_cast<std::size_t>(search_start_index_)],
                    g_values_[static_cast<std::size_t>(search_start_index_)])) {
      return true;
    }

    if (open_queue_.empty()) {
      return false;
    }

    const OpenItem current_item = open_queue_.top();
    const Key start_key = calculateKey(search_start_index_);
    if (!compareKeys(current_item.key, start_key) &&
        nearlyEqual(rhs_values_[static_cast<std::size_t>(search_start_index_)],
                    g_values_[static_cast<std::size_t>(search_start_index_)])) {
      return true;
    }

    open_queue_.pop();
    const int current_index = current_item.index;
    const Key current_key = calculateKey(current_index);
    if (compareKeys(current_key, current_item.key)) {
      continue;
    }

    if (compareKeys(current_item.key, current_key)) {
      open_queue_.push(OpenItem{current_key, current_index});
      continue;
    }

    if (g_values_[static_cast<std::size_t>(current_index)] >
        rhs_values_[static_cast<std::size_t>(current_index)]) {
      g_values_[static_cast<std::size_t>(current_index)] =
          rhs_values_[static_cast<std::size_t>(current_index)];
      for (const int predecessor_index : neighbors(current_index)) {
        updateVertex(predecessor_index);
      }
    } else {
      g_values_[static_cast<std::size_t>(current_index)] = std::numeric_limits<double>::infinity();
      updateVertex(current_index);
      for (const int predecessor_index : neighbors(current_index)) {
        updateVertex(predecessor_index);
      }
    }
  }

  ROS_WARN("D* Lite C++: 搜索迭代次数超过上限，停止规划。");
  return false;
}

void DStarLitePlanner::publishPath(const std::vector<int>& path_indices) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(path_indices.size());
  for (const int linear_index : path_indices) {
    const std::pair<int, int> grid = indexToGrid(linear_index);
    const std::pair<double, double> world = gridToWorld(grid.first, grid.second);

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = world.first;
    pose.pose.position.y = world.second;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
  logDebugPathSuccess("dstar_lite", "cpp", path_topic_, path_msg.poses.size());
}

void DStarLitePlanner::publishFailure(const std::string& reason) const {
  publishEmptyDebugPath(path_pub_, map_frame_, "dstar_lite", "cpp", path_topic_, reason);
}

}  // namespace optimal
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::optimal::DStarLitePlanner,
                       mr_traditional_planner::PlannerPlugin)
