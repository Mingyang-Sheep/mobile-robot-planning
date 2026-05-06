#include "mr_traditional_planner/optimal/theta_star_planner.h"

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <queue>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
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

ThetaStarPlanner::ThetaStarPlanner()
    : map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15),
      map_frame_("map"),
      robot_frame_("base_footprint") {}

void ThetaStarPlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_ = nh;
  private_nh_ = private_nh;

  std::string map_topic;
  std::string goal_topic;
  std::string path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, std::string("/map"));
  private_nh_.param<std::string>("goal_topic", goal_topic, std::string("/move_base_simple/goal"));
  private_nh_.param<std::string>("path_topic", path_topic,
                                 std::string("/mr_traditional_planner/optimal_path"));
  private_nh_.param<double>("robot_radius", robot_radius_, robot_radius_);
  private_nh_.param<std::string>("map_frame", map_frame_, map_frame_);
  private_nh_.param<std::string>("robot_frame", robot_frame_, robot_frame_);

  map_sub_ = nh_.subscribe(map_topic, 1, &ThetaStarPlanner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic, 1, &ThetaStarPlanner::goalCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);
}

void ThetaStarPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void ThetaStarPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  latest_goal_ = msg;

  if (!latest_map_) {
    ROS_WARN("Theta* C++: /map has not been received yet.");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    ROS_WARN_STREAM("Theta* C++: goal frame must be " << map_frame_ << ", got "
                                                      << msg->header.frame_id << ".");
    return;
  }

  double start_world_x = 0.0;
  double start_world_y = 0.0;
  if (!lookupStartPose(start_world_x, start_world_y)) {
    return;
  }

  const std::pair<int, int> start = worldToGrid(start_world_x, start_world_y);
  const std::pair<int, int> goal = worldToGrid(msg->pose.position.x, msg->pose.position.y);

  if (!inBounds(start.first, start.second)) {
    ROS_WARN("Theta* C++: start is outside the map.");
    return;
  }
  if (!inBounds(goal.first, goal.second)) {
    ROS_WARN("Theta* C++: goal is outside the map.");
    return;
  }
  if (isObstacle(start.first, start.second)) {
    ROS_WARN("Theta* C++: start is inside an inflated obstacle.");
    return;
  }
  if (isObstacle(goal.first, goal.second)) {
    ROS_WARN("Theta* C++: goal is inside an inflated obstacle.");
    return;
  }

  const std::vector<int> path_indices = planPath(start.first, start.second, goal.first, goal.second);
  if (path_indices.empty()) {
    ROS_WARN("Theta* C++: no path found.");
    return;
  }

  publishPath(path_indices);
}

void ThetaStarPlanner::buildObstacleLookup() {
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

void ThetaStarPlanner::precomputeInflationOffsets() {
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

bool ThetaStarPlanner::lookupStartPose(double& start_world_x, double& start_world_y) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.2));
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM("Theta* C++: failed to lookup " << map_frame_ << " -> " << robot_frame_
                                                    << ": " << ex.what());
    return false;
  }

  start_world_x = transform.getOrigin().x();
  start_world_y = transform.getOrigin().y();
  return true;
}

bool ThetaStarPlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool ThetaStarPlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }
  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

int ThetaStarPlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> ThetaStarPlanner::indexToGrid(int linear_index) const {
  return std::make_pair(linear_index % map_width_, linear_index / map_width_);
}

std::pair<int, int> ThetaStarPlanner::worldToGrid(double world_x, double world_y) const {
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
}

std::pair<double, double> ThetaStarPlanner::gridToWorld(int grid_x, int grid_y) const {
  return std::make_pair(origin_x_ + (static_cast<double>(grid_x) + 0.5) * resolution_,
                        origin_y_ + (static_cast<double>(grid_y) + 0.5) * resolution_);
}

double ThetaStarPlanner::heuristic(int grid_x, int grid_y, int goal_x, int goal_y) const {
  return std::hypot(static_cast<double>(goal_x - grid_x), static_cast<double>(goal_y - grid_y));
}

bool ThetaStarPlanner::lineOfSight(int from_index, int to_x, int to_y) const {
  std::pair<int, int> from = indexToGrid(from_index);
  int x0 = from.first;
  int y0 = from.second;
  const int x1 = to_x;
  const int y1 = to_y;
  const int dx = std::abs(x1 - x0);
  const int dy = std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx - dy;

  while (true) {
    if (isObstacle(x0, y0)) {
      return false;
    }
    if (x0 == x1 && y0 == y1) {
      return true;
    }

    const int twice_error = 2 * error;
    if (twice_error > -dy) {
      error -= dy;
      x0 += sx;
    }
    if (twice_error < dx) {
      error += dx;
      y0 += sy;
    }
  }
}

std::vector<int> ThetaStarPlanner::planPath(int start_x, int start_y, int goal_x, int goal_y) {
  static const double kDiagonalCost = std::sqrt(2.0);
  static const std::vector<std::tuple<int, int, double>> kMotionModel = {
      std::make_tuple(1, 0, 1.0),           std::make_tuple(0, 1, 1.0),
      std::make_tuple(-1, 0, 1.0),          std::make_tuple(0, -1, 1.0),
      std::make_tuple(1, 1, kDiagonalCost), std::make_tuple(-1, 1, kDiagonalCost),
      std::make_tuple(-1, -1, kDiagonalCost), std::make_tuple(1, -1, kDiagonalCost),
  };

  const int start_index = toIndex(start_x, start_y);
  const int goal_index = toIndex(goal_x, goal_y);
  const double start_h = heuristic(start_x, start_y, goal_x, goal_y);

  std::priority_queue<OpenItem> open_set;
  std::unordered_map<int, ThetaStarNode> node_lookup;
  std::unordered_set<int> closed_set;
  node_lookup.reserve(static_cast<std::size_t>(map_width_ * map_height_ / 4));
  closed_set.reserve(static_cast<std::size_t>(map_width_ * map_height_ / 4));

  node_lookup.emplace(start_index, ThetaStarNode{start_x, start_y, 0.0, start_h, -1});
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

    const ThetaStarNode current_node = current_it->second;
    if (current_item.index == goal_index) {
      return reconstructPath(goal_index, node_lookup);
    }

    closed_set.insert(current_item.index);

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

      double tentative_g = current_node.g + step_cost;
      int parent_index = current_item.index;
      if (current_node.parent_index >= 0 && lineOfSight(current_node.parent_index, next_x, next_y)) {
        const auto parent_it = node_lookup.find(current_node.parent_index);
        if (parent_it != node_lookup.end()) {
          const ThetaStarNode& parent_node = parent_it->second;
          tentative_g =
              parent_node.g + std::hypot(static_cast<double>(next_x - parent_node.x),
                                         static_cast<double>(next_y - parent_node.y));
          parent_index = current_node.parent_index;
        }
      }

      const auto existing_it = node_lookup.find(next_index);
      if (existing_it != node_lookup.end() && tentative_g >= existing_it->second.g) {
        continue;
      }

      const double heuristic_cost = heuristic(next_x, next_y, goal_x, goal_y);
      node_lookup[next_index] = ThetaStarNode{next_x, next_y, tentative_g, heuristic_cost,
                                              parent_index};
      open_set.push(OpenItem{tentative_g + heuristic_cost, heuristic_cost, next_index});
    }
  }

  return std::vector<int>();
}

std::vector<int> ThetaStarPlanner::reconstructPath(
    int goal_index, const std::unordered_map<int, ThetaStarNode>& node_lookup) const {
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

void ThetaStarPlanner::publishPath(const std::vector<int>& path_indices) const {
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
}

}  // namespace optimal
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::optimal::ThetaStarPlanner,
                       mr_traditional_planner::PlannerPlugin)
