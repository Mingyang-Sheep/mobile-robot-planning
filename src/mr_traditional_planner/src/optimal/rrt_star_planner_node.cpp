#include "mr_traditional_planner/optimal/rrt_star_planner.h"

#include "mr_traditional_planner/debug_path_tools.h"

#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

RRTStarPlanner::RRTStarPlanner()
    : map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15),
      map_frame_("map"),
      robot_frame_("base_footprint"),
      max_iterations_(1000),
      expand_distance_(0.5),
      path_resolution_(0.05),
      goal_sample_rate_(20),
      connect_circle_distance_(2.0),
      search_until_max_iter_(false),
      random_engine_(0U) {}

void RRTStarPlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_ = nh;
  private_nh_ = private_nh;

  std::string map_topic;
  std::string goal_topic;
  int random_seed = 0;
  private_nh_.param<std::string>("map_topic", map_topic, std::string("/map"));
  private_nh_.param<std::string>("goal_topic", goal_topic, std::string("/move_base_simple/goal"));
  private_nh_.param<std::string>("path_topic", path_topic_,
                                 std::string("/mr_traditional_planner/debug_optimal_path"));
  private_nh_.param<double>("robot_radius", robot_radius_, robot_radius_);
  private_nh_.param<std::string>("map_frame", map_frame_, map_frame_);
  private_nh_.param<std::string>("robot_frame", robot_frame_, robot_frame_);
  private_nh_.param<int>("max_iterations", max_iterations_, max_iterations_);
  private_nh_.param<double>("expand_distance", expand_distance_, expand_distance_);
  private_nh_.param<double>("path_resolution", path_resolution_, path_resolution_);
  private_nh_.param<int>("goal_sample_rate", goal_sample_rate_, goal_sample_rate_);
  private_nh_.param<double>("connect_circle_distance", connect_circle_distance_,
                            connect_circle_distance_);
  private_nh_.param<bool>("search_until_max_iter", search_until_max_iter_,
                          search_until_max_iter_);
  private_nh_.param<int>("random_seed", random_seed, random_seed);

  max_iterations_ = std::max(1, max_iterations_);
  expand_distance_ = std::max(1.0e-3, expand_distance_);
  path_resolution_ = std::max(1.0e-3, path_resolution_);
  goal_sample_rate_ = std::max(0, std::min(100, goal_sample_rate_));
  connect_circle_distance_ = std::max(1.0e-3, connect_circle_distance_);
  random_engine_.seed(static_cast<std::mt19937::result_type>(random_seed));

  map_sub_ = nh_.subscribe(map_topic, 1, &RRTStarPlanner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic, 1, &RRTStarPlanner::goalCallback, this);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic_, 1, true);
  publishFailure("startup_clear");
}

void RRTStarPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void RRTStarPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  latest_goal_ = msg;

  if (!latest_map_) {
    ROS_WARN("RRT* C++: /map has not been received yet.");
    publishFailure("map_missing");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    ROS_WARN_STREAM("RRT* C++: goal frame must be " << map_frame_ << ", got "
                                                    << msg->header.frame_id << ".");
    publishFailure("goal_frame_mismatch");
    return;
  }

  double start_world_x = 0.0;
  double start_world_y = 0.0;
  if (!lookupStartPose(start_world_x, start_world_y)) {
    publishFailure("tf_lookup_failed");
    return;
  }

  const double goal_world_x = msg->pose.position.x;
  const double goal_world_y = msg->pose.position.y;
  if (!isWorldPointFree(start_world_x, start_world_y)) {
    ROS_WARN("RRT* C++: start is outside the map or inside an inflated obstacle.");
    publishFailure("start_blocked");
    return;
  }
  if (!isWorldPointFree(goal_world_x, goal_world_y)) {
    ROS_WARN("RRT* C++: goal is outside the map or inside an inflated obstacle.");
    publishFailure("goal_blocked");
    return;
  }

  const std::vector<std::pair<double, double>> path_points =
      planPath(start_world_x, start_world_y, goal_world_x, goal_world_y);
  if (path_points.empty()) {
    ROS_WARN("RRT* C++: no path found.");
    publishFailure("no_path");
    return;
  }

  publishPath(path_points);
}

void RRTStarPlanner::buildObstacleLookup() {
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

void RRTStarPlanner::precomputeInflationOffsets() {
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

bool RRTStarPlanner::lookupStartPose(double& start_world_x, double& start_world_y) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.2));
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM("RRT* C++: failed to lookup " << map_frame_ << " -> " << robot_frame_
                                                  << ": " << ex.what());
    return false;
  }

  start_world_x = transform.getOrigin().x();
  start_world_y = transform.getOrigin().y();
  return true;
}

bool RRTStarPlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool RRTStarPlanner::worldInBounds(double world_x, double world_y) const {
  return world_x >= origin_x_ && world_y >= origin_y_ &&
         world_x < origin_x_ + static_cast<double>(map_width_) * resolution_ &&
         world_y < origin_y_ + static_cast<double>(map_height_) * resolution_;
}

bool RRTStarPlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }
  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

bool RRTStarPlanner::isWorldPointFree(double world_x, double world_y) const {
  if (!worldInBounds(world_x, world_y)) {
    return false;
  }
  const std::pair<int, int> grid = worldToGrid(world_x, world_y);
  return !isObstacle(grid.first, grid.second);
}

bool RRTStarPlanner::isSegmentFree(double from_x, double from_y, double to_x, double to_y) const {
  const double segment_length = distance(from_x, from_y, to_x, to_y);
  const double collision_step = std::max(1.0e-6, std::min(path_resolution_, resolution_ * 0.5));
  const int steps = std::max(1, static_cast<int>(std::ceil(segment_length / collision_step)));

  for (int step = 0; step <= steps; ++step) {
    const double ratio = static_cast<double>(step) / static_cast<double>(steps);
    const double check_x = from_x + (to_x - from_x) * ratio;
    const double check_y = from_y + (to_y - from_y) * ratio;
    if (!isWorldPointFree(check_x, check_y)) {
      return false;
    }
  }

  return true;
}

int RRTStarPlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> RRTStarPlanner::worldToGrid(double world_x, double world_y) const {
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
}

double RRTStarPlanner::distance(double from_x, double from_y, double to_x, double to_y) const {
  return std::hypot(to_x - from_x, to_y - from_y);
}

RRTStarNode RRTStarPlanner::sampleNode(double goal_x, double goal_y) {
  std::uniform_int_distribution<int> goal_distribution(0, 100);
  if (goal_distribution(random_engine_) < goal_sample_rate_) {
    return RRTStarNode{goal_x, goal_y, 0.0, -1};
  }

  std::uniform_real_distribution<double> x_distribution(
      origin_x_, origin_x_ + static_cast<double>(map_width_) * resolution_);
  std::uniform_real_distribution<double> y_distribution(
      origin_y_, origin_y_ + static_cast<double>(map_height_) * resolution_);
  return RRTStarNode{x_distribution(random_engine_), y_distribution(random_engine_), 0.0, -1};
}

RRTStarNode RRTStarPlanner::steer(const RRTStarNode& from_node,
                                  const RRTStarNode& to_node) const {
  const double node_distance = distance(from_node.x, from_node.y, to_node.x, to_node.y);
  if (node_distance <= expand_distance_) {
    return RRTStarNode{to_node.x, to_node.y, 0.0, -1};
  }

  const double theta = std::atan2(to_node.y - from_node.y, to_node.x - from_node.x);
  return RRTStarNode{from_node.x + expand_distance_ * std::cos(theta),
                     from_node.y + expand_distance_ * std::sin(theta), 0.0, -1};
}

int RRTStarPlanner::nearestNodeIndex(const std::vector<RRTStarNode>& nodes,
                                     const RRTStarNode& target_node) const {
  int best_index = 0;
  double best_distance = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double squared_distance =
        (nodes[index].x - target_node.x) * (nodes[index].x - target_node.x) +
        (nodes[index].y - target_node.y) * (nodes[index].y - target_node.y);
    if (squared_distance < best_distance) {
      best_distance = squared_distance;
      best_index = static_cast<int>(index);
    }
  }

  return best_index;
}

std::vector<int> RRTStarPlanner::nearNodeIndices(const std::vector<RRTStarNode>& nodes,
                                                 const RRTStarNode& new_node) const {
  const double n_nodes = static_cast<double>(nodes.size() + 1U);
  double radius = connect_circle_distance_;
  if (n_nodes > 1.0) {
    radius *= std::sqrt(std::log(n_nodes) / n_nodes);
  }
  radius = std::min(radius, expand_distance_);
  const double radius_squared = radius * radius;

  std::vector<int> near_indices;
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const double squared_distance =
        (nodes[index].x - new_node.x) * (nodes[index].x - new_node.x) +
        (nodes[index].y - new_node.y) * (nodes[index].y - new_node.y);
    if (squared_distance <= radius_squared) {
      near_indices.push_back(static_cast<int>(index));
    }
  }

  return near_indices;
}

void RRTStarPlanner::chooseParent(const std::vector<RRTStarNode>& nodes,
                                  const std::vector<int>& near_indices,
                                  RRTStarNode& new_node) const {
  for (const int near_index : near_indices) {
    const RRTStarNode& near_node = nodes[static_cast<std::size_t>(near_index)];
    if (!isSegmentFree(near_node.x, near_node.y, new_node.x, new_node.y)) {
      continue;
    }

    const double candidate_cost =
        near_node.cost + distance(near_node.x, near_node.y, new_node.x, new_node.y);
    if (candidate_cost < new_node.cost) {
      new_node.cost = candidate_cost;
      new_node.parent_index = near_index;
    }
  }
}

void RRTStarPlanner::rewire(std::vector<RRTStarNode>& nodes, int new_index,
                            const std::vector<int>& near_indices) const {
  const RRTStarNode& new_node = nodes[static_cast<std::size_t>(new_index)];

  for (const int near_index : near_indices) {
    if (near_index == new_node.parent_index) {
      continue;
    }

    RRTStarNode& near_node = nodes[static_cast<std::size_t>(near_index)];
    if (!isSegmentFree(new_node.x, new_node.y, near_node.x, near_node.y)) {
      continue;
    }

    const double candidate_cost =
        new_node.cost + distance(new_node.x, new_node.y, near_node.x, near_node.y);
    if (candidate_cost < near_node.cost) {
      near_node.cost = candidate_cost;
      near_node.parent_index = new_index;
      propagateCostToChildren(nodes, near_index);
    }
  }
}

void RRTStarPlanner::propagateCostToChildren(std::vector<RRTStarNode>& nodes,
                                             int parent_index) const {
  const RRTStarNode& parent_node = nodes[static_cast<std::size_t>(parent_index)];

  for (std::size_t index = 0; index < nodes.size(); ++index) {
    RRTStarNode& node = nodes[index];
    if (node.parent_index != parent_index) {
      continue;
    }

    node.cost = parent_node.cost + distance(parent_node.x, parent_node.y, node.x, node.y);
    propagateCostToChildren(nodes, static_cast<int>(index));
  }
}

int RRTStarPlanner::bestGoalNodeIndex(const std::vector<RRTStarNode>& nodes, double goal_x,
                                      double goal_y) const {
  int best_index = -1;
  double best_cost = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < nodes.size(); ++index) {
    const RRTStarNode& node = nodes[index];
    const double goal_distance = distance(node.x, node.y, goal_x, goal_y);
    if (goal_distance > expand_distance_) {
      continue;
    }
    if (!isSegmentFree(node.x, node.y, goal_x, goal_y)) {
      continue;
    }

    const double candidate_cost = node.cost + goal_distance;
    if (candidate_cost < best_cost) {
      best_cost = candidate_cost;
      best_index = static_cast<int>(index);
    }
  }

  return best_index;
}

std::vector<std::pair<double, double>> RRTStarPlanner::reconstructPath(
    const std::vector<RRTStarNode>& nodes, int goal_parent_index, double goal_x, double goal_y) const {
  std::vector<std::pair<double, double>> path_points;
  path_points.emplace_back(goal_x, goal_y);

  int current_index = goal_parent_index;
  while (current_index >= 0) {
    const RRTStarNode& node = nodes[static_cast<std::size_t>(current_index)];
    path_points.emplace_back(node.x, node.y);
    current_index = node.parent_index;
  }

  std::reverse(path_points.begin(), path_points.end());
  return path_points;
}

std::vector<std::pair<double, double>> RRTStarPlanner::planPath(double start_x, double start_y,
                                                                double goal_x, double goal_y) {
  if (distance(start_x, start_y, goal_x, goal_y) <= path_resolution_ &&
      isSegmentFree(start_x, start_y, goal_x, goal_y)) {
    return std::vector<std::pair<double, double>>{{start_x, start_y}, {goal_x, goal_y}};
  }

  std::vector<RRTStarNode> nodes;
  nodes.reserve(static_cast<std::size_t>(max_iterations_) + 1U);
  nodes.push_back(RRTStarNode{start_x, start_y, 0.0, -1});

  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    const RRTStarNode sampled_node = sampleNode(goal_x, goal_y);
    const int nearest_index = nearestNodeIndex(nodes, sampled_node);
    const RRTStarNode& nearest_node = nodes[static_cast<std::size_t>(nearest_index)];
    RRTStarNode new_node = steer(nearest_node, sampled_node);

    if (!worldInBounds(new_node.x, new_node.y) ||
        !isSegmentFree(nearest_node.x, nearest_node.y, new_node.x, new_node.y)) {
      continue;
    }

    new_node.parent_index = nearest_index;
    new_node.cost =
        nearest_node.cost + distance(nearest_node.x, nearest_node.y, new_node.x, new_node.y);

    const std::vector<int> near_indices = nearNodeIndices(nodes, new_node);
    chooseParent(nodes, near_indices, new_node);

    nodes.push_back(new_node);
    const int new_index = static_cast<int>(nodes.size()) - 1;
    rewire(nodes, new_index, near_indices);

    if (!search_until_max_iter_) {
      const int best_index = bestGoalNodeIndex(nodes, goal_x, goal_y);
      if (best_index >= 0) {
        return reconstructPath(nodes, best_index, goal_x, goal_y);
      }
    }
  }

  const int best_index = bestGoalNodeIndex(nodes, goal_x, goal_y);
  if (best_index >= 0) {
    return reconstructPath(nodes, best_index, goal_x, goal_y);
  }

  return std::vector<std::pair<double, double>>();
}

void RRTStarPlanner::publishPath(const std::vector<std::pair<double, double>>& path_points) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(path_points.size());
  for (const std::pair<double, double>& point : path_points) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
  logDebugPathSuccess("rrt_star", "cpp", path_topic_, path_msg.poses.size());
}

void RRTStarPlanner::publishFailure(const std::string& reason) const {
  publishEmptyDebugPath(path_pub_, map_frame_, "rrt_star", "cpp", path_topic_, reason);
}

}  // namespace optimal
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::optimal::RRTStarPlanner,
                       mr_traditional_planner::PlannerPlugin)
