#include "mr_traditional_planner/optimal/cubic_spline_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

namespace {

class CubicSpline1D {
 public:
  CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y) : x_(x), a_(y) {
    const std::size_t n = x_.size();
    b_.assign(n > 1U ? n - 1U : 0U, 0.0);
    c_.assign(n, 0.0);
    d_.assign(n > 1U ? n - 1U : 0U, 0.0);
    if (n < 2U) {
      return;
    }

    std::vector<double> h(n - 1U, 0.0);
    for (std::size_t i = 0; i + 1U < n; ++i) {
      h[i] = x_[i + 1U] - x_[i];
    }

    std::vector<double> alpha(n, 0.0);
    for (std::size_t i = 1U; i + 1U < n; ++i) {
      alpha[i] = 3.0 * (a_[i + 1U] - a_[i]) / h[i] -
                 3.0 * (a_[i] - a_[i - 1U]) / h[i - 1U];
    }

    std::vector<double> l(n, 1.0);
    std::vector<double> mu(n, 0.0);
    std::vector<double> z(n, 0.0);
    for (std::size_t i = 1U; i + 1U < n; ++i) {
      l[i] = 2.0 * (x_[i + 1U] - x_[i - 1U]) - h[i - 1U] * mu[i - 1U];
      mu[i] = h[i] / l[i];
      z[i] = (alpha[i] - h[i - 1U] * z[i - 1U]) / l[i];
    }

    for (std::size_t j = n - 1U; j-- > 0U;) {
      c_[j] = z[j] - mu[j] * c_[j + 1U];
      b_[j] = (a_[j + 1U] - a_[j]) / h[j] - h[j] * (c_[j + 1U] + 2.0 * c_[j]) / 3.0;
      d_[j] = (c_[j + 1U] - c_[j]) / (3.0 * h[j]);
    }
  }

  double calcPosition(double x) const {
    const std::size_t i = searchIndex(x);
    const double dx = x - x_[i];
    return a_[i] + b_[i] * dx + c_[i] * dx * dx + d_[i] * dx * dx * dx;
  }

  double calcFirstDerivative(double x) const {
    const std::size_t i = searchIndex(x);
    const double dx = x - x_[i];
    return b_[i] + 2.0 * c_[i] * dx + 3.0 * d_[i] * dx * dx;
  }

 private:
  std::size_t searchIndex(double x) const {
    std::vector<double>::const_iterator it = std::upper_bound(x_.begin(), x_.end(), x);
    if (it == x_.begin()) {
      return 0U;
    }
    const std::size_t index = static_cast<std::size_t>(std::distance(x_.begin(), it) - 1);
    return std::min(index, b_.size() - 1U);
  }

  std::vector<double> x_;
  std::vector<double> a_;
  std::vector<double> b_;
  std::vector<double> c_;
  std::vector<double> d_;
};

class CubicSpline2D {
 public:
  explicit CubicSpline2D(const std::vector<std::pair<double, double>>& points)
      : s_(calcS(points)), sx_(s_, extractX(points)), sy_(s_, extractY(points)) {}

  double maxS() const {
    return s_.empty() ? 0.0 : s_.back();
  }

  std::pair<double, double> calcPosition(double s) const {
    return std::make_pair(sx_.calcPosition(s), sy_.calcPosition(s));
  }

  double calcYaw(double s) const {
    return std::atan2(sy_.calcFirstDerivative(s), sx_.calcFirstDerivative(s));
  }

 private:
  static std::vector<double> calcS(const std::vector<std::pair<double, double>>& points) {
    std::vector<double> s;
    s.reserve(points.size());
    s.push_back(0.0);
    for (std::size_t i = 1U; i < points.size(); ++i) {
      const double ds = std::hypot(points[i].first - points[i - 1U].first,
                                   points[i].second - points[i - 1U].second);
      s.push_back(s.back() + ds);
    }
    return s;
  }

  static std::vector<double> extractX(const std::vector<std::pair<double, double>>& points) {
    std::vector<double> result;
    result.reserve(points.size());
    for (const std::pair<double, double>& point : points) {
      result.push_back(point.first);
    }
    return result;
  }

  static std::vector<double> extractY(const std::vector<std::pair<double, double>>& points) {
    std::vector<double> result;
    result.reserve(points.size());
    for (const std::pair<double, double>& point : points) {
      result.push_back(point.second);
    }
    return result;
  }

  std::vector<double> s_;
  CubicSpline1D sx_;
  CubicSpline1D sy_;
};

std::vector<std::pair<double, double>> deduplicateAnchors(
    const std::vector<std::pair<double, double>>& anchors) {
  std::vector<std::pair<double, double>> result;
  result.reserve(anchors.size());
  for (const std::pair<double, double>& point : anchors) {
    if (!result.empty() &&
        std::hypot(point.first - result.back().first, point.second - result.back().second) <
            1.0e-6) {
      continue;
    }
    result.push_back(point);
  }
  return result;
}

}  // namespace

CubicSplinePlanner::CubicSplinePlanner()
    : map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15),
      spline_resolution_(0.05),
      control_point_ratio_(0.35),
      collision_check_(true),
      map_frame_("map"),
      robot_frame_("base_footprint") {}

void CubicSplinePlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_ = nh;
  private_nh_ = private_nh;

  std::string map_topic;
  std::string goal_topic;
  std::string path_topic;
  private_nh_.param<std::string>("map_topic", map_topic, std::string("/map"));
  private_nh_.param<std::string>("goal_topic", goal_topic, std::string("/move_base_simple/goal"));
  private_nh_.param<std::string>("path_topic", path_topic,
                                 std::string("/mr_traditional_planner/debug_optimal_path"));
  private_nh_.param<std::string>("input_path_topic", input_path_topic_, input_path_topic_);
  private_nh_.param<double>("robot_radius", robot_radius_, robot_radius_);
  private_nh_.param<double>("spline_resolution", spline_resolution_, spline_resolution_);
  private_nh_.param<double>("control_point_ratio", control_point_ratio_, control_point_ratio_);
  private_nh_.param<bool>("collision_check", collision_check_, collision_check_);
  private_nh_.param<std::string>("map_frame", map_frame_, map_frame_);
  private_nh_.param<std::string>("robot_frame", robot_frame_, robot_frame_);

  spline_resolution_ = std::max(1.0e-3, spline_resolution_);
  control_point_ratio_ = std::max(0.05, std::min(1.0, control_point_ratio_));

  map_sub_ = nh_.subscribe(map_topic, 1, &CubicSplinePlanner::mapCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic, 1, &CubicSplinePlanner::goalCallback, this);
  if (!input_path_topic_.empty()) {
    input_path_sub_ =
        nh_.subscribe(input_path_topic_, 1, &CubicSplinePlanner::inputPathCallback, this);
  }
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);
}

void CubicSplinePlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void CubicSplinePlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  latest_goal_ = msg;

  if (collision_check_ && !latest_map_) {
    ROS_WARN("CubicSpline C++: /map has not been received yet.");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    ROS_WARN_STREAM("CubicSpline C++: goal frame must be " << map_frame_ << ", got "
                                                           << msg->header.frame_id << ".");
    return;
  }

  double start_x = 0.0;
  double start_y = 0.0;
  double start_yaw = 0.0;
  if (!lookupStartPose(start_x, start_y, start_yaw)) {
    return;
  }

  std::vector<std::pair<double, double>> raw_path;
  if (collision_check_) {
    raw_path = buildGridPath(start_x, start_y, msg->pose.position.x, msg->pose.position.y);
    if (raw_path.empty()) {
      ROS_WARN("CubicSpline C++: failed to generate collision-free raw path.");
      return;
    }
  } else {
    const double goal_yaw = tf::getYaw(msg->pose.orientation);
    raw_path = buildGoalAnchors(start_x, start_y, start_yaw, msg->pose.position.x,
                                msg->pose.position.y, goal_yaw);
  }

  raw_path = sanitizePathPoints(raw_path);
  const std::vector<std::pair<double, double>> path_points = smoothAnchors(raw_path);
  if (path_points.empty()) {
    ROS_WARN("CubicSpline C++: failed to generate spline path.");
    return;
  }

  if (collision_check_ && !pathIsFree(path_points)) {
    ROS_WARN("[CubicSpline] collision detected, fallback to raw path");
    publishPath(raw_path);
    return;
  }

  publishPath(path_points);
}

void CubicSplinePlanner::inputPathCallback(const nav_msgs::PathConstPtr& msg) {
  if (collision_check_ && !latest_map_) {
    ROS_WARN("CubicSpline C++: /map has not been received yet.");
    return;
  }

  if (msg->poses.size() < 2U) {
    ROS_WARN("CubicSpline C++: input path needs at least two poses.");
    return;
  }

  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    ROS_WARN_STREAM("CubicSpline C++: input path frame must be " << map_frame_ << ", got "
                                                                 << msg->header.frame_id << ".");
    return;
  }

  std::vector<std::pair<double, double>> anchors;
  anchors.reserve(msg->poses.size());
  for (const geometry_msgs::PoseStamped& pose : msg->poses) {
    anchors.emplace_back(pose.pose.position.x, pose.pose.position.y);
  }
  anchors = sanitizePathPoints(anchors);
  if (anchors.size() < 2U) {
    ROS_WARN("CubicSpline C++: input path has fewer than two valid poses after cleanup.");
    return;
  }

  const std::vector<std::pair<double, double>> path_points = smoothAnchors(anchors);
  if (path_points.empty()) {
    ROS_WARN("CubicSpline C++: failed to smooth input path.");
    return;
  }

  if (collision_check_ && !pathIsFree(path_points)) {
    ROS_WARN("[CubicSpline] collision detected, fallback to raw path");
    publishPath(anchors);
    return;
  }

  publishPath(path_points);
}

void CubicSplinePlanner::buildObstacleLookup() {
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

void CubicSplinePlanner::precomputeInflationOffsets() {
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

bool CubicSplinePlanner::lookupStartPose(double& start_world_x, double& start_world_y,
                                         double& start_yaw) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.2));
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM("CubicSpline C++: failed to lookup " << map_frame_ << " -> " << robot_frame_
                                                         << ": " << ex.what());
    return false;
  }

  start_world_x = transform.getOrigin().x();
  start_world_y = transform.getOrigin().y();
  start_yaw = tf::getYaw(transform.getRotation());
  return true;
}

bool CubicSplinePlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool CubicSplinePlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }
  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

bool CubicSplinePlanner::isWorldPointFree(double world_x, double world_y) const {
  if (!std::isfinite(world_x) || !std::isfinite(world_y)) {
    return false;
  }
  if (!latest_map_) {
    return true;
  }
  const std::pair<int, int> grid = worldToGrid(world_x, world_y);
  return !isObstacle(grid.first, grid.second);
}

int CubicSplinePlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> CubicSplinePlanner::worldToGrid(double world_x, double world_y) const {
  return std::make_pair(static_cast<int>(std::floor((world_x - origin_x_) / resolution_)),
                        static_cast<int>(std::floor((world_y - origin_y_) / resolution_)));
}

std::pair<double, double> CubicSplinePlanner::gridToWorld(int grid_x, int grid_y) const {
  return std::make_pair(origin_x_ + (static_cast<double>(grid_x) + 0.5) * resolution_,
                        origin_y_ + (static_cast<double>(grid_y) + 0.5) * resolution_);
}

std::vector<std::pair<double, double>> CubicSplinePlanner::buildGridPath(
    double start_x, double start_y, double goal_x, double goal_y) const {
  if (!latest_map_ || resolution_ <= 0.0) {
    return std::vector<std::pair<double, double>>();
  }

  const std::pair<int, int> start_grid = worldToGrid(start_x, start_y);
  const std::pair<int, int> goal_grid = worldToGrid(goal_x, goal_y);
  if (isObstacle(start_grid.first, start_grid.second) ||
      isObstacle(goal_grid.first, goal_grid.second)) {
    return std::vector<std::pair<double, double>>();
  }

  struct QueueNode {
    int index;
    double priority;

    bool operator<(const QueueNode& other) const {
      return priority > other.priority;
    }
  };

  const int total_cells = map_width_ * map_height_;
  const int start_index = toIndex(start_grid.first, start_grid.second);
  const int goal_index = toIndex(goal_grid.first, goal_grid.second);
  std::vector<double> cost(static_cast<std::size_t>(total_cells),
                           std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<std::size_t>(total_cells), -1);
  std::vector<std::uint8_t> closed(static_cast<std::size_t>(total_cells), 0U);
  std::priority_queue<QueueNode> open;

  const auto heuristic = [&](int grid_x, int grid_y) {
    return std::hypot(static_cast<double>(goal_grid.first - grid_x),
                      static_cast<double>(goal_grid.second - grid_y)) *
           resolution_;
  };

  cost[static_cast<std::size_t>(start_index)] = 0.0;
  parent[static_cast<std::size_t>(start_index)] = start_index;
  open.push(QueueNode{start_index, heuristic(start_grid.first, start_grid.second)});

  const int directions[8][2] = {{1, 0},   {-1, 0},  {0, 1},  {0, -1},
                                {1, 1},   {1, -1},  {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    const QueueNode current = open.top();
    open.pop();

    if (closed[static_cast<std::size_t>(current.index)] != 0U) {
      continue;
    }
    closed[static_cast<std::size_t>(current.index)] = 1U;

    if (current.index == goal_index) {
      break;
    }

    const int current_x = current.index % map_width_;
    const int current_y = current.index / map_width_;
    for (const int* direction : directions) {
      const int next_x = current_x + direction[0];
      const int next_y = current_y + direction[1];
      if (isObstacle(next_x, next_y)) {
        continue;
      }
      if (direction[0] != 0 && direction[1] != 0 &&
          (isObstacle(current_x + direction[0], current_y) ||
           isObstacle(current_x, current_y + direction[1]))) {
        continue;
      }

      const int next_index = toIndex(next_x, next_y);
      const double step_cost =
          std::hypot(static_cast<double>(direction[0]), static_cast<double>(direction[1])) *
          resolution_;
      const double next_cost = cost[static_cast<std::size_t>(current.index)] + step_cost;
      if (next_cost < cost[static_cast<std::size_t>(next_index)]) {
        cost[static_cast<std::size_t>(next_index)] = next_cost;
        parent[static_cast<std::size_t>(next_index)] = current.index;
        open.push(QueueNode{next_index, next_cost + heuristic(next_x, next_y)});
      }
    }
  }

  if (parent[static_cast<std::size_t>(goal_index)] < 0) {
    return std::vector<std::pair<double, double>>();
  }

  std::vector<int> reversed_indices;
  for (int index = goal_index; index != start_index;
       index = parent[static_cast<std::size_t>(index)]) {
    reversed_indices.push_back(index);
  }
  reversed_indices.push_back(start_index);
  std::reverse(reversed_indices.begin(), reversed_indices.end());

  std::vector<std::pair<double, double>> path_points;
  path_points.reserve(reversed_indices.size());
  for (const int index : reversed_indices) {
    path_points.push_back(gridToWorld(index % map_width_, index / map_width_));
  }
  if (!path_points.empty()) {
    path_points.front() = std::make_pair(start_x, start_y);
    path_points.back() = std::make_pair(goal_x, goal_y);
  }
  return sanitizePathPoints(path_points);
}

std::vector<std::pair<double, double>> CubicSplinePlanner::buildGoalAnchors(
    double start_x, double start_y, double start_yaw, double goal_x, double goal_y,
    double goal_yaw) const {
  const double path_length = std::hypot(goal_x - start_x, goal_y - start_y);
  if (path_length < 1.0e-6) {
    return std::vector<std::pair<double, double>>{{start_x, start_y}};
  }

  const double tangent_length = path_length * control_point_ratio_;
  std::vector<std::pair<double, double>> anchors;
  anchors.emplace_back(start_x, start_y);
  anchors.emplace_back(start_x + tangent_length * std::cos(start_yaw),
                       start_y + tangent_length * std::sin(start_yaw));
  anchors.emplace_back(goal_x - tangent_length * std::cos(goal_yaw),
                       goal_y - tangent_length * std::sin(goal_yaw));
  anchors.emplace_back(goal_x, goal_y);
  return deduplicateAnchors(anchors);
}

std::vector<std::pair<double, double>> CubicSplinePlanner::smoothAnchors(
    const std::vector<std::pair<double, double>>& anchors) const {
  const std::vector<std::pair<double, double>> clean_anchors = sanitizePathPoints(anchors);
  if (clean_anchors.size() < 2U) {
    return clean_anchors;
  }

  CubicSpline2D spline(clean_anchors);
  const double max_s = spline.maxS();
  if (max_s < 1.0e-6) {
    return clean_anchors;
  }

  std::vector<std::pair<double, double>> path_points;
  for (double s = 0.0; s < max_s; s += spline_resolution_) {
    path_points.push_back(spline.calcPosition(s));
  }
  path_points.push_back(spline.calcPosition(max_s));
  return sanitizePathPoints(path_points);
}

double CubicSplinePlanner::pathCleanupDistance() const {
  const double fallback_distance = std::max(1.0e-6, spline_resolution_ * 0.5);
  if (resolution_ <= 0.0) {
    return fallback_distance;
  }
  return std::max(1.0e-6, std::min(fallback_distance, resolution_ * 0.25));
}

std::vector<std::pair<double, double>> CubicSplinePlanner::sanitizePathPoints(
    const std::vector<std::pair<double, double>>& path_points) const {
  std::vector<std::pair<double, double>> clean_points;
  clean_points.reserve(path_points.size());
  const double min_distance = pathCleanupDistance();

  for (const std::pair<double, double>& point : path_points) {
    if (!std::isfinite(point.first) || !std::isfinite(point.second)) {
      continue;
    }
    if (!clean_points.empty() &&
        std::hypot(point.first - clean_points.back().first,
                   point.second - clean_points.back().second) < min_distance) {
      continue;
    }
    if (clean_points.size() >= 2U) {
      const std::pair<double, double>& previous_previous =
          clean_points[clean_points.size() - 2U];
      if (std::hypot(point.first - previous_previous.first,
                     point.second - previous_previous.second) < min_distance) {
        clean_points.pop_back();
        continue;
      }
    }
    clean_points.push_back(point);
  }

  return clean_points;
}

double CubicSplinePlanner::collisionCheckStep() const {
  const double fallback_step = std::max(1.0e-6, spline_resolution_);
  if (resolution_ <= 0.0) {
    return fallback_step;
  }
  return std::max(1.0e-6, std::min(fallback_step, resolution_ * 0.5));
}

bool CubicSplinePlanner::pathIsFree(
    const std::vector<std::pair<double, double>>& path_points) const {
  if (path_points.empty()) {
    return true;
  }

  if (!isWorldPointFree(path_points.front().first, path_points.front().second)) {
    return false;
  }

  const double sample_step = collisionCheckStep();
  for (std::size_t i = 1U; i < path_points.size(); ++i) {
    const std::pair<double, double>& start = path_points[i - 1U];
    const std::pair<double, double>& end = path_points[i];
    const double distance = std::hypot(end.first - start.first, end.second - start.second);
    const int sample_count = std::max(1, static_cast<int>(std::ceil(distance / sample_step)));

    for (int sample = 1; sample <= sample_count; ++sample) {
      const double ratio = static_cast<double>(sample) / static_cast<double>(sample_count);
      const double world_x = start.first + (end.first - start.first) * ratio;
      const double world_y = start.second + (end.second - start.second) * ratio;
      if (!isWorldPointFree(world_x, world_y)) {
        return false;
      }
    }
  }
  return true;
}

void CubicSplinePlanner::publishPath(
    const std::vector<std::pair<double, double>>& path_points) const {
  const std::vector<std::pair<double, double>> clean_points = sanitizePathPoints(path_points);
  if (clean_points.empty()) {
    return;
  }

  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(clean_points.size());
  for (std::size_t i = 0; i < clean_points.size(); ++i) {
    double yaw = 0.0;
    if (i + 1U < clean_points.size()) {
      yaw = std::atan2(clean_points[i + 1U].second - clean_points[i].second,
                       clean_points[i + 1U].first - clean_points[i].first);
    } else if (i > 0U) {
      yaw = std::atan2(clean_points[i].second - clean_points[i - 1U].second,
                       clean_points[i].first - clean_points[i - 1U].first);
    }

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = clean_points[i].first;
    pose.pose.position.y = clean_points[i].second;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
}

}  // namespace optimal
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::optimal::CubicSplinePlanner,
                       mr_traditional_planner::PlannerPlugin)
