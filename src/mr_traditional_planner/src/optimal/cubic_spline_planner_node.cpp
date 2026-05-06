#include "mr_traditional_planner/optimal/cubic_spline_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
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
      collision_check_(false),
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
                                 std::string("/mr_traditional_planner/optimal_path"));
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

  const double goal_yaw = tf::getYaw(msg->pose.orientation);
  const std::vector<std::pair<double, double>> anchors =
      buildGoalAnchors(start_x, start_y, start_yaw, msg->pose.position.x, msg->pose.position.y,
                       goal_yaw);
  const std::vector<std::pair<double, double>> path_points = smoothAnchors(anchors);
  if (path_points.empty()) {
    ROS_WARN("CubicSpline C++: failed to generate spline path.");
    return;
  }

  if (collision_check_ && !pathIsFree(path_points)) {
    ROS_WARN("CubicSpline C++: generated spline path intersects an inflated obstacle.");
    return;
  }

  publishPath(path_points);
}

void CubicSplinePlanner::inputPathCallback(const nav_msgs::PathConstPtr& msg) {
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

  const std::vector<std::pair<double, double>> path_points = smoothAnchors(anchors);
  if (path_points.empty()) {
    ROS_WARN("CubicSpline C++: failed to smooth input path.");
    return;
  }

  if (collision_check_ && !pathIsFree(path_points)) {
    ROS_WARN("CubicSpline C++: smoothed path intersects an inflated obstacle.");
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
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
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
  const std::vector<std::pair<double, double>> clean_anchors = deduplicateAnchors(anchors);
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
  return path_points;
}

bool CubicSplinePlanner::pathIsFree(
    const std::vector<std::pair<double, double>>& path_points) const {
  for (const std::pair<double, double>& point : path_points) {
    if (!isWorldPointFree(point.first, point.second)) {
      return false;
    }
  }
  return true;
}

void CubicSplinePlanner::publishPath(
    const std::vector<std::pair<double, double>>& path_points) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(path_points.size());
  for (std::size_t i = 0; i < path_points.size(); ++i) {
    double yaw = 0.0;
    if (i + 1U < path_points.size()) {
      yaw = std::atan2(path_points[i + 1U].second - path_points[i].second,
                       path_points[i + 1U].first - path_points[i].first);
    } else if (i > 0U) {
      yaw = std::atan2(path_points[i].second - path_points[i - 1U].second,
                       path_points[i].first - path_points[i - 1U].first);
    }

    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = path_points[i].first;
    pose.pose.position.y = path_points[i].second;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
}

}  // namespace optimal
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::optimal::CubicSplinePlanner,
                       mr_traditional_planner::PlannerPlugin)
