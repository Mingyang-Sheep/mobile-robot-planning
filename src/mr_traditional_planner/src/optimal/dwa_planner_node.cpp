#include "mr_traditional_planner/optimal/dwa_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

DynamicWindowApproachPlanner::DynamicWindowApproachPlanner()
    : goal_active_(false),
      map_width_(0),
      map_height_(0),
      resolution_(0.0),
      origin_x_(0.0),
      origin_y_(0.0),
      robot_radius_(0.15),
      map_frame_("map"),
      robot_frame_("base_footprint"),
      max_speed_(0.22),
      min_speed_(0.0),
      max_yaw_rate_(1.82),
      max_accel_(0.2),
      max_delta_yaw_rate_(2.0),
      velocity_resolution_(0.02),
      yaw_rate_resolution_(0.1),
      dt_(0.1),
      predict_time_(2.0),
      to_goal_cost_gain_(0.15),
      speed_cost_gain_(1.0),
      obstacle_cost_gain_(1.0),
      goal_tolerance_(0.2),
      robot_stuck_flag_cons_(0.001) {}

void DynamicWindowApproachPlanner::initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  nh_ = nh;
  private_nh_ = private_nh;

  std::string map_topic;
  std::string odom_topic;
  std::string goal_topic;
  std::string cmd_vel_topic;
  std::string path_topic;
  double control_frequency = 10.0;
  private_nh_.param<std::string>("map_topic", map_topic, std::string("/map"));
  private_nh_.param<std::string>("odom_topic", odom_topic, std::string("/odom"));
  private_nh_.param<std::string>("goal_topic", goal_topic, std::string("/move_base_simple/goal"));
  private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic, std::string("/cmd_vel"));
  private_nh_.param<std::string>("path_topic", path_topic,
                                 std::string("/mr_traditional_planner/optimal_path"));
  private_nh_.param<double>("robot_radius", robot_radius_, robot_radius_);
  private_nh_.param<std::string>("map_frame", map_frame_, map_frame_);
  private_nh_.param<std::string>("robot_frame", robot_frame_, robot_frame_);
  private_nh_.param<double>("max_speed", max_speed_, max_speed_);
  private_nh_.param<double>("min_speed", min_speed_, min_speed_);
  private_nh_.param<double>("max_yaw_rate", max_yaw_rate_, max_yaw_rate_);
  private_nh_.param<double>("max_accel", max_accel_, max_accel_);
  private_nh_.param<double>("max_delta_yaw_rate", max_delta_yaw_rate_, max_delta_yaw_rate_);
  private_nh_.param<double>("velocity_resolution", velocity_resolution_, velocity_resolution_);
  private_nh_.param<double>("yaw_rate_resolution", yaw_rate_resolution_, yaw_rate_resolution_);
  private_nh_.param<double>("dt", dt_, dt_);
  private_nh_.param<double>("predict_time", predict_time_, predict_time_);
  private_nh_.param<double>("to_goal_cost_gain", to_goal_cost_gain_, to_goal_cost_gain_);
  private_nh_.param<double>("speed_cost_gain", speed_cost_gain_, speed_cost_gain_);
  private_nh_.param<double>("obstacle_cost_gain", obstacle_cost_gain_, obstacle_cost_gain_);
  private_nh_.param<double>("goal_tolerance", goal_tolerance_, goal_tolerance_);
  private_nh_.param<double>("control_frequency", control_frequency, control_frequency);

  max_speed_ = std::max(0.0, max_speed_);
  min_speed_ = std::min(min_speed_, max_speed_);
  max_yaw_rate_ = std::max(1.0e-3, max_yaw_rate_);
  max_accel_ = std::max(1.0e-3, max_accel_);
  max_delta_yaw_rate_ = std::max(1.0e-3, max_delta_yaw_rate_);
  velocity_resolution_ = std::max(1.0e-3, velocity_resolution_);
  yaw_rate_resolution_ = std::max(1.0e-3, yaw_rate_resolution_);
  dt_ = std::max(1.0e-3, dt_);
  predict_time_ = std::max(dt_, predict_time_);
  goal_tolerance_ = std::max(1.0e-3, goal_tolerance_);
  control_frequency = std::max(1.0, control_frequency);

  map_sub_ = nh_.subscribe(map_topic, 1, &DynamicWindowApproachPlanner::mapCallback, this);
  odom_sub_ = nh_.subscribe(odom_topic, 1, &DynamicWindowApproachPlanner::odomCallback, this);
  goal_sub_ = nh_.subscribe(goal_topic, 1, &DynamicWindowApproachPlanner::goalCallback, this);
  cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  path_pub_ = nh_.advertise<nav_msgs::Path>(path_topic, 1, true);
  control_timer_ = nh_.createTimer(ros::Duration(1.0 / control_frequency),
                                  &DynamicWindowApproachPlanner::controlTimerCallback, this);
}

void DynamicWindowApproachPlanner::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
  latest_map_ = msg;
  map_width_ = static_cast<int>(msg->info.width);
  map_height_ = static_cast<int>(msg->info.height);
  resolution_ = msg->info.resolution;
  origin_x_ = msg->info.origin.position.x;
  origin_y_ = msg->info.origin.position.y;
  buildObstacleLookup();
}

void DynamicWindowApproachPlanner::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
  latest_odom_ = msg;
}

void DynamicWindowApproachPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
    ROS_WARN_STREAM("DWA C++: goal frame must be " << map_frame_ << ", got "
                                                   << msg->header.frame_id << ".");
    return;
  }

  latest_goal_ = msg;
  goal_active_ = true;
}

void DynamicWindowApproachPlanner::controlTimerCallback(const ros::TimerEvent&) {
  if (!goal_active_) {
    return;
  }

  if (!latest_map_) {
    ROS_WARN_THROTTLE(1.0, "DWA C++: /map has not been received yet.");
    publishStop();
    return;
  }

  if (!latest_odom_) {
    ROS_WARN_THROTTLE(1.0, "DWA C++: /odom has not been received yet.");
    publishStop();
    return;
  }

  if (!latest_goal_) {
    publishStop();
    return;
  }

  DwaState state;
  if (!lookupRobotState(state)) {
    publishStop();
    return;
  }

  const double goal_x = latest_goal_->pose.position.x;
  const double goal_y = latest_goal_->pose.position.y;
  if (std::hypot(state.x - goal_x, state.y - goal_y) <= goal_tolerance_) {
    goal_active_ = false;
    publishStop();
    publishTrajectory(std::vector<DwaState>{state});
    ROS_INFO("DWA C++: goal reached.");
    return;
  }

  DwaControl control{0.0, 0.0};
  std::vector<DwaState> trajectory;
  if (!selectControl(state, control, trajectory)) {
    ROS_WARN_THROTTLE(1.0, "DWA C++: failed to find a collision-free control.");
    publishStop();
    return;
  }

  publishCommand(control);
  publishTrajectory(trajectory);
}

void DynamicWindowApproachPlanner::buildObstacleLookup() {
  obstacle_grid_.assign(static_cast<std::size_t>(map_width_ * map_height_), 0U);
  obstacle_points_.clear();
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

  for (int grid_y = 0; grid_y < map_height_; ++grid_y) {
    for (int grid_x = 0; grid_x < map_width_; ++grid_x) {
      if (obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] == 0U) {
        continue;
      }
      obstacle_points_.emplace_back(origin_x_ + (static_cast<double>(grid_x) + 0.5) * resolution_,
                                    origin_y_ + (static_cast<double>(grid_y) + 0.5) * resolution_);
    }
  }
}

void DynamicWindowApproachPlanner::precomputeInflationOffsets() {
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

bool DynamicWindowApproachPlanner::lookupRobotState(DwaState& state) {
  tf::StampedTransform transform;

  try {
    tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(0.05));
    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(1.0, "DWA C++: failed to lookup " << map_frame_ << " -> "
                                                               << robot_frame_ << ": "
                                                               << ex.what());
    return false;
  }

  state.x = transform.getOrigin().x();
  state.y = transform.getOrigin().y();
  state.yaw = tf::getYaw(transform.getRotation());
  state.velocity = latest_odom_ ? latest_odom_->twist.twist.linear.x : 0.0;
  state.yaw_rate = latest_odom_ ? latest_odom_->twist.twist.angular.z : 0.0;
  return true;
}

bool DynamicWindowApproachPlanner::inBounds(int grid_x, int grid_y) const {
  return grid_x >= 0 && grid_x < map_width_ && grid_y >= 0 && grid_y < map_height_;
}

bool DynamicWindowApproachPlanner::isObstacle(int grid_x, int grid_y) const {
  if (!inBounds(grid_x, grid_y)) {
    return true;
  }
  return obstacle_grid_[static_cast<std::size_t>(toIndex(grid_x, grid_y))] != 0U;
}

bool DynamicWindowApproachPlanner::isWorldPointFree(double world_x, double world_y) const {
  const std::pair<int, int> grid = worldToGrid(world_x, world_y);
  return !isObstacle(grid.first, grid.second);
}

int DynamicWindowApproachPlanner::toIndex(int grid_x, int grid_y) const {
  return grid_y * map_width_ + grid_x;
}

std::pair<int, int> DynamicWindowApproachPlanner::worldToGrid(double world_x,
                                                              double world_y) const {
  return std::make_pair(static_cast<int>((world_x - origin_x_) / resolution_),
                        static_cast<int>((world_y - origin_y_) / resolution_));
}

DwaState DynamicWindowApproachPlanner::motion(const DwaState& state, const DwaControl& control,
                                              double dt) const {
  DwaState next_state = state;
  next_state.yaw = normalizeAngle(next_state.yaw + control.yaw_rate * dt);
  next_state.x += control.velocity * std::cos(next_state.yaw) * dt;
  next_state.y += control.velocity * std::sin(next_state.yaw) * dt;
  next_state.velocity = control.velocity;
  next_state.yaw_rate = control.yaw_rate;
  return next_state;
}

std::vector<double> DynamicWindowApproachPlanner::calcDynamicWindow(const DwaState& state) const {
  const double min_velocity =
      std::max(min_speed_, state.velocity - max_accel_ * dt_);
  const double max_velocity =
      std::min(max_speed_, state.velocity + max_accel_ * dt_);
  const double min_yaw_rate =
      std::max(-max_yaw_rate_, state.yaw_rate - max_delta_yaw_rate_ * dt_);
  const double max_yaw_rate =
      std::min(max_yaw_rate_, state.yaw_rate + max_delta_yaw_rate_ * dt_);
  return std::vector<double>{min_velocity, max_velocity, min_yaw_rate, max_yaw_rate};
}

std::vector<DwaState> DynamicWindowApproachPlanner::predictTrajectory(
    const DwaState& state, const DwaControl& control) const {
  std::vector<DwaState> trajectory;
  trajectory.reserve(static_cast<std::size_t>(std::ceil(predict_time_ / dt_)) + 2U);
  trajectory.push_back(state);

  DwaState current_state = state;
  for (double time = 0.0; time <= predict_time_; time += dt_) {
    current_state = motion(current_state, control, dt_);
    trajectory.push_back(current_state);
  }

  return trajectory;
}

double DynamicWindowApproachPlanner::calcToGoalCost(
    const std::vector<DwaState>& trajectory) const {
  const DwaState& final_state = trajectory.back();
  const double dx = latest_goal_->pose.position.x - final_state.x;
  const double dy = latest_goal_->pose.position.y - final_state.y;
  const double goal_angle = std::atan2(dy, dx);
  return std::fabs(normalizeAngle(goal_angle - final_state.yaw));
}

double DynamicWindowApproachPlanner::calcObstacleCost(
    const std::vector<DwaState>& trajectory) const {
  double min_distance = std::numeric_limits<double>::infinity();

  for (const DwaState& state : trajectory) {
    if (!isWorldPointFree(state.x, state.y)) {
      return std::numeric_limits<double>::infinity();
    }

    for (const std::pair<double, double>& obstacle : obstacle_points_) {
      min_distance = std::min(min_distance,
                              std::hypot(state.x - obstacle.first, state.y - obstacle.second));
    }
  }

  if (!std::isfinite(min_distance)) {
    return 0.0;
  }
  return 1.0 / std::max(min_distance, 1.0e-6);
}

bool DynamicWindowApproachPlanner::selectControl(const DwaState& state, DwaControl& control,
                                                 std::vector<DwaState>& trajectory) const {
  const std::vector<double> dynamic_window = calcDynamicWindow(state);
  if (dynamic_window[0] > dynamic_window[1] || dynamic_window[2] > dynamic_window[3]) {
    return false;
  }

  double min_cost = std::numeric_limits<double>::infinity();
  for (double velocity = dynamic_window[0]; velocity <= dynamic_window[1] + 1.0e-9;
       velocity += velocity_resolution_) {
    for (double yaw_rate = dynamic_window[2]; yaw_rate <= dynamic_window[3] + 1.0e-9;
         yaw_rate += yaw_rate_resolution_) {
      DwaControl candidate_control{velocity, yaw_rate};
      std::vector<DwaState> candidate_trajectory = predictTrajectory(state, candidate_control);
      const double obstacle_cost = calcObstacleCost(candidate_trajectory);
      if (!std::isfinite(obstacle_cost)) {
        continue;
      }

      const double to_goal_cost = to_goal_cost_gain_ * calcToGoalCost(candidate_trajectory);
      const double speed_cost = speed_cost_gain_ * (max_speed_ - candidate_trajectory.back().velocity);
      const double final_cost = to_goal_cost + speed_cost + obstacle_cost_gain_ * obstacle_cost;
      if (final_cost < min_cost) {
        min_cost = final_cost;
        control = candidate_control;
        trajectory = candidate_trajectory;
      }
    }
  }

  if (!std::isfinite(min_cost)) {
    return false;
  }

  if (std::fabs(control.velocity) < robot_stuck_flag_cons_ &&
      std::fabs(state.velocity) < robot_stuck_flag_cons_) {
    control.yaw_rate = -max_delta_yaw_rate_;
  }
  return true;
}

void DynamicWindowApproachPlanner::publishCommand(const DwaControl& control) const {
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = control.velocity;
  cmd_vel.angular.z = control.yaw_rate;
  cmd_vel_pub_.publish(cmd_vel);
}

void DynamicWindowApproachPlanner::publishStop() const {
  publishCommand(DwaControl{0.0, 0.0});
}

void DynamicWindowApproachPlanner::publishTrajectory(
    const std::vector<DwaState>& trajectory) const {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = map_frame_;

  path_msg.poses.reserve(trajectory.size());
  for (const DwaState& state : trajectory) {
    geometry_msgs::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(state.yaw);
    path_msg.poses.push_back(pose);
  }

  path_pub_.publish(path_msg);
}

double DynamicWindowApproachPlanner::normalizeAngle(double angle) const {
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace optimal
}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(mr_traditional_planner::optimal::DynamicWindowApproachPlanner,
                       mr_traditional_planner::PlannerPlugin)
