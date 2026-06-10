#include "mr_traditional_planner/path_topic_global_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace mr_traditional_planner {

PathTopicGlobalPlanner::PathTopicGlobalPlanner()
    : initialized_(false),
      waiting_for_path_(false),
      path_received_(false),
      plan_timeout_(5.0),
      goal_match_tolerance_(0.35),
      endpoint_insert_tolerance_(0.05) {}

PathTopicGlobalPlanner::PathTopicGlobalPlanner(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : PathTopicGlobalPlanner() {
  initialize(std::move(name), costmap_ros);
}

void PathTopicGlobalPlanner::initialize(
    std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
  if (initialized_) {
    ROS_WARN("PathTopicGlobalPlanner has already been initialized.");
    return;
  }

  ros::NodeHandle move_base_nh("~");
  ros::NodeHandle private_nh("~/" + name);
  global_frame_ = costmap_ros ? costmap_ros->getGlobalFrameID() : "map";

  move_base_nh.param<std::string>(
      "traditional_goal_topic", goal_topic_,
      std::string("/mr_traditional_planner/planner_goal"));
  move_base_nh.param<std::string>(
      "traditional_path_topic", path_topic_,
      std::string("/mr_traditional_planner/optimal_path"));
  move_base_nh.param<double>(
      "traditional_plan_timeout", plan_timeout_, plan_timeout_);

  private_nh.param<std::string>(
      "goal_topic", goal_topic_, goal_topic_);
  private_nh.param<std::string>(
      "path_topic", path_topic_, path_topic_);
  private_nh.param<double>("plan_timeout", plan_timeout_, plan_timeout_);
  private_nh.param<double>(
      "goal_match_tolerance", goal_match_tolerance_,
      goal_match_tolerance_);
  private_nh.param<double>(
      "endpoint_insert_tolerance", endpoint_insert_tolerance_,
      endpoint_insert_tolerance_);

  plan_timeout_ = std::max(0.1, plan_timeout_);
  goal_match_tolerance_ = std::max(0.01, goal_match_tolerance_);
  endpoint_insert_tolerance_ = std::max(0.0, endpoint_insert_tolerance_);

  ros::NodeHandle nh;
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
      goal_topic_, 1, true);
  path_sub_ = nh.subscribe(
      path_topic_, 1, &PathTopicGlobalPlanner::pathCallback, this);

  initialized_ = true;
  ROS_INFO_STREAM(
      "PathTopicGlobalPlanner initialized: goal=" << goal_topic_
      << ", path=" << path_topic_
      << ", timeout=" << plan_timeout_ << " s");
}

bool PathTopicGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) {
  plan.clear();
  if (!initialized_) {
    ROS_ERROR("PathTopicGlobalPlanner is not initialized.");
    return false;
  }

  if ((!start.header.frame_id.empty() &&
       start.header.frame_id != global_frame_) ||
      (!goal.header.frame_id.empty() &&
       goal.header.frame_id != global_frame_)) {
    ROS_ERROR_STREAM(
        "PathTopicGlobalPlanner requires start and goal in "
        << global_frame_ << ". Got start=" << start.header.frame_id
        << ", goal=" << goal.header.frame_id << ".");
    return false;
  }

  geometry_msgs::PoseStamped request = goal;
  request.header.frame_id = global_frame_;
  request.header.stamp = ros::Time::now();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    requested_goal_ = request;
    received_path_ = nav_msgs::Path();
    path_received_ = false;
    waiting_for_path_ = true;
  }

  goal_pub_.publish(request);

  std::unique_lock<std::mutex> lock(mutex_);
  const bool received = path_condition_.wait_for(
      lock, std::chrono::duration<double>(plan_timeout_),
      [this]() { return path_received_ || !ros::ok(); });

  waiting_for_path_ = false;
  if (!received || !path_received_ || received_path_.poses.empty()) {
    ROS_WARN_STREAM(
        "Timed out waiting for a valid path on " << path_topic_
        << " after " << plan_timeout_ << " s.");
    return false;
  }

  plan = received_path_.poses;
  lock.unlock();

  normalizePlan(start, goal, plan);
  ROS_INFO_STREAM(
      "PathTopicGlobalPlanner accepted a plan with "
      << plan.size() << " poses.");
  return !plan.empty();
}

void PathTopicGlobalPlanner::pathCallback(
    const nav_msgs::PathConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!waiting_for_path_ || msg->poses.empty()) {
    return;
  }

  if (!pathMatchesGoal(*msg, requested_goal_)) {
    ROS_WARN_THROTTLE(
        1.0,
        "Ignoring a custom path whose endpoint does not match the requested goal.");
    return;
  }

  received_path_ = *msg;
  path_received_ = true;
  path_condition_.notify_one();
}

bool PathTopicGlobalPlanner::pathMatchesGoal(
    const nav_msgs::Path& path,
    const geometry_msgs::PoseStamped& goal) const {
  if (path.poses.empty()) {
    return false;
  }

  if (!path.header.frame_id.empty() &&
      path.header.frame_id != global_frame_) {
    return false;
  }

  const geometry_msgs::Point& endpoint =
      path.poses.back().pose.position;
  const double dx = endpoint.x - goal.pose.position.x;
  const double dy = endpoint.y - goal.pose.position.y;
  return std::hypot(dx, dy) <= goal_match_tolerance_;
}

void PathTopicGlobalPlanner::normalizePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan) const {
  const ros::Time stamp = ros::Time::now();
  for (geometry_msgs::PoseStamped& pose : plan) {
    pose.header.frame_id = global_frame_;
    pose.header.stamp = stamp;
  }

  if (plan.empty()) {
    return;
  }

  const double start_dx =
      plan.front().pose.position.x - start.pose.position.x;
  const double start_dy =
      plan.front().pose.position.y - start.pose.position.y;
  if (std::hypot(start_dx, start_dy) >
      endpoint_insert_tolerance_) {
    geometry_msgs::PoseStamped start_pose = start;
    start_pose.header.frame_id = global_frame_;
    start_pose.header.stamp = stamp;
    plan.insert(plan.begin(), start_pose);
  }

  const double goal_dx =
      plan.back().pose.position.x - goal.pose.position.x;
  const double goal_dy =
      plan.back().pose.position.y - goal.pose.position.y;
  if (std::hypot(goal_dx, goal_dy) >
      endpoint_insert_tolerance_) {
    geometry_msgs::PoseStamped goal_pose = goal;
    goal_pose.header.frame_id = global_frame_;
    goal_pose.header.stamp = stamp;
    plan.push_back(goal_pose);
  } else {
    plan.back().pose = goal.pose;
  }

  for (std::size_t index = 0; index + 1 < plan.size(); ++index) {
    const double dx =
        plan[index + 1].pose.position.x -
        plan[index].pose.position.x;
    const double dy =
        plan[index + 1].pose.position.y -
        plan[index].pose.position.y;
    if (std::hypot(dx, dy) > 1.0e-6) {
      plan[index].pose.orientation =
          tf::createQuaternionMsgFromYaw(std::atan2(dy, dx));
    }
  }
  plan.back().pose.orientation = goal.pose.orientation;
}

}  // namespace mr_traditional_planner

PLUGINLIB_EXPORT_CLASS(
    mr_traditional_planner::PathTopicGlobalPlanner,
    nav_core::BaseGlobalPlanner)
