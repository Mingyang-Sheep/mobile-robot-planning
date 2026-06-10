#ifndef MR_TRADITIONAL_PLANNER_PATH_TOPIC_GLOBAL_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_PATH_TOPIC_GLOBAL_PLANNER_H_

#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

namespace mr_traditional_planner {

class PathTopicGlobalPlanner : public nav_core::BaseGlobalPlanner {
 public:
  PathTopicGlobalPlanner();
  PathTopicGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override;

 private:
  void pathCallback(const nav_msgs::PathConstPtr& msg);
  bool pathMatchesGoal(const nav_msgs::Path& path,
                       const geometry_msgs::PoseStamped& goal) const;
  void normalizePlan(const geometry_msgs::PoseStamped& start,
                     const geometry_msgs::PoseStamped& goal,
                     std::vector<geometry_msgs::PoseStamped>& plan) const;

  bool initialized_;
  bool waiting_for_path_;
  bool path_received_;
  double plan_timeout_;
  double goal_match_tolerance_;
  double endpoint_insert_tolerance_;
  std::string global_frame_;
  std::string goal_topic_;
  std::string path_topic_;

  ros::Publisher goal_pub_;
  ros::Subscriber path_sub_;

  mutable std::mutex mutex_;
  std::condition_variable path_condition_;
  geometry_msgs::PoseStamped requested_goal_;
  nav_msgs::Path received_path_;
};

}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_PATH_TOPIC_GLOBAL_PLANNER_H_
