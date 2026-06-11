#ifndef MR_TRADITIONAL_PLANNER_OPTIMAL_RRT_STAR_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_OPTIMAL_RRT_STAR_PLANNER_H_

#include "mr_traditional_planner/planner_plugin.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

struct RRTStarNode {
  double x;
  double y;
  double cost;
  int parent_index;
};

class RRTStarPlanner : public PlannerPlugin {
 public:
  RRTStarPlanner();
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) override;

 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void buildObstacleLookup();
  void precomputeInflationOffsets();
  bool lookupStartPose(double& start_world_x, double& start_world_y);
  bool inBounds(int grid_x, int grid_y) const;
  bool worldInBounds(double world_x, double world_y) const;
  bool isObstacle(int grid_x, int grid_y) const;
  bool isWorldPointFree(double world_x, double world_y) const;
  bool isSegmentFree(double from_x, double from_y, double to_x, double to_y) const;
  int toIndex(int grid_x, int grid_y) const;
  std::pair<int, int> worldToGrid(double world_x, double world_y) const;
  double distance(double from_x, double from_y, double to_x, double to_y) const;
  RRTStarNode sampleNode(double goal_x, double goal_y);
  RRTStarNode steer(const RRTStarNode& from_node, const RRTStarNode& to_node) const;
  int nearestNodeIndex(const std::vector<RRTStarNode>& nodes,
                       const RRTStarNode& target_node) const;
  std::vector<int> nearNodeIndices(const std::vector<RRTStarNode>& nodes,
                                   const RRTStarNode& new_node) const;
  void chooseParent(const std::vector<RRTStarNode>& nodes, const std::vector<int>& near_indices,
                    RRTStarNode& new_node) const;
  void rewire(std::vector<RRTStarNode>& nodes, int new_index,
              const std::vector<int>& near_indices) const;
  void propagateCostToChildren(std::vector<RRTStarNode>& nodes, int parent_index) const;
  int bestGoalNodeIndex(const std::vector<RRTStarNode>& nodes, double goal_x, double goal_y) const;
  std::vector<std::pair<double, double>> reconstructPath(
      const std::vector<RRTStarNode>& nodes, int goal_parent_index, double goal_x,
      double goal_y) const;
  std::vector<std::pair<double, double>> planPath(double start_x, double start_y, double goal_x,
                                                  double goal_y);
  void publishPath(const std::vector<std::pair<double, double>>& path_points) const;
  void publishFailure(const std::string& reason) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  tf::TransformListener tf_listener_;

  nav_msgs::OccupancyGridConstPtr latest_map_;
  geometry_msgs::PoseStampedConstPtr latest_goal_;
  int map_width_;
  int map_height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  double robot_radius_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string path_topic_;
  std::vector<std::uint8_t> obstacle_grid_;
  std::vector<std::pair<int, int>> inflation_offsets_;

  int max_iterations_;
  double expand_distance_;
  double path_resolution_;
  int goal_sample_rate_;
  double connect_circle_distance_;
  bool search_until_max_iter_;
  std::mt19937 random_engine_;
};

}  // namespace optimal
}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_OPTIMAL_RRT_STAR_PLANNER_H_
