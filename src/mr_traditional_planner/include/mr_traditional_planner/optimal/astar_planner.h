#ifndef MR_TRADITIONAL_PLANNER_OPTIMAL_ASTAR_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_OPTIMAL_ASTAR_PLANNER_H_

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

struct Node {
  int x;
  int y;
  double g;
  double h;
  int parent_index;
};

class AStarPlanner {
 public:
  AStarPlanner();

 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void buildObstacleLookup();
  void precomputeInflationOffsets();
  bool lookupStartPose(double& start_world_x, double& start_world_y);
  bool inBounds(int grid_x, int grid_y) const;
  bool isObstacle(int grid_x, int grid_y) const;
  int toIndex(int grid_x, int grid_y) const;
  std::pair<int, int> worldToGrid(double world_x, double world_y) const;
  std::pair<double, double> gridToWorld(int grid_x, int grid_y) const;
  double heuristic(int grid_x, int grid_y, int goal_x, int goal_y) const;
  std::vector<int> planPath(int start_x, int start_y, int goal_x, int goal_y);
  std::vector<int> reconstructPath(int goal_index,
                                   const std::unordered_map<int, Node>& node_lookup) const;
  void publishPath(const std::vector<int>& path_indices) const;

  ros::NodeHandle nh_;
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
  std::vector<std::uint8_t> obstacle_grid_;
  std::vector<std::pair<int, int>> inflation_offsets_;
};

}  // namespace optimal
}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_OPTIMAL_ASTAR_PLANNER_H_
