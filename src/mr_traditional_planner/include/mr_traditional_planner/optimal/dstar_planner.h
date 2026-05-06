#ifndef MR_TRADITIONAL_PLANNER_OPTIMAL_DSTAR_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_OPTIMAL_DSTAR_PLANNER_H_

#include "mr_traditional_planner/planner_plugin.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

class DStarPlanner : public PlannerPlugin {
 public:
  DStarPlanner();
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) override;

 private:
  enum class SearchTag {
    kNew,
    kOpen,
    kClosed,
  };

  struct SearchState {
    int x;
    int y;
    double h;
    double k;
    int parent_index;
    SearchTag tag;
  };

  struct OpenItem {
    double k;
    int index;

    bool operator<(const OpenItem& other) const {
      if (k != other.k) {
        return k > other.k;
      }
      return index > other.index;
    }
  };

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void buildObstacleLookup();
  void precomputeInflationOffsets();
  bool lookupStartPose(double& start_world_x, double& start_world_y);
  bool inBounds(int grid_x, int grid_y) const;
  bool isObstacle(int grid_x, int grid_y) const;
  int toIndex(int grid_x, int grid_y) const;
  std::pair<int, int> indexToGrid(int linear_index) const;
  std::pair<int, int> worldToGrid(double world_x, double world_y) const;
  std::pair<double, double> gridToWorld(int grid_x, int grid_y) const;
  std::vector<int> planPath(int start_x, int start_y, int goal_x, int goal_y);
  std::vector<int> neighbors(int linear_index) const;
  double moveCost(int from_index, int to_index) const;
  double processState();
  double getKMin();
  bool popMinOpen(int& linear_index, double& key);
  void insertState(int linear_index, double new_h);
  bool nearlyEqual(double lhs, double rhs) const;
  void publishPath(const std::vector<int>& path_indices) const;

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
  std::vector<std::uint8_t> obstacle_grid_;
  std::vector<std::pair<int, int>> inflation_offsets_;
  std::vector<SearchState> search_states_;
  std::priority_queue<OpenItem> open_queue_;
};

}  // namespace optimal
}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_OPTIMAL_DSTAR_PLANNER_H_
