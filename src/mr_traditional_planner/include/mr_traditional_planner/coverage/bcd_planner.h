#ifndef MR_TRADITIONAL_PLANNER_COVERAGE_BCD_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_COVERAGE_BCD_PLANNER_H_

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace coverage {

class BcdPlanner {
 public:
  BcdPlanner();

 private:
  struct Segment {
    int y;
    int x_start;
    int x_end;
    int cell_id;
  };

  struct GridNode {
    int x;
    int y;
    double g;
    double h;
    int parent_index;
  };

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void buildObstacleLookup();
  void precomputeInflationOffsets();
  bool lookupStartPose(double& start_world_x, double& start_world_y);
  bool waitForMoveBaseServer();
  bool inBounds(int grid_x, int grid_y) const;
  bool isObstacle(int grid_x, int grid_y) const;
  int toIndex(int grid_x, int grid_y) const;
  std::pair<int, int> indexToGrid(int linear_index) const;
  std::pair<int, int> worldToGrid(double world_x, double world_y) const;
  std::pair<double, double> gridToWorld(int grid_x, int grid_y) const;
  std::vector<int> buildCoveragePath(int start_x, int start_y) const;
  std::vector<int> sampledRows(int sweep_step_cells) const;
  std::vector<Segment> extractFreeSegments(int row_y) const;
  bool segmentsOverlap(const Segment& lhs, const Segment& rhs) const;
  std::vector<int> generateCellPath(const std::vector<Segment>& cell_segments) const;
  std::vector<int> traceScanSegment(const Segment& segment, bool left_to_right) const;
  std::vector<int> planSafePath(int start_x, int start_y, int goal_x, int goal_y) const;
  std::vector<int> reconstructPath(int goal_index,
                                   const std::unordered_map<int, GridNode>& node_lookup) const;
  void appendIndices(const std::vector<int>& extension, std::vector<int>& path_indices) const;
  std::vector<int> compressPath(const std::vector<int>& path_indices) const;
  double heuristic(int grid_x, int grid_y, int goal_x, int goal_y) const;
  double computeWaypointYaw(const std::vector<int>& path_indices,
                            std::size_t waypoint_index) const;
  void publishPath(const std::vector<int>& path_indices) const;
  void executeCoveragePath(const std::vector<int>& path_indices);

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher path_pub_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
  tf::TransformListener tf_listener_;

  nav_msgs::OccupancyGridConstPtr latest_map_;
  int map_width_;
  int map_height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  double robot_radius_;
  double sweep_spacing_;
  double goal_timeout_;
  bool is_executing_;
  std::vector<std::uint8_t> obstacle_grid_;
  std::vector<std::pair<int, int>> inflation_offsets_;
};

}  // namespace coverage
}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_COVERAGE_BCD_PLANNER_H_
