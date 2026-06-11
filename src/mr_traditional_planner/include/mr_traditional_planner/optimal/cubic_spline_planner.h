#ifndef MR_TRADITIONAL_PLANNER_OPTIMAL_CUBIC_SPLINE_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_OPTIMAL_CUBIC_SPLINE_PLANNER_H_

#include "mr_traditional_planner/planner_plugin.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

class CubicSplinePlanner : public PlannerPlugin {
 public:
  CubicSplinePlanner();
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) override;

 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void inputPathCallback(const nav_msgs::PathConstPtr& msg);
  void buildObstacleLookup();
  void precomputeInflationOffsets();
  bool lookupStartPose(double& start_world_x, double& start_world_y, double& start_yaw);
  bool inBounds(int grid_x, int grid_y) const;
  bool isObstacle(int grid_x, int grid_y) const;
  bool isWorldPointFree(double world_x, double world_y) const;
  int toIndex(int grid_x, int grid_y) const;
  std::pair<int, int> worldToGrid(double world_x, double world_y) const;
  std::pair<double, double> gridToWorld(int grid_x, int grid_y) const;
  std::vector<std::pair<double, double>> buildGridPath(double start_x, double start_y,
                                                       double goal_x, double goal_y) const;
  std::vector<std::pair<double, double>> buildGoalAnchors(double start_x, double start_y,
                                                          double start_yaw, double goal_x,
                                                          double goal_y,
                                                          double goal_yaw) const;
  std::vector<std::pair<double, double>> smoothAnchors(
      const std::vector<std::pair<double, double>>& anchors) const;
  double pathCleanupDistance() const;
  std::vector<std::pair<double, double>> sanitizePathPoints(
      const std::vector<std::pair<double, double>>& path_points) const;
  double collisionCheckStep() const;
  bool pathIsFree(const std::vector<std::pair<double, double>>& path_points) const;
  void publishPath(const std::vector<std::pair<double, double>>& path_points) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber input_path_sub_;
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
  double spline_resolution_;
  double control_point_ratio_;
  bool collision_check_;
  std::string map_frame_;
  std::string robot_frame_;
  std::string input_path_topic_;
  std::vector<std::uint8_t> obstacle_grid_;
  std::vector<std::pair<int, int>> inflation_offsets_;
};

}  // namespace optimal
}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_OPTIMAL_CUBIC_SPLINE_PLANNER_H_
