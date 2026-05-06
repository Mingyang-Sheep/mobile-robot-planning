#ifndef MR_TRADITIONAL_PLANNER_OPTIMAL_DWA_PLANNER_H_
#define MR_TRADITIONAL_PLANNER_OPTIMAL_DWA_PLANNER_H_

#include "mr_traditional_planner/planner_plugin.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace mr_traditional_planner {
namespace optimal {

struct DwaState {
  double x;
  double y;
  double yaw;
  double velocity;
  double yaw_rate;
};

struct DwaControl {
  double velocity;
  double yaw_rate;
};

class DynamicWindowApproachPlanner : public PlannerPlugin {
 public:
  DynamicWindowApproachPlanner();
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) override;

 private:
  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& msg);
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);
  void buildObstacleLookup();
  void precomputeInflationOffsets();
  bool lookupRobotState(DwaState& state);
  bool inBounds(int grid_x, int grid_y) const;
  bool isObstacle(int grid_x, int grid_y) const;
  bool isWorldPointFree(double world_x, double world_y) const;
  int toIndex(int grid_x, int grid_y) const;
  std::pair<int, int> worldToGrid(double world_x, double world_y) const;
  DwaState motion(const DwaState& state, const DwaControl& control, double dt) const;
  std::vector<double> calcDynamicWindow(const DwaState& state) const;
  std::vector<DwaState> predictTrajectory(const DwaState& state, const DwaControl& control) const;
  double calcToGoalCost(const std::vector<DwaState>& trajectory) const;
  double calcObstacleCost(const std::vector<DwaState>& trajectory) const;
  bool selectControl(const DwaState& state, DwaControl& control,
                     std::vector<DwaState>& trajectory) const;
  void publishCommand(const DwaControl& control) const;
  void publishStop() const;
  void publishTrajectory(const std::vector<DwaState>& trajectory) const;
  double normalizeAngle(double angle) const;

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher path_pub_;
  ros::Timer control_timer_;
  tf::TransformListener tf_listener_;

  nav_msgs::OccupancyGridConstPtr latest_map_;
  nav_msgs::OdometryConstPtr latest_odom_;
  geometry_msgs::PoseStampedConstPtr latest_goal_;
  bool goal_active_;
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
  std::vector<std::pair<double, double>> obstacle_points_;

  double max_speed_;
  double min_speed_;
  double max_yaw_rate_;
  double max_accel_;
  double max_delta_yaw_rate_;
  double velocity_resolution_;
  double yaw_rate_resolution_;
  double dt_;
  double predict_time_;
  double to_goal_cost_gain_;
  double speed_cost_gain_;
  double obstacle_cost_gain_;
  double goal_tolerance_;
  double robot_stuck_flag_cons_;
};

}  // namespace optimal
}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_OPTIMAL_DWA_PLANNER_H_
