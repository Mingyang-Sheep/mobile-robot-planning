#ifndef MR_TRADITIONAL_PLANNER_PLANNER_PLUGIN_H_
#define MR_TRADITIONAL_PLANNER_PLANNER_PLUGIN_H_

#include <ros/ros.h>

namespace mr_traditional_planner {

class PlannerPlugin {
 public:
  virtual ~PlannerPlugin() {}

  virtual void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) = 0;
};

}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_PLANNER_PLUGIN_H_
