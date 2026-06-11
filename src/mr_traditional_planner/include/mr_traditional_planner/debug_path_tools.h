#ifndef MR_TRADITIONAL_PLANNER_DEBUG_PATH_TOOLS_H_
#define MR_TRADITIONAL_PLANNER_DEBUG_PATH_TOOLS_H_

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <cstddef>
#include <string>

namespace mr_traditional_planner {

inline void publishEmptyDebugPath(const ros::Publisher& path_pub, const std::string& frame_id,
                                  const std::string& algorithm, const std::string& impl,
                                  const std::string& topic, const std::string& reason) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = frame_id.empty() ? std::string("map") : frame_id;
  path_pub.publish(path_msg);
  const std::string message = "[DebugPlanner] algorithm=" + algorithm + " impl=" + impl +
                              " success=false points=0 reason=" + reason + " topic=" + topic;
  if (reason == "startup_clear") {
    ROS_INFO_STREAM(message);
  } else {
    ROS_WARN_STREAM(message);
  }
}

inline void logDebugPathSuccess(const std::string& algorithm, const std::string& impl,
                                const std::string& topic, std::size_t points,
                                const std::string& reason = "ok") {
  ROS_INFO_STREAM("[DebugPlanner] algorithm=" << algorithm << " impl=" << impl
                                              << " success=true points=" << points
                                              << " reason=" << reason << " topic=" << topic);
}

}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_DEBUG_PATH_TOOLS_H_
