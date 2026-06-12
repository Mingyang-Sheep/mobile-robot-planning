#ifndef MR_TRADITIONAL_PLANNER_DEBUG_PATH_TOOLS_H_
#define MR_TRADITIONAL_PLANNER_DEBUG_PATH_TOOLS_H_

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace mr_traditional_planner {

inline void logDebugNodeStart(const std::string& algorithm, const std::string& impl,
                              const std::string& executable, const std::string& plugin,
                              const std::string& path_topic) {
  ROS_INFO_STREAM("[DebugPlanner] node_start script=" << executable << " node="
                                                       << ros::this_node::getName()
                                                       << " algorithm=" << algorithm
                                                       << " impl=" << impl
                                                       << " plugin=" << plugin
                                                       << " path_topic=" << path_topic);
}

inline void logDebugAlgorithmSelected(const std::string& algorithm, const std::string& impl) {
  ROS_INFO_STREAM("[DebugPlanner] algorithm_selected=" << algorithm << " impl=" << impl);
}

inline void logDebugModuleLoaded(const std::string& algorithm, const std::string& impl,
                                 const std::string& plugin, const void* object_ptr) {
  ROS_INFO_STREAM("[DebugPlanner] module_loaded path=" << plugin << " class=" << plugin
                                                       << " object_id=" << object_ptr
                                                       << " algorithm=" << algorithm
                                                       << " impl=" << impl);
}

inline void logDebugSubscriptionsReady(const std::string& algorithm, const std::string& impl,
                                       const std::string& map_topic,
                                       const std::string& goal_topic, const void* object_ptr,
                                       const std::string& map_callback,
                                       const std::string& goal_callback) {
  ROS_INFO_STREAM("[DebugPlanner] subscriptions_ready map=" << map_topic << " goal="
                                                            << goal_topic << " object_id="
                                                            << object_ptr << " algorithm="
                                                            << algorithm << " impl=" << impl
                                                            << " map_callback=" << map_callback
                                                            << " goal_callback=" << goal_callback);
}

inline void logDebugCallbackEnter(const std::string& callback_name, const std::string& algorithm,
                                  const std::string& impl, const void* object_ptr,
                                  const std::string& function_name) {
  ROS_INFO_STREAM("[DebugPlanner] " << callback_name << "_enter object_id=" << object_ptr
                                    << " algorithm=" << algorithm << " impl=" << impl
                                    << " function=" << function_name);
}

inline void logDebugMapReady(const std::string& algorithm, const std::string& impl,
                             const std::string& topic, int width, int height, double resolution,
                             const std::string& frame_id) {
  ROS_INFO_STREAM("[DebugPlanner] map_ready width=" << width << " height=" << height
                                                    << " resolution=" << resolution
                                                    << " algorithm=" << algorithm
                                                    << " impl=" << impl << " topic=" << topic
                                                    << " frame=" << frame_id);
}

inline void logDebugGoalReceived(const std::string& algorithm, const std::string& impl,
                                 const std::string& topic, const std::string& frame_id,
                                 double x, double y, bool map_ready) {
  ROS_INFO_STREAM("[DebugPlanner] goal_received x=" << x << " y=" << y << " frame="
                                                    << frame_id << " algorithm=" << algorithm
                                                    << " impl=" << impl << " topic=" << topic
                                                    << " map_ready="
                                                    << (map_ready ? "true" : "false"));
}

inline bool lookupDebugStartPose(tf::TransformListener& tf_listener,
                                 const std::string& map_frame,
                                 const std::string& configured_robot_frame,
                                 const std::string& algorithm, const std::string& impl,
                                 double timeout, double& start_world_x,
                                 double& start_world_y) {
  std::vector<std::string> robot_frames;
  const std::vector<std::string> candidates = {configured_robot_frame, "base_footprint",
                                               "base_link"};
  for (const std::string& candidate : candidates) {
    if (candidate.empty()) {
      continue;
    }
    bool seen = false;
    for (const std::string& frame : robot_frames) {
      if (frame == candidate) {
        seen = true;
        break;
      }
    }
    if (!seen) {
      robot_frames.push_back(candidate);
    }
  }

  std::vector<std::string> errors;
  for (const std::string& robot_frame : robot_frames) {
    tf::StampedTransform transform;
    try {
      tf_listener.waitForTransform(map_frame, robot_frame, ros::Time(0), ros::Duration(timeout));
      tf_listener.lookupTransform(map_frame, robot_frame, ros::Time(0), transform);
      start_world_x = transform.getOrigin().x();
      start_world_y = transform.getOrigin().y();
      ROS_INFO_STREAM("[DebugPlanner] tf_success base_frame=" << robot_frame
                                                              << " start_x=" << start_world_x
                                                              << " start_y=" << start_world_y
                                                              << " algorithm=" << algorithm
                                                              << " impl=" << impl
                                                              << " map_frame=" << map_frame);
      return true;
    } catch (tf::TransformException& ex) {
      errors.push_back(robot_frame + ": " + ex.what());
    }
  }

  std::string joined_errors;
  for (std::size_t index = 0; index < errors.size(); ++index) {
    if (index != 0U) {
      joined_errors += " | ";
    }
    joined_errors += errors[index];
  }
  ROS_WARN_STREAM("[DebugPlanner] algorithm=" << algorithm << " impl=" << impl
                                              << " success=false points=0"
                                              << " reason=tf_lookup_failed map_frame="
                                              << map_frame << " errors=" << joined_errors);
  return false;
}

inline void logDebugGridPoints(const std::string& algorithm, const std::string& impl,
                               int start_x, int start_y, int goal_x, int goal_y) {
  ROS_INFO_STREAM("[DebugPlanner] grid_start=(" << start_x << "," << start_y
                                                << ") grid_goal=(" << goal_x << "," << goal_y
                                                << ") algorithm=" << algorithm
                                                << " impl=" << impl);
}

inline void logDebugPlanCall(const std::string& algorithm, const std::string& impl) {
  ROS_INFO_STREAM("[DebugPlanner] plan_call algorithm=" << algorithm << " impl=" << impl);
}

inline void logDebugPlanReturn(const std::string& algorithm, const std::string& impl,
                               std::size_t points) {
  ROS_INFO_STREAM("[DebugPlanner] plan_return points=" << points << " algorithm=" << algorithm
                                                       << " impl=" << impl);
}

inline void publishEmptyDebugPath(const ros::Publisher& path_pub, const std::string& frame_id,
                                  const std::string& algorithm, const std::string& impl,
                                  const std::string& topic, const std::string& reason) {
  nav_msgs::Path path_msg;
  path_msg.header.stamp = ros::Time::now();
  path_msg.header.frame_id = frame_id.empty() ? std::string("map") : frame_id;
  path_pub.publish(path_msg);
  ROS_INFO_STREAM("[DebugPlanner] publish topic=" << topic << " points=0 frame="
                                                  << path_msg.header.frame_id
                                                  << " algorithm=" << algorithm
                                                  << " impl=" << impl << " reason=" << reason);
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
                                const std::string& frame_id = "map",
                                const std::string& reason = "ok") {
  ROS_INFO_STREAM("[DebugPlanner] publish topic=" << topic << " points=" << points
                                                  << " frame=" << frame_id
                                                  << " algorithm=" << algorithm
                                                  << " impl=" << impl);
  ROS_INFO_STREAM("[DebugPlanner] algorithm=" << algorithm << " impl=" << impl
                                              << " success=true points=" << points
                                              << " reason=" << reason << " topic=" << topic);
}

}  // namespace mr_traditional_planner

#endif  // MR_TRADITIONAL_PLANNER_DEBUG_PATH_TOOLS_H_
