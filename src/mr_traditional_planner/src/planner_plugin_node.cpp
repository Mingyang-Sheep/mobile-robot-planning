#include "mr_traditional_planner/planner_plugin.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pluginlib/class_loader.hpp>
#include <ros/ros.h>

#include <map>
#include <string>
#include <utility>

namespace {

std::string builtinPluginForAlgorithm(const std::string& algorithm) {
  static const std::map<std::string, std::string> kBuiltinPlugins = {
      std::make_pair("astar", "mr_traditional_planner/AStarPlanner"),
      std::make_pair("dijkstra", "mr_traditional_planner/DijkstraPlanner"),
      std::make_pair("dstar", "mr_traditional_planner/DStarPlanner"),
      std::make_pair("dstar_lite", "mr_traditional_planner/DStarLitePlanner"),
      std::make_pair("theta_star", "mr_traditional_planner/ThetaStarPlanner"),
      std::make_pair("rrt_star", "mr_traditional_planner/RRTStarPlanner"),
      std::make_pair("dwa", "mr_traditional_planner/DynamicWindowApproachPlanner"),
      std::make_pair("cubic_spline", "mr_traditional_planner/CubicSplinePlanner"),
      std::make_pair("bcd", "mr_traditional_planner/BcdPlanner"),
      std::make_pair("stc", "mr_traditional_planner/StcPlanner"),
  };

  const std::map<std::string, std::string>::const_iterator plugin_it =
      kBuiltinPlugins.find(algorithm);
  if (plugin_it == kBuiltinPlugins.end()) {
    return std::string();
  }

  return plugin_it->second;
}

std::string resolvePlannerPlugin(ros::NodeHandle& private_nh) {
  std::string planner_plugin;
  private_nh.param<std::string>("planner_plugin", planner_plugin, std::string());
  if (!planner_plugin.empty()) {
    return planner_plugin;
  }

  std::string algorithm;
  private_nh.param<std::string>("algorithm", algorithm, std::string("astar"));
  planner_plugin = builtinPluginForAlgorithm(algorithm);
  if (!planner_plugin.empty()) {
    return planner_plugin;
  }

  ROS_FATAL_STREAM("Unknown traditional planner algorithm '" << algorithm
                   << "'. Set ~planner_plugin to a registered pluginlib class.");
  return std::string();
}

}  // namespace

class DebugPathSupervisor {
 public:
  void initialize(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
    private_nh.param<std::string>("algorithm", algorithm_, std::string("unknown"));
    private_nh.param<std::string>("path_topic", path_topic_,
                                  std::string("/mr_traditional_planner/debug_optimal_path"));
    private_nh.param<std::string>("goal_topic", goal_topic_,
                                  std::string("/move_base_simple/goal"));
    path_pub_ = nh.advertise<nav_msgs::Path>(path_topic_, 1, true);
    path_sub_ = nh.subscribe(path_topic_, 10, &DebugPathSupervisor::pathCallback, this);
    goal_sub_ = nh.subscribe(goal_topic_, 1, &DebugPathSupervisor::goalCallback, this);
    publishEmptyPath("map");
    ros::spinOnce();
    ROS_INFO_STREAM("[planner_plugin_node] cleared debug path topic: " << path_topic_);
  }

 private:
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
    const std::string frame_id =
        msg->header.frame_id.empty() ? std::string("map") : msg->header.frame_id;
    publishEmptyPath(frame_id);
  }

  void publishEmptyPath(const std::string& frame_id) {
    nav_msgs::Path empty_path;
    empty_path.header.stamp = ros::Time::now();
    empty_path.header.frame_id = frame_id;
    path_pub_.publish(empty_path);
  }

  void pathCallback(const nav_msgs::PathConstPtr& msg) {
    ROS_INFO_STREAM("[DebugPlanner] algorithm=" << algorithm_ << " topic=" << path_topic_
                                                << " success="
                                                << (msg->poses.empty() ? "false" : "true")
                                                << " points=" << msg->poses.size()
                                                << " frame=" << msg->header.frame_id);
  }

  std::string algorithm_;
  std::string path_topic_;
  std::string goal_topic_;
  ros::Publisher path_pub_;
  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_plugin_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  const std::string planner_plugin = resolvePlannerPlugin(private_nh);
  if (planner_plugin.empty()) {
    return 1;
  }
  DebugPathSupervisor debug_path_supervisor;
  debug_path_supervisor.initialize(nh, private_nh);

  try {
    pluginlib::ClassLoader<mr_traditional_planner::PlannerPlugin> loader(
        "mr_traditional_planner", "mr_traditional_planner::PlannerPlugin");
    boost::shared_ptr<mr_traditional_planner::PlannerPlugin> planner =
        loader.createInstance(planner_plugin);
    planner->initialize(nh, private_nh);

    ROS_INFO_STREAM("Loaded traditional planner plugin: " << planner_plugin);
    ros::spin();
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("Failed to load traditional planner plugin '" << planner_plugin
                     << "': " << ex.what());
    return 1;
  }

  return 0;
}
