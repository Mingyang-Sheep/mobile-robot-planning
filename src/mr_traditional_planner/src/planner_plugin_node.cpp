#include "mr_traditional_planner/planner_plugin.h"

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

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner_plugin_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  const std::string planner_plugin = resolvePlannerPlugin(private_nh);
  if (planner_plugin.empty()) {
    return 1;
  }

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
