# mr_traditional_planner

`mr_traditional_planner` is the traditional-planning package for
`mobile-robot-benchmark`.

调试笔记见 [DEBUG_NOTE.md](/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/DEBUG_NOTE.md)。

## Interface Standard

### Optimal planners

- Subscribe: `/map` (`nav_msgs/OccupancyGrid`)
- Subscribe: `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)
- Publish: `/mr_traditional_planner/optimal_path` (`nav_msgs/Path`)

### Coverage planners

- Subscribe: `/map` (`nav_msgs/OccupancyGrid`)
- Subscribe: `/move_base_simple/goal` (`geometry_msgs/PoseStamped`)
- Publish: `/mr_traditional_planner/coverage_path` (`nav_msgs/Path`)
- Action client: `/move_base` (`move_base_msgs/MoveBaseAction`)

## Launch Switching

- Unified entry: `roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp`
- Full simulation entry: `roslaunch mr_traditional_planner planner_sim.launch algorithm:=stc impl:=py`
- Supported `algorithm`: `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, `rrt_star`, `bcd`, `stc`
- Supported `impl`: `cpp`, `py`
- C++ plugin override: `roslaunch mr_traditional_planner planner.launch impl:=cpp planner_plugin:=mr_traditional_planner/AStarPlanner`
- Built-in C++ plugins:
  - `mr_traditional_planner/AStarPlanner`
  - `mr_traditional_planner/DijkstraPlanner`
  - `mr_traditional_planner/DStarPlanner`
  - `mr_traditional_planner/DStarLitePlanner`
  - `mr_traditional_planner/ThetaStarPlanner`
  - `mr_traditional_planner/RRTStarPlanner`
  - `mr_traditional_planner/BcdPlanner`
  - `mr_traditional_planner/StcPlanner`
- Debug trigger: all algorithms use RViz `2D Nav Goal`
- Coverage note: for `bcd` / `stc`, the clicked point only acts as a trigger signal

## C++ Plugin Extension

New C++ algorithms implement `mr_traditional_planner::PlannerPlugin`, export the class
with `PLUGINLIB_EXPORT_CLASS`, and add one class entry to `planner_plugins.xml`.
After that, switch to it without changing launch files:

```bash
roslaunch mr_traditional_planner planner.launch impl:=cpp planner_plugin:=your_pkg/YourPlanner
```

Common private parameters exposed by the built-in plugins are `map_topic`,
`goal_topic`, `path_topic`, `robot_radius`, `map_frame`, and `robot_frame`.
Coverage plugins also expose `goal_timeout`; BCD exposes `sweep_spacing`, and STC
exposes `tree_spacing`.

## Current Scope

- A* and Dijkstra are implemented in both C++ and Python.
- D*, D* Lite, Theta*, and RRT* are adapted from PythonRobotics and implemented in both C++ and Python.
- BCD coverage is implemented in both C++ and Python.
- STC coverage is implemented in both C++ and Python.
