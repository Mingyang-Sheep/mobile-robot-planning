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

### Local controllers and smoothers

- `dwa` subscribes `/map`, `/odom`, and `/move_base_simple/goal`; publishes `/cmd_vel`
  and the selected preview trajectory on `/mr_traditional_planner/optimal_path`.
- `cubic_spline` publishes a smoothed `nav_msgs/Path` to
  `/mr_traditional_planner/optimal_path`; it can use either RViz `2D Nav Goal` or
  `input_path_topic`.

## Launch Switching

- Unified entry: `roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp`
- Full simulation entry: `roslaunch mr_traditional_planner planner_sim.launch algorithm:=stc impl:=py`
- Supported `algorithm`: `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, `rrt_star`, `dwa`, `cubic_spline`, `bcd`, `stc`
- Supported `impl`: `cpp`, `py`
- C++ plugin override: `roslaunch mr_traditional_planner planner.launch impl:=cpp planner_plugin:=mr_traditional_planner/AStarPlanner`
- Built-in C++ plugins:
  - `mr_traditional_planner/AStarPlanner`
  - `mr_traditional_planner/DijkstraPlanner`
  - `mr_traditional_planner/DStarPlanner`
  - `mr_traditional_planner/DStarLitePlanner`
  - `mr_traditional_planner/ThetaStarPlanner`
  - `mr_traditional_planner/RRTStarPlanner`
  - `mr_traditional_planner/DynamicWindowApproachPlanner`
  - `mr_traditional_planner/CubicSplinePlanner`
  - `mr_traditional_planner/BcdPlanner`
  - `mr_traditional_planner/StcPlanner`
- Debug trigger: all algorithms use RViz `2D Nav Goal`
- Coverage note: for `bcd` / `stc`, the clicked point only acts as a trigger signal

## TurtleBot move_base Execution

The standalone optimal planners can be used as the global path source for
TurtleBot while `move_base` continues to use `DWAPlannerROS` as its local
controller:

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=burger \
  map_name:=maze_2_hector \
  use_traditional_global_planner:=true \
  traditional_algorithm:=astar \
  traditional_impl:=cpp \
  traditional_robot_radius:=0.25
```

Increase `traditional_robot_radius` to keep the global path farther from walls.
For TurtleBot3 Burger, start with `0.25`; use `0.30` if the local controller
still clips corners. A* also rejects diagonal moves that would cut through an
occupied corner.

Replace `astar` with one of:

```text
astar
dijkstra
dstar
dstar_lite
theta_star
rrt_star
```

In this mode, `PathTopicGlobalPlanner` sends a planning request to the selected
standalone planner, receives `/mr_traditional_planner/optimal_path`, and returns
that path to `move_base`. The existing local DWA planner then follows the path
and publishes velocity commands.

Do not select `dwa`, `bcd`, or `stc` through this global-path mode. The custom
`dwa` implementation is itself a velocity controller, while `bcd` and `stc`
produce coverage paths with different execution semantics.

The Go2W launch explicitly keeps `navfn/NavfnROS`, so this TurtleBot option does
not change the Go2W navigation stack.

## C++ Plugin Extension

New C++ algorithms implement `mr_traditional_planner::PlannerPlugin`, export the class
with `PLUGINLIB_EXPORT_CLASS`, and add one class entry to `planner_plugins.xml`.
After that, switch to it without changing launch files:

```bash
roslaunch mr_traditional_planner planner.launch impl:=cpp planner_plugin:=your_pkg/YourPlanner
```

Common private parameters exposed by the built-in plugins are `map_topic`,
`goal_topic`, `path_topic`, `robot_radius`, `map_frame`, and `robot_frame`.
DWA additionally exposes `odom_topic`, `cmd_vel_topic`, and sampling/control
parameters such as `max_speed`, `predict_time`, and `control_frequency`.
Cubic Spline additionally exposes `input_path_topic`, `spline_resolution`,
`control_point_ratio`, and `collision_check`.
Coverage plugins also expose `goal_timeout`; BCD exposes `sweep_spacing`, and STC
exposes `tree_spacing`.

## Current Scope

- A* and Dijkstra are implemented in both C++ and Python.
- D*, D* Lite, Theta*, RRT*, Dynamic Window Approach, and Cubic Spline are adapted from PythonRobotics and implemented in both C++ and Python.
- BCD coverage is implemented in both C++ and Python.
- STC coverage is implemented in both C++ and Python.
