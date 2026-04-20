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
- Supported `algorithm`: `astar`, `dijkstra`, `bcd`, `stc`
- Supported `impl`: `cpp`, `py`
- Debug trigger: all algorithms use RViz `2D Nav Goal`
- Coverage note: for `bcd` / `stc`, the clicked point only acts as a trigger signal

## Current Scope

- A* and Dijkstra are implemented in both C++ and Python.
- BCD coverage is implemented in both C++ and Python.
- STC coverage is implemented in both C++ and Python.
