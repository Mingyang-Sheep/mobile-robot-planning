<div align="right">

[Chinese](DEBUG_NOTE_zh.md)

</div>

# mr_traditional_planner Debug Note

This file is kept as a package-local quick entry. The complete and maintained explanations live in the repository-level documentation:

| Topic | Document |
|---|---|
| Planning framework | [../../docs/planner_framework.md](../../docs/planner_framework.md) |
| Point-to-point planners | [../../docs/optimal_path_planners.md](../../docs/optimal_path_planners.md) |
| Coverage planning | [../../docs/coverage_path_planning.md](../../docs/coverage_path_planning.md) |
| Launch arguments | [../../docs/launch_reference.md](../../docs/launch_reference.md) |
| Topics and TF | [../../docs/topics_and_tf.md](../../docs/topics_and_tf.md) |

## Quick Reading

| What you see | Meaning |
|---|---|
| `/mr_traditional_planner/debug_optimal_path` | Standalone debug path; it does not control the robot by default |
| `/mr_traditional_planner/executed_global_path` | Custom global path actually returned to `move_base` |
| `/mr_traditional_planner/coverage_path` | BCD/STC coverage path, later executed as `/move_base` waypoints |
| `/move_base/DWAPlannerROS/local_plan` | Current executable local trajectory |
| `/cmd_vel` | Final velocity command output |

## Minimal Commands

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=py
roslaunch mr_traditional_planner planner_sim.launch algorithm:=astar impl:=cpp
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc
```

Supported `algorithm` keys:

```text
astar
dijkstra
dstar
dstar_lite
theta_star
rrt_star
dwa
cubic_spline
bcd
stc
```

Debugging order: verify `/scan`, `/odom`, `/tf`, and `/map`; then inspect debug paths; finally inspect `executed_global_path`, DWA plans, and `/cmd_vel`.
