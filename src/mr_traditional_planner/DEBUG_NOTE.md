<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# mr_traditional_planner 调试笔记

这份文件保留为包内快速入口。完整、长期维护的说明请看仓库级文档：

| 主题 | 文档 |
|---|---|
| 规划框架 | [../../docs/planner_framework.md](../../docs/planner_framework.md) |
| 普通路径规划算法 | [../../docs/optimal_path_planners.md](../../docs/optimal_path_planners.md) |
| 覆盖路径规划 | [../../docs/coverage_path_planning.md](../../docs/coverage_path_planning.md) |
| Launch 参数 | [../../docs/launch_reference.md](../../docs/launch_reference.md) |
| Topic 与 TF | [../../docs/topics_and_tf.md](../../docs/topics_and_tf.md) |

## 快速判断

| 你看到的内容 | 含义 |
|---|---|
| `/mr_traditional_planner/debug_optimal_path` | 独立算法调试路径，默认不控制机器人 |
| `/mr_traditional_planner/executed_global_path` | `move_base` 实际拿到的自定义全局路径 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 覆盖路径，会被拆成 waypoint 交给 `/move_base` |
| `/move_base/DWAPlannerROS/local_plan` | 当前局部可执行轨迹 |
| `/cmd_vel` | 最终速度控制输出 |

## 最小命令

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=py
roslaunch mr_traditional_planner planner_sim.launch algorithm:=astar impl:=cpp
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc
```

支持的 `algorithm` key：

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

调试时的原则：先确认 `/scan`、`/odom`、`/tf`、`/map`，再看 debug path，最后看 `executed_global_path`、DWA plan 和 `/cmd_vel`。

---

<a id="english"></a>

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
