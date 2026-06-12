<div align="right">

[英文版](DEBUG_NOTE.md)

</div>

# mr_traditional_planner 调试笔记

这份文件保留为包内快速入口。完整、长期维护的说明请看仓库级文档：

| 主题 | 文档 |
|---|---|
| 规划框架 | [../../docs/zh/planner_framework.md](../../docs/zh/planner_framework.md) |
| 普通路径规划算法 | [../../docs/zh/optimal_path_planners.md](../../docs/zh/optimal_path_planners.md) |
| 覆盖路径规划 | [../../docs/zh/coverage_path_planning.md](../../docs/zh/coverage_path_planning.md) |
| 启动参数 | [../../docs/zh/launch_reference.md](../../docs/zh/launch_reference.md) |
| 话题与 TF | [../../docs/zh/topics_and_tf.md](../../docs/zh/topics_and_tf.md) |

## 快速判断

| 你看到的内容 | 含义 |
|---|---|
| `/mr_traditional_planner/debug_optimal_path` | 独立算法调试路径，默认不控制机器人 |
| `/mr_traditional_planner/executed_global_path` | `move_base` 实际拿到的自定义全局路径 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 覆盖路径，会被拆成 路径点 交给 `/move_base` |
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

调试时的原则：先确认 `/scan`、`/odom`、`/tf`、`/map`，再看 调试路径，最后看 `executed_global_path`、DWA 规划 和 `/cmd_vel`。
