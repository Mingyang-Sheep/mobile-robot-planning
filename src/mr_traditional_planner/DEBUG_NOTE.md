# mr_traditional_planner 调试入口

这份包内调试笔记已经整理到仓库级文档。请优先阅读：

- [规划框架](../../docs/planner_framework.md)
- [普通路径规划算法](../../docs/optimal_path_planners.md)
- [覆盖路径规划](../../docs/coverage_path_planning.md)
- [Launch 文件与启动参数参考](../../docs/launch_reference.md)
- [Topic 与 TF 参考](../../docs/topics_and_tf.md)

## 当前关键话题

| 话题 | 含义 |
|---|---|
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 调试规划器输出，不代表机器人一定实际执行 |
| `/mr_traditional_planner/executed_global_path` | `GlobalPlannerAdapter` 返回给 `move_base` 的实际全局路径副本 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 覆盖规划路径 |
| `/move_base/DWAPlannerROS/global_plan` | DWA 内部跟踪的全局参考 |
| `/move_base/DWAPlannerROS/local_plan` | DWA 当前局部轨迹 |

## 最小命令

完整仿真 + 调试算法：

```bash
roslaunch mr_traditional_planner planner_sim.launch algorithm:=astar impl:=cpp
```

只启动算法节点：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=py
```

覆盖规划：

```bash
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc
```

## 当前算法 key

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

更完整的启动参数和限制见 [Launch 参考](../../docs/launch_reference.md)。
