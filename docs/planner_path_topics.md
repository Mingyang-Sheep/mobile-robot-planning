<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# Planner Path Topics 参考入口

这份文件保留为兼容入口。路径话题的详细说明已经合并到下面几份文档：

| 需要的信息 | 请阅读 |
|---|---|
| debug path 与 executed path 的区别 | [planner_framework.md](planner_framework.md) |
| `/move_base`、DWA 和规划器 topic | [topics_and_tf.md](topics_and_tf.md) |
| 普通全局规划器支持矩阵 | [optimal_path_planners.md](optimal_path_planners.md) |
| 覆盖规划路径和 waypoint 执行 | [coverage_path_planning.md](coverage_path_planning.md) |

判断机器人实际执行路径时，请优先看 `/mr_traditional_planner/executed_global_path`、DWA global/local plan 和 `/cmd_vel`，不要只看 `/mr_traditional_planner/debug_optimal_path`。

---

<a id="english"></a>

# Planner Path Topics Entry

This file is kept as a compatibility entry. Detailed path-topic explanations have been merged into the documents below:

| Information needed | Read |
|---|---|
| Difference between debug paths and executed paths | [planner_framework.md](planner_framework.md) |
| `/move_base`, DWA, and planner topics | [topics_and_tf.md](topics_and_tf.md) |
| Point-to-point global planner support matrix | [optimal_path_planners.md](optimal_path_planners.md) |
| Coverage paths and waypoint execution | [coverage_path_planning.md](coverage_path_planning.md) |

When checking the path actually followed by the robot, inspect `/mr_traditional_planner/executed_global_path`, DWA global/local plans, and `/cmd_vel`; do not rely only on `/mr_traditional_planner/debug_optimal_path`.
