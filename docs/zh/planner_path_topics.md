<div align="right">

[英文版](../planner_path_topics.md)

</div>

# Planner Path 话题s 参考入口

这份文件保留为兼容入口。路径话题的详细说明已经合并到下面几份文档：

| 需要的信息 | 请阅读 |
|---|---|
| 调试路径 与 executed path 的区别 | [planner_framework.md](planner_framework.md) |
| `/move_base`、DWA 和规划器 话题 | [topics_and_tf.md](topics_and_tf.md) |
| 普通全局规划器支持矩阵 | [optimal_path_planners.md](optimal_path_planners.md) |
| 覆盖规划路径和 路径点 执行 | [coverage_path_planning.md](coverage_path_planning.md) |

判断机器人实际执行路径时，请优先看 `/mr_traditional_planner/executed_global_path`、DWA 全局/局部规划 和 `/cmd_vel`，不要只看 `/mr_traditional_planner/debug_optimal_path`。
