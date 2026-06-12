<div align="right">

[Chinese](zh/planner_path_topics.md)

</div>

# Planner Path Topics Entry

This file is kept as a compatibility entry. Detailed path-topic explanations have been merged into the documents below:

| Information needed | Read |
|---|---|
| Difference between debug paths and executed paths | [planner_framework.md](planner_framework.md) |
| `/move_base`, DWA, and planner topics | [topics_and_tf.md](topics_and_tf.md) |
| Point-to-point global planner support matrix | [optimal_path_planners.md](optimal_path_planners.md) |
| Coverage paths and waypoint execution | [coverage_path_planning.md](coverage_path_planning.md) |

When checking the path actually followed by the robot, inspect `/mr_traditional_planner/executed_global_path`, DWA global/local plans, and `/cmd_vel`; do not rely only on `/mr_traditional_planner/debug_optimal_path`.
