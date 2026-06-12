<div align="right">

[Chinese](zh/planner_framework.md)

</div>

# Planner Framework

This page explains the difference between point-to-point navigation, coverage planning, debug paths, and executed paths.

For normal navigation, `move_base` calls a global planner and `DWAPlannerROS`, then publishes `/cmd_vel`. For coverage tasks, BCD/STC generate a coverage path and send waypoints through the `/move_base` action interface.

Do not treat `/mr_traditional_planner/debug_optimal_path` as the path the robot is executing. For actual execution, inspect `/mr_traditional_planner/executed_global_path`, DWA global/local plans, and `/cmd_vel`.
