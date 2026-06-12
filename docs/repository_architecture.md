<div align="right">

[Chinese](zh/repository_architecture.md)

</div>

# Repository Architecture

This page explains how the ROS packages fit together.

Package roles:

- `mr_description`: URDF/Xacro, meshes, and `robot_description` launch entries.
- `mr_gazebo`: Gazebo worlds, models, and robot spawning.
- `mr_maps`: static maps and the `map_server` entry.
- `mr_slam`: unified `gmapping` and `hector` launch interface.
- `mr_navigation`: AMCL, `move_base`, DWA, RViz, teleop, and simulation launch orchestration.
- `mr_traditional_planner`: global planners, coverage planners, debug nodes, and `GlobalPlannerAdapter`.
- `mr_learning`: stage 1 DQN experimental demo.
- `mr_msgs`: benchmark-level message package with `PlannerResult.msg`.

The main navigation flow is URDF/Gazebo sensors -> map or SLAM -> AMCL -> `move_base` -> global planner -> DWA -> `/cmd_vel`.
