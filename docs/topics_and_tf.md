<div align="right">

[Chinese](zh/topics_and_tf.md)

</div>

# Topics and TF

This page collects the important ROS topics and TF frames used by the workspace.

Typical checks include `/scan`, `/odom`, `/tf`, `/tf_static`, `/map`, `/cmd_vel`, `/move_base/*`, `/mr_traditional_planner/debug_optimal_path`, `/mr_traditional_planner/executed_global_path`, and `/mr_traditional_planner/coverage_path`.

The expected TF chain for navigation is usually `map -> odom -> base_footprint -> base_link -> sensor frames`. When navigation fails, verify topics first, then TF, then planner outputs.
