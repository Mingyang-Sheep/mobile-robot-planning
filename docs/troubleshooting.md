<div align="right">

[Chinese](zh/troubleshooting.md)

</div>

# Troubleshooting

This page collects common failure modes and checks.

Typical problems include missing ROS environment setup, `Resource not found`, plugin loading failures, Gazebo startup issues, absent `/scan` or `/odom`, TF breaks, AMCL/map mismatch, no `/cmd_vel`, invalid planner combinations, and coverage waypoint execution failures.

The debugging order is: confirm the workspace is sourced, confirm packages resolve, confirm launch files expand, inspect topics, inspect TF, then inspect planner outputs and Gazebo behavior.
