<div align="right">

[Chinese](zh/navigation.md)

</div>

# Navigation

This page explains the AMCL + `move_base` navigation chain.

Main flow:

- Gazebo publishes `/scan`, `/odom`, and TF.
- `map_server` publishes `/map`.
- AMCL publishes the `map -> odom` localization transform.
- `move_base` runs global and local costmaps, a global planner, `DWAPlannerROS`, and outputs `/cmd_vel`.

Use `global_planner:=navfn` for the ROS baseline, or `global_planner:=astar|dijkstra|dstar|dstar_lite|theta_star|rrt_star` for the custom adapter. `cubic_spline` is a smoother, not a global planner.
