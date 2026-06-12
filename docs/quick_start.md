<div align="right">

[Chinese](zh/quick_start.md)

</div>

# Quick Start

This page is the shortest validated way to start the repository. The default demo uses `burger`, `turtlebot3_world`, `map_server`, AMCL, `move_base`, DWA, Gazebo, and RViz.

Basic run sequence:

- Build the workspace with `catkin_make`.
- Source both `/opt/ros/noetic/setup.bash` and `devel/setup.bash`.
- Launch `roslaunch mr_navigation navigation_sim.launch`.
- In RViz, use `2D Pose Estimate` if localization is misaligned and `2D Nav Goal` to send a target.
- Check `/scan`, `/odom`, `/tf`, `/map`, and `/cmd_vel` if the robot does not move.

The default pose is `x=-2.0`, `y=-0.5`, `yaw=0.0`. The Chinese version includes the full topic checklist and success criteria.
