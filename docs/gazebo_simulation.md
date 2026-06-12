<div align="right">

[Chinese](zh/gazebo_simulation.md)

</div>

# Gazebo Simulation

This page focuses on Gazebo simulation entries, world files, robot spawning, and sensor/plugin behavior.

Use it when you need to check whether the robot appears in Gazebo, publishes `/scan` and `/odom`, accepts `/cmd_vel`, or needs a different world file. Navigation-ready worlds should have a matching static map when AMCL and `move_base` are used. The repository contains multiple worlds, but only the documented map/world pairs should be assumed ready for Navigation.

For lower resource usage, pass launch arguments such as `gui:=false`, `headless:=true`, or `use_rviz:=false` when supported by the launch file.
