<div align="right">

[Chinese](zh/slam_mapping.md)

</div>

# SLAM Mapping

This page describes the current SLAM scope. `mr_slam` provides a unified launch interface for `gmapping` and `hector`.

Typical entry points:

- `roslaunch mr_slam slam.launch slam_method:=gmapping`
- `roslaunch mr_slam slam.launch slam_method:=hector`
- `roslaunch mr_slam slam_sim.launch slam_method:=gmapping`

The current SLAM support is intended for basic mapping, course validation, and topic/TF learning. It is not presented as an advanced multi-sensor SLAM framework.
