<div align="right">

[Chinese](zh/launch_reference.md)

</div>

# Launch Files and Arguments

This page is the detailed launch and argument reference. It is the best place to confirm real launch file names, accepted parameters, and example command combinations.

Important entries include:

- `mr_navigation navigation_sim.launch` for one-command Gazebo + map + AMCL + move_base + RViz.
- `mr_slam slam_sim.launch` for SLAM simulation.
- `mr_traditional_planner planner.launch` for standalone planner debug nodes.
- `mr_traditional_planner planner_sim.launch` for simulation plus planner debugging.
- `mr_learning dqn_train.launch` for the stage 1 DQN experiment.

Always prefer the exact arguments listed in this file over assumptions from ROS conventions.
