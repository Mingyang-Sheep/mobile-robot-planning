<div align="right">

[Chinese](zh/robot_models.md)

</div>

# Robot Models and Switching

This page documents the robot model switch system. `robot_model` selects the URDF/Xacro model, while `model` selects navigation parameters such as footprint, costmap, and DWA settings.

Currently documented model keys are `burger`, `waffle`, `waffle_pi`, `wpb_home`, and `wpb_home_mani`. TurtleBot3-style models use `base_scan`; WPB Home models use `laser` as the laser frame. WPB Home and WPB Home Mani keep the official URDF/mesh assets and add a simulation-only adaptation layer for Gazebo diff-drive, laser, camera/depth, and IMU interfaces.

The Mani model is treated as a mobile base for this repository. Arm planning, MoveIt, and grasping are not implemented.
