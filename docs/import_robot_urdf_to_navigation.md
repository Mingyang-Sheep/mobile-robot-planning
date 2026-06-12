<div align="right">

[Chinese](zh/import_robot_urdf_to_navigation.md)

</div>

# Import an External Robot URDF into Gazebo / SLAM / Navigation

This page is the practical guide for migrating a new robot model into the simulation and navigation chain.

Recommended order:

- Preserve upstream URDF/Xacro/mesh files.
- Add a simulation adaptation layer under `mr_description`.
- Add Gazebo plugins for differential drive, laser, camera/depth, IMU, and odometry as needed.
- Verify topic names and TF frames.
- Add model-specific footprint, costmap, AMCL, and DWA configuration.
- Validate Gazebo first, then SLAM, then Navigation.

The WPB Home migration is the main example in the current repository.
