<div align="right">

[Chinese](zh/installation.md)

</div>

# Installation and Environment Setup

This page describes the environment needed by the current repository: Ubuntu 20.04, ROS Noetic, Gazebo 11, catkin, and Python 3.

Recommended flow:

- Source ROS with `source /opt/ros/noetic/setup.bash`.
- Run `bash tools/check_environment.sh` to inspect the local setup.
- Install ROS dependencies with `sudo bash tools/install_dependencies.sh`.
- Build with `catkin_make` from the workspace root.
- Source `devel/setup.bash` in every terminal that runs ROS commands.

Learning features need `numpy` and optionally `torch`. PyTorch is not forced by the install script because CPU/GPU and CUDA choices vary by machine.
