<div align="right">

[Chinese](zh/configuration_reference.md)

</div>

# Configuration Reference

This page summarizes configuration files and important parameters.

Most navigation parameters live under `src/mr_navigation/config/`, including model-specific costmap files, DWA parameter files, move_base parameters, AMCL settings, and robot model metadata. Planner-related parameters live in `src/mr_traditional_planner/launch/` and planner nodes/plugins.

When changing a robot model, keep URDF frames, laser frame, footprint, costmap parameters, DWA limits, and initial pose consistent.
