<div align="right">

[Chinese](README_zh.md)

</div>

# mr_slam

`mr_slam` is the unified SLAM launch/config package for this workspace. It integrates `gmapping` and `hector` as normal ROS dependencies and is mainly used for basic mapping, course validation, and topic/TF learning.

Full topic documentation is in [../../docs/slam_mapping.md](../../docs/slam_mapping.md).

## Current Backends

| Argument value | Expected node | Notes |
|---|---|---|
| `gmapping` | `/slam_gmapping` | Uses 2D laser and odometry; suitable for the default simulation mapping flow |
| `hector` | `/hector_mapping` | Useful for comparing SLAM backend topic/TF behavior |

## Minimal Commands

```bash
roslaunch mr_slam slam.launch slam_method:=gmapping
roslaunch mr_slam slam.launch slam_method:=hector
```

One-command Gazebo entries:

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
roslaunch mr_slam slam_sim.launch slam_method:=hector
```

For lower load, disable the Gazebo GUI or RViz when the launch file supports it:

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping gui:=false
roslaunch mr_slam slam_sim.launch slam_method:=gmapping use_rviz:=false
```

## Common Arguments

| Argument | Default |
|---|---|
| `scan_topic` | `/scan` |
| `odom_topic` | `/odom` |
| `base_frame` | `base_footprint` |
| `odom_frame` | `odom` |
| `map_frame` | `map` |

## Quick Checks

After building and sourcing the workspace:

```bash
rospack find mr_slam
roslaunch --nodes mr_slam slam.launch slam_method:=gmapping
roslaunch --nodes mr_slam slam.launch slam_method:=hector
```

During runtime, inspect:

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /tf
rostopic echo -n 1 /map
rosrun tf tf_echo map base_footprint
```

This package should not be described as an advanced multi-sensor SLAM framework. Its current scope is basic 2D SLAM integration and teaching-oriented validation.
