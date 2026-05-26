# mr_slam

`mr_slam` is the unified SLAM launch/config package for this workspace. It owns
the benchmark-facing launch interface and keeps third-party SLAM backends as
normal ROS package dependencies.

## Supported Backends

- `gmapping`
- `hector`

## Build

The examples below use `<workspace_root>` for the repository root.

```bash
cd <workspace_root>
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

Install dependencies with `tools/list_requirements.sh` as the checklist. The
SLAM-specific ROS packages are `ros-noetic-gmapping` and
`ros-noetic-hector-mapping`.

## Launch Interface

All SLAM backends use the same entrypoint:

```bash
roslaunch mr_slam slam.launch slam_method:=gmapping
roslaunch mr_slam slam.launch slam_method:=hector
```

Common arguments:

- `scan_topic`, default `/scan`
- `odom_topic`, default `/odom`
- `base_frame`, default `base_footprint`
- `odom_frame`, default `odom`
- `map_frame`, default `map`

One-command simulation entry:

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
roslaunch mr_slam slam_sim.launch slam_method:=hector
roslaunch mr_slam slam_sim.launch slam_method:=gmapping world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_1.world x:=1.7 y:=1.0
roslaunch mr_slam slam_sim.launch slam_method:=gmapping world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_1.world x:=1.7 y:=1.0 wheel_mu:=1.5
```

For lower CPU/GPU load while mapping:

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_1.world x:=1.7 y:=1.0 gui:=false
roslaunch mr_slam slam_sim.launch slam_method:=gmapping world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_1.world x:=1.7 y:=1.0 use_rviz:=false
```

## Static Checks

After building and sourcing the workspace:

```bash
rospack find mr_slam
roslaunch --nodes mr_slam slam.launch slam_method:=gmapping
roslaunch --nodes mr_slam slam.launch slam_method:=hector
```

Expected nodes:

- `gmapping`: `/slam_gmapping`
- `hector`: `/hector_mapping`

## Runtime Smoke Test

Start one backend:

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
```

Drive the robot, then inspect:

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /tf
rostopic echo -n 1 /map
rosrun tf tf_echo map base_footprint
```

The backend is basically running if `/scan` is publishing, `/map` receives data,
and `map -> base_footprint` is available while the robot moves.
