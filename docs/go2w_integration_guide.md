<div align="right">

[Chinese](zh/go2w_integration_guide.md)

</div>

# Go2W + RL-SAR Integration Guide

This document summarizes the file-level integration behind the
`go2w-navigation` branch. It is intended for developers who need to understand
what was added to the original mobile robot planning workspace to run the Go2W
course bonus demo.

For normal use, start with [go2w_rl_sar_navigation.md](go2w_rl_sar_navigation.md).
For future development of the baseline framework, start from the
[`main` branch](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main).

## Integration Goal

The branch integrates:

- `maze_2.world` from `mr_gazebo`.
- `maze_2_hector` from `mr_maps`.
- Go2W URDF, meshes, Gazebo configuration, and laser frame.
- RL-SAR Gazebo simulation control.
- `policy/go2w/robot_lab/policy.pt`.
- ROS Navigation through `map_server`, AMCL, Navfn, DWA, and RViz goals.
- A filtered velocity bridge from DWA to RL-SAR.
- A planar odometry bridge from Gazebo model states to ROS Navigation.

## Imported Source Sets

| Source set | Target path | Purpose |
|---|---|---|
| RL-SAR controller | `src/rl_sar` | Policy inference, FSM, Gazebo simulation node |
| Robot messages | `src/robot_msgs` | Motor and robot state command messages |
| Gazebo joint controller | `src/robot_joint_controller` | Applies RL-SAR commands to Gazebo joints |
| Go2W description | `src/go2w_description` | URDF, xacro, meshes, Gazebo sensor and control config |
| Go2W policy | `policy/go2w` | Base robot parameters and `robot_lab` TorchScript policy |
| Runtime scripts | `scripts` | LibTorch and ONNX Runtime download helpers |

The Go2W course bonus integration references
[fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar) as the upstream RL-SAR
repository. The local package metadata also records RL-SAR maintainership and
Apache-2.0 license information in `src/rl_sar/package.xml`.

## Custom Files Added for Navigation

| File | Role |
|---|---|
| `src/mr_navigation/launch/go2w_navigation_sim.launch` | Single launch entry for Gazebo, Go2W, RL-SAR, map, AMCL, `move_base`, velocity filter, and RViz |
| `src/mr_navigation/scripts/cmd_vel_filter.py` | Limits, smooths, and republishes DWA velocity commands for RL-SAR |
| `src/mr_gazebo/scripts/gazebo_model_odom.py` | Converts `/gazebo/model_states` into planar `/odom` and `odom -> base_footprint` |
| `src/mr_navigation/config/costmap_common_params_go2w.yaml` | Go2W footprint, obstacle, raytrace, inflation, and `/scan` observation config |
| `src/mr_navigation/config/dwa_local_planner_params_go2w.yaml` | Go2W-specific DWA velocity, acceleration, tolerance, and scoring config |
| `src/mr_maps/maps/maze_2.*` | Maze map set |
| `src/mr_maps/maps/maze_2_hector.*` | Navigation map used by the Go2W launch |

## Existing Launch Files Used by Go2W

| File | Go2W usage |
|---|---|
| `src/mr_maps/launch/map_loader.launch` | Loads `$(find mr_maps)/maps/$(arg map_name).yaml` |
| `src/mr_navigation/launch/navigation.launch` | Connects map, AMCL, and `move_base` |
| `src/mr_navigation/launch/amcl.launch` | Accepts scan topic, odometry model, and initial pose |
| `src/mr_navigation/launch/move_base.launch` | Loads `*_go2w.yaml` files through the `model:=go2w` argument |

## Go2W Launch Composition

`go2w_navigation_sim.launch` starts:

1. Gazebo empty world with `maze_2.world`.
2. Go2W xacro through `robot_description`.
3. `robot_state_publisher`.
4. Static `base_footprint -> base` transform.
5. Gazebo model spawn as `go2w_gazebo`.
6. `go2w_description/config/robot_control.yaml`.
7. `gazebo_model_odom.py`.
8. `rl_sar/rl_sim`.
9. `cmd_vel_filter.py` when enabled.
10. `navigation.launch`.
11. RViz with the existing navigation view.

Important defaults:

```text
model: go2w
map_name: maze_2_hector
world_name: $(find mr_gazebo)/worlds/maze_2.world
robot_name: go2w
gazebo_model_name: go2w_gazebo
x: 1.7
y: 0.8
z: 0.55
yaw: 1.5708
robot_base_height: 0.34
auto_start_rl: true
start_navigation_mode: true
use_cmd_vel_filter: true
```

## Policy and Runtime

The policy directory must contain:

```text
policy/go2w/
|-- base.yaml
`-- robot_lab/
    |-- config.yaml
    `-- policy.pt
```

`base.yaml` defines 16 DoFs, wheel indices, PD gains, joint names, and:

```text
dt: 0.005
decimation: 4
```

The resulting policy interval is `0.02 s`, or 50 Hz. The navigation velocity
interface is intentionally slower by default:

```text
move_base controller loop: 20 Hz
cmd_vel_filter output: 20 Hz
rl_sim cmd_vel input limit: 20 Hz
Gazebo odometry bridge: 30 Hz
laser: 10 Hz
```

LibTorch is downloaded into `library/inference_runtime` by:

```bash
bash scripts/download_inference_runtime.sh libtorch
```

`src/rl_sar/CMakeLists.txt` resolves both `policy` and `library/inference_runtime`
from the workspace root.

## Build Guard for Hardware Targets

The current branch uses:

```bash
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

`src/rl_sar/CMakeLists.txt` defines:

```cmake
option(
  BUILD_RL_REAL_TARGETS
  "Build real-robot hardware executables that require vendor SDKs"
  OFF
)
```

Hardware SDK dependent executables are guarded by that option and by checks for
their SDK paths. This keeps the Gazebo simulation build independent from missing
vendor SDK submodules.

## Package and CMake Adjustments

The imported RL-SAR source set is built inside the same Catkin workspace as the
baseline mobile robot packages. The important integration points are:

| Package | Adjustment |
|---|---|
| `src/rl_sar` | Uses the Catkin package manifest, depends on `robot_msgs` and `robot_joint_controller`, and builds `rl_sim` for the Gazebo chain |
| `src/robot_msgs` | Generates `MotorCommand`, `MotorState`, `RobotCommand`, and `RobotState` messages before dependent targets compile |
| `src/robot_joint_controller` | Links against Catkin libraries and depends on exported message targets |
| `src/go2w_description` | Adds the Gazebo, Gazebo plugin, and xacro dependencies needed by the Go2W model |
| `src/mr_gazebo` | Installs `scripts/gazebo_model_odom.py` as a Python node |
| `src/mr_navigation` | Installs `scripts/cmd_vel_filter.py` as a Python node |

The message generation ordering matters. If `robot_joint_controller` or
`rl_sim` compiles before `robot_msgs` headers are generated, the build can fail
with `robot_msgs/MotorCommand.h` missing. The branch avoids that by adding Catkin
exported target dependencies to the dependent targets.

The Go2W description package needs the simulation and xacro dependencies because
the launch file expands:

```text
$(find go2w_description)/xacro/robot.xacro
```

and the included Gazebo xacro defines the ray sensor, IMU plugin, and control
interfaces used by the demo.

## Laser Integration

`src/go2w_description/xacro/robot.xacro` adds the `base_scan` link on `trunk`:

```text
trunk -> base_scan
origin: xyz="0 0 0.20"
```

`src/go2w_description/xacro/gazebo.xacro` adds a Gazebo ray sensor:

| Property | Value |
|---|---|
| Topic | `/scan` |
| Frame | `base_scan` |
| Update rate | 10 Hz |
| Range | 0.12 m to 10.0 m |
| Horizontal samples | 720 |
| Field of view | 360 degrees |

The Go2W costmap config consumes that laser through `observation_sources: scan`.

## Odometry Bridge

`gazebo_model_odom.py` subscribes to:

```text
/gazebo/model_states
```

It publishes:

```text
/odom
odom -> base_footprint
```

The bridge:

- Finds the Gazebo model by name instead of using a fixed index.
- Uses the model `x`, `y`, and yaw.
- Sets odometry `z` to `0.0`.
- Removes roll and pitch from the navigation pose.
- Converts world-frame linear velocity into the body frame for
  `Odometry.twist`.

This keeps the two-dimensional navigation stack independent from wheel-leg body
height and pitch/roll motion.

## Velocity Filter

`cmd_vel_filter.py` adapts the DWA command before RL-SAR receives it.

Default command route:

```text
move_base cmd_vel remap
-> /move_base_cmd_vel
-> cmd_vel_filter.py
-> /cmd_vel
-> rl_sar / rl_sim
```

Main functions:

- Clamp linear and angular velocities.
- Disable lateral velocity with `max_y=0.0`.
- Smooth acceleration and deceleration.
- Publish at a fixed rate.
- Stop on command timeout.
- Stop after reaching the current goal within `goal_xy_tolerance`.
- Optionally support turn-in-place behavior, disabled by default.

Important defaults from the launch file:

```text
cmd_vel_min_x: 0.16
cmd_vel_max_x: 0.65
cmd_vel_max_yaw: 1.00
cmd_vel_max_acc_x: 0.60
cmd_vel_max_acc_yaw: 1.00
cmd_vel_max_decel_x: 1.50
cmd_vel_max_decel_yaw: 2.00
goal_xy_tolerance: 0.22
goal_stop_enabled: true
goal_stop_latch: true
```

## RL-SAR Navigation Input

The simulation node accepts private parameters from
`go2w_navigation_sim.launch`:

```text
cmd_vel_topic
cmd_vel_input_rate
auto_start
start_navigation
auto_locomotion_delay
```

The node subscribes to `/cmd_vel`, limits the command update rate, and copies the
latest twist command into the policy observation commands:

```text
commands = [linear.x, linear.y, angular.z]
```

The launch file sets `auto_start=true` and `start_navigation=true`, so the demo
does not require manual keyboard state transitions before RViz goal navigation.

## Costmap and DWA Parameters

Go2W costmap parameters:

| Parameter | Value |
|---|---|
| `obstacle_range` | `4.0` |
| `raytrace_range` | `5.0` |
| `footprint` | `[[-0.35,-0.22], [-0.35,0.22], [0.35,0.22], [0.35,-0.22]]` |
| `inflation_radius` | `0.35` |
| `cost_scaling_factor` | `3.0` |
| `scan.sensor_frame` | `base_scan` |
| `scan.topic` | `/scan` |

Go2W DWA parameters:

| Parameter | Value |
|---|---|
| `max_vel_x` | `0.65` |
| `min_vel_x` | `0.16` |
| `max_vel_y` | `0.0` |
| `max_vel_theta` | `1.0` |
| `xy_goal_tolerance` | `0.22` |
| `yaw_goal_tolerance` | `3.14159` |
| `path_distance_bias` | `32.0` |
| `goal_distance_bias` | `24.0` |
| `occdist_scale` | `0.08` |

The DWA parameter file still has:

```text
DWAPlannerROS/controller_frequency: 10.0
```

The launch file overrides the `move_base` controller frequency to 20 Hz. If the
branch is refined later, synchronizing that private DWA value with the launch
argument would be a reasonable cleanup.

## Maps

The Go2W demo uses:

```text
src/mr_gazebo/worlds/maze_2.world
src/mr_maps/maps/maze_2_hector.yaml
src/mr_maps/maps/maze_2_hector.pgm
```

Map YAML files should use relative image paths, for example:

```yaml
image: maze_2_hector.pgm
```

Avoid machine-specific absolute paths so the branch remains portable.

## TurtleBot3 Coexistence

Go2W has its own launch entry:

```bash
roslaunch mr_navigation go2w_navigation_sim.launch
```

The existing TurtleBot3 navigation entry remains separate:

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=burger \
  map_name:=maze_2_hector
```

| Item | TurtleBot3 path | Go2W path |
|---|---|---|
| Model | `mr_description` | `go2w_description` |
| Low-level control | Differential drive plugin | RL-SAR policy |
| Velocity topic | `/cmd_vel` | `/move_base_cmd_vel -> /cmd_vel` |
| Odometry | Differential drive plugin | `gazebo_model_odom.py` |
| Local parameters | `*_burger.yaml` | `*_go2w.yaml` |
| Get-up state | Not required | RL-SAR FSM |

## Validation Checklist

Build:

- `catkin_make -DBUILD_RL_REAL_TARGETS=OFF` succeeds.
- The build does not require missing vendor SDK sources.
- `robot_msgs` headers are generated before dependent targets compile.

Model and control:

- Go2W spawns in Gazebo as `go2w_gazebo`.
- RL-SAR enters locomotion mode automatically.
- Manual `/cmd_vel` can move the robot when no other node publishes to it.

Perception:

- `/scan` publishes at about 10 Hz.
- `/scan.header.frame_id` is `base_scan`.
- Laser points align with maze walls after AMCL pose correction.

Localization and TF:

- `/odom` publishes at about 30 Hz.
- `map -> odom -> base_footprint -> base -> base_scan` is connected.
- RViz RobotModel is not below the map plane.

Navigation:

- Navfn publishes a global plan.
- DWA publishes a local plan.
- `/move_base_cmd_vel` changes after a valid goal.
- `/cmd_vel` is smoothed and has `linear.y = 0`.
- Go2W moves toward the target and stops near the goal.

## References

- [Repository `main` branch](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
- [ROS Navigation stack](http://wiki.ros.org/navigation)
- [fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar)
- RL-SAR project sources imported under `src/rl_sar`
- Go2W / Unitree-related model, control, and policy assets imported under
  `src/go2w_description`, `src/robot_joint_controller`, and `policy/go2w`
