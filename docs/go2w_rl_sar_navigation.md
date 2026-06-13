<div align="right">

[Chinese](zh/go2w_rl_sar_navigation.md)

</div>

# Go2W RL-SAR Navigation Guide

This document is the user-facing guide for running the Go2W course bonus demo in
the `go2w-navigation` branch. It focuses on build, launch, verification, and
troubleshooting.

For the broader baseline framework and future development, start from the
[`main` branch](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main).

## Scope

This branch runs a Gazebo simulation chain:

```text
Go2W model
-> Gazebo laser and model states
-> AMCL + move_base
-> NavfnROS + DWAPlannerROS
-> cmd_vel_filter.py
-> rl_sar / rl_sim
-> policy/go2w/robot_lab/policy.pt
-> robot_joint_controller
```

It is not a real robot deployment guide and does not cover policy training.

## Build

```bash
cd ~/mobile-robot-planning
source /opt/ros/noetic/setup.bash

bash scripts/download_inference_runtime.sh libtorch
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
source devel/setup.bash
```

The build flag disables hardware executables that depend on vendor SDKs. The
Gazebo simulation node `rl_sar/rl_sim` remains part of the build.

## Launch

```bash
roslaunch mr_navigation go2w_navigation_sim.launch
```

The default launch path uses:

| Item | Value |
|---|---|
| World | `$(find mr_gazebo)/worlds/maze_2.world` |
| Map | `maze_2_hector` |
| Robot key | `go2w` |
| Gazebo model name | `go2w_gazebo` |
| Initial pose | `x=1.7`, `y=0.8`, `z=0.55`, `yaw=1.5708` |
| Base height TF | `base_footprint -> base`, `z=0.34` |
| RL policy | `policy/go2w/robot_lab/policy.pt` |
| Raw velocity topic | `/move_base_cmd_vel` |
| Filtered velocity topic | `/cmd_vel` |

## RViz Workflow

1. Wait for Gazebo to load `maze_2.world`.
2. Wait for the Go2W model and controllers to spawn.
3. Wait for RL-SAR to complete get-up and enter locomotion mode.
4. Check that AMCL, `move_base`, and RViz are running.
5. If the pose is not aligned with the map, use `2D Pose Estimate`.
6. Send a target with `2D Nav Goal`.

## Common Launch Arguments

Set the initial pose:

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  x:=1.7 y:=0.8 z:=0.55 yaw:=1.5708
```

Start without RViz:

```bash
roslaunch mr_navigation go2w_navigation_sim.launch rviz:=false
```

Adjust the automatic locomotion delay:

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  auto_locomotion_delay:=9.0
```

Change the shared navigation command rate:

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  rl_cmd_vel_input_rate:=10.0
```

Compare behavior without the velocity filter:

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  use_cmd_vel_filter:=false
```

That mode is useful for diagnosis only; the filtered path is the default demo
path.

## Data Flow

Navigation command flow:

```text
RViz 2D Nav Goal
-> NavfnROS global plan
-> DWAPlannerROS local command
-> /move_base_cmd_vel
-> cmd_vel_filter.py
-> /cmd_vel
-> rl_sar / rl_sim
-> policy.pt
-> RobotCommand / MotorCommand
-> robot_joint_controller
-> Gazebo Go2W
```

Localization and feedback flow:

```text
/gazebo/model_states
-> gazebo_model_odom.py
-> /odom
-> odom -> base_footprint
-> AMCL
-> map -> odom
```

Perception flow:

```text
Gazebo ray sensor on base_scan
-> /scan
-> AMCL
-> global_costmap and local_costmap
```

The expected TF chain is:

```text
map
-> odom
-> base_footprint
-> base
-> trunk
-> base_scan
```

## Runtime Rates

| Signal | Expected rate |
|---|---:|
| `/scan` | About 10 Hz |
| `/odom` | About 30 Hz |
| `move_base` controller loop | 20 Hz by launch argument |
| `cmd_vel_filter.py` output | 20 Hz by default |
| RL-SAR policy inference | 50 Hz from `dt=0.005` and `decimation=4` |

The RL policy can reuse the latest navigation command between command updates.
That is expected behavior.

## Verification Commands

Check nodes:

```bash
rosnode list | grep -E "gazebo|rl_sar|cmd_vel|amcl|move_base|map_server|robot_state"
```

Expected key nodes include:

```text
/gazebo
/rl_sar
/go2w_cmd_vel_filter
/go2w_gazebo_odom
/amcl
/move_base
/map_server
/robot_state_publisher
```

Check topics:

```bash
rostopic list | grep -E "/scan|/odom|cmd_vel|model_states|move_base"
```

Check rates:

```bash
rostopic hz /scan
rostopic hz /odom
rostopic hz /move_base_cmd_vel
rostopic hz /cmd_vel
```

Compare raw and filtered velocity commands:

```bash
rostopic echo /move_base_cmd_vel
rostopic echo /cmd_vel
```

Check TF:

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint base
rosrun tf tf_echo base base_scan
```

Check plans:

```bash
rostopic echo /move_base/NavfnROS/plan
rostopic echo /move_base/DWAPlannerROS/local_plan
```

## Manual `/cmd_vel` Test

Use this only when `move_base` and the velocity filter are not also publishing to
`/cmd_vel`.

```bash
rostopic pub -r 4 /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.30
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.20"
```

Recommended manual range:

```text
linear.x: 0.20 to 0.50
linear.y: 0.0
angular.z: -0.50 to 0.50
rate: 4 to 20 Hz
```

Check publishers before manual control:

```bash
rostopic info /cmd_vel
```

## Troubleshooting

### `unitree_sdk2` missing `CMakeLists.txt`

**Reason:** The hardware SDK source is not required for this Gazebo demo, but a
hardware target may try to include it if not disabled.

**Fix:**

```bash
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

### `robot_msgs/MotorCommand.h` is missing

**Reason:** Generated message headers were not available before dependent
targets compiled.

**Fix:** Confirm `src/robot_msgs/msg/MotorCommand.msg` exists, then rebuild. If a
stale CMake cache is suspected, clean `build` and `devel` first.

```bash
rm -rf build devel
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

### Go2W moves unpredictably

**Reason:** Common causes are multiple `/cmd_vel` publishers, RL-SAR not yet in
locomotion mode, an incorrect Gazebo model name, or very jumpy velocity commands.

**Fix:**

```bash
rostopic info /cmd_vel
rostopic echo /cmd_vel
rosparam get /gazebo_model_name
```

The default Gazebo model name should be:

```text
go2w_gazebo
```

### Laser does not match the map

**Reason:** The laser is a live Gazebo measurement, not the static map. Mismatch
usually comes from AMCL initial pose, map origin, or TF alignment.

**Fix:**

```bash
rostopic echo -n 1 /scan/header
rosrun tf tf_echo map base_scan
```

Use RViz `2D Pose Estimate` if the robot pose is visibly offset.

### Robot barely moves after a valid goal

**Reason:** DWA may output low speeds, or the filter may clamp the command before
the RL policy receives it.

**Fix:** Compare:

```bash
rostopic echo /move_base_cmd_vel
rostopic echo /cmd_vel
```

If `/move_base_cmd_vel` is nonzero but `/cmd_vel` is too small, adjust the launch
parameters for `cmd_vel_filter.py`. If `/move_base_cmd_vel` is almost zero, check
AMCL, local costmap, and the RViz goal.

### Target is reached but rotation continues

**Reason:** Strict final yaw alignment is not useful for this wheel-leg maze
demo.

**Fix:** Keep the relaxed yaw tolerance and goal-stop latch used by this branch:

```text
xy_goal_tolerance: 0.22
yaw_goal_tolerance: 3.14159
goal_stop_enabled: true
goal_stop_latch: true
```

### RobotModel appears below the map plane in RViz

**Reason:** Two-dimensional navigation should use a planar `base_footprint`,
while the URDF root link is offset above it.

**Fix:**

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint base
```

Expected values:

```text
odom -> base_footprint: z = 0
base_footprint -> base: z = 0.34
```

## Notes and Limits

- This branch demonstrates a Gazebo-only Go2W navigation loop.
- The RL-SAR policy is pretrained and loaded from `policy/go2w/robot_lab`.
- The branch does not retrain the policy.
- The branch does not replace `main` as the recommended development entry.
- The current default uses forward motion plus yaw; lateral velocity is clamped
  to `0.0` in the launch configuration.

## References

- [Repository `main` branch](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
- [ROS Navigation stack](http://wiki.ros.org/navigation)
- [fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar), the upstream
  reference repository for the RL-SAR part of the Go2W course bonus integration
- Local RL-SAR and Go2W assets in `src/rl_sar`, `src/go2w_description`,
  `policy/go2w`, and `src/robot_joint_controller`
