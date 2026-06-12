# 常见问题与排错

适合读者：launch 已经启动但没有 topic、机器人不动、RViz 报错或路径异常的用户。

排错时先确认你已经在每个终端执行：

```bash
cd ~/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## Gazebo 启动后没有 `/gazebo` 节点

**现象：**

```bash
rosnode list
```

看不到 `/gazebo`，`/clock` 没有数据，`spawn_model` 一直等待 `/gazebo/spawn_urdf_model`。

**检查：**

```bash
pgrep -fa 'gzserver|gazebo|roslaunch'
rostopic echo -n 1 /clock
```

也可以单独验证 world 是否能加载：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
gzserver --verbose src/mr_gazebo/worlds/turtlebot3_world.world
```

**解决：**

- 先按 `Ctrl+C` 结束 launch，再重启一次。
- 确认没有残留 `gzserver`、`rosmaster`。
- 虚拟机中优先使用 `gui:=false headless:=true`。
- 如果一直失败，检查显卡驱动、Gazebo 插件路径和 `LD_LIBRARY_PATH`。

## Gazebo 提示 `/gazebo/set_physics_properties` 未发布

**原因：** Gazebo ROS API 插件还没有初始化完成。

**检查：**

```bash
rosnode list
rostopic list | grep gazebo
```

**解决：**

通常等几秒即可。如果长时间不出现 `/gazebo` 相关 topic，按上一节排查 Gazebo 是否已经退出。

## 没有 `/scan`

**影响：** AMCL、SLAM、costmap 和规划器都会受影响。

**检查：**

```bash
rostopic echo -n 1 /scan
rosnode list
```

**常见原因：**

- 机器人没有成功 spawn；
- Gazebo laser 插件没有加载；
- robot model 的 laser frame 和 costmap 配置不一致；
- Gazebo 没有运行或 `/clock` 没有数据。

**相关文件：**

- `src/mr_description/urdf/project/*.gazebo.xacro`
- `src/mr_description/urdf/wpb_home/simulation/wpb_home_gazebo_plugins.xacro`
- `src/mr_navigation/config/costmap_common_params_*.yaml`

## 没有 `/odom`

**影响：** `odom -> base_footprint` 不存在，move_base 和 SLAM 无法稳定运行。

**检查：**

```bash
rostopic echo -n 1 /odom
rosrun tf tf_echo odom base_footprint
rostopic info /cmd_vel
```

**常见原因：**

- diff drive 插件没有加载；
- `/cmd_vel` topic remap 错误；
- 机器人未生成；
- Gazebo 物理仿真未启动。

## RViz 报 `Fixed Frame [map] does not exist`

**原因：** `/map` 或 `map -> odom` 还没建立。

**检查：**

```bash
rostopic echo -n 1 /map
rosrun tf tf_echo map odom
```

**解决：**

- Navigation 模式确认 `map_server` 和 `amcl` 存在；
- SLAM 模式确认 `gmapping` 或 `hector_mapping` 存在；
- 确认 `/scan` 有数据。

## `Timed out waiting for transform from base_footprint to map`

**原因：** `map -> odom -> base_footprint` 链路不完整。

**检查：**

```bash
rosnode list
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rosrun tf tf_echo map base_footprint
```

**解决：**

- Navigation 模式中先用 `2D Pose Estimate` 给 AMCL 初始位姿；
- 确认 AMCL 正在订阅 `/scan`；
- 确认 robot base frame 是 `base_footprint`。

## 机器人收到 `/cmd_vel` 但不动

**检查：**

```bash
rostopic info /cmd_vel
rostopic echo -n 1 /odom
```

**常见原因：**

- Gazebo diff drive 插件没有订阅 `/cmd_vel`；
- wheel joint 名称不匹配；
- 轮子摩擦或接触参数异常；
- 机器人实际被障碍物卡住。

**解决：**

先用一个很小速度测试：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果 `/odom` 没有变化，优先看 Gazebo 插件和机器人 spawn。

## 看到了 `debug_optimal_path` 但机器人没按这条线走

**原因：** `debug_optimal_path` 是独立算法节点的调试路径，默认不代表 move_base 实际执行路径。

**检查：**

```bash
rostopic echo -n 1 /mr_traditional_planner/executed_global_path
rostopic echo -n 1 /move_base/DWAPlannerROS/global_plan
rostopic echo -n 1 /move_base/DWAPlannerROS/local_plan
rostopic echo -n 5 /cmd_vel
```

**解决：**

确认 `global_planner` 是你想让 move_base 实际使用的算法；`algorithm` 只决定额外启动的调试节点。

## `global_planner:=cubic_spline` 启动失败

**原因：** Cubic Spline 在当前仓库中是路径平滑器或独立调试节点，不是普通全局规划器。

**正确用法：**

```bash
roslaunch mr_navigation navigation_sim.launch global_planner:=theta_star path_smoother:=cubic_spline
```

或者只启动调试节点：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=cubic_spline impl:=cpp
```

## 覆盖规划点击目标后不动

**检查：**

```bash
rostopic echo -n 1 /mr_traditional_planner/coverage_path
rostopic echo -n 1 /move_base/status
rosrun tf tf_echo map base_footprint
```

**说明：** BCD/STC 中 RViz 点击只用于触发覆盖规划，不是覆盖终点。覆盖路径从机器人当前位置附近开始生成，并通过 `/move_base` action 逐点执行。

## 下一步阅读

路径含义看 [planner_framework.md](planner_framework.md)，启动参数看 [launch_reference.md](launch_reference.md)。
