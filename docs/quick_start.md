<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# 快速上手

适合读者：完全不熟悉 ROS，但想先确认仓库是否能在本机跑起来的用户。

本 Quick Start 选择仓库默认的 `burger + turtlebot3_world + navigation_sim.launch`。原因是：默认文件已经配套，命令最短，Gazebo、地图、AMCL、move_base、DWA 和 RViz 都能从一个入口启动，适合几分钟内确认环境。

## 1. 你需要打开终端

终端就是输入命令的窗口。Ubuntu 里可以用 `Ctrl+Alt+T` 打开。

ROS 程序通常会在终端里持续运行。看到大量日志是正常的；要结束当前 launch，回到启动它的终端按 `Ctrl+C`。

## 2. 编译和加载环境

打开第一个终端：

```bash
cd ~/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

`source /opt/ros/noetic/setup.bash` 是加载系统 ROS。

`source devel/setup.bash` 是加载当前仓库编译出来的 ROS package。

## 3. 启动 Quick Start Demo

推荐第一次直接启动带 RViz 和 Gazebo GUI 的完整演示：

```bash
roslaunch mr_navigation navigation_sim.launch
```

如果虚拟机性能较弱，可以先关闭 Gazebo GUI 和 RViz 做后台验证：

```bash
roslaunch mr_navigation navigation_sim.launch gui:=false headless:=true use_rviz:=false
```

## 4. Quick Start 默认配置

| 项目 | 当前默认值 |
|---|---|
| 机器人 | `burger` |
| 机器人尺寸与传感器 | TurtleBot3 Burger 风格差速底盘，`base_footprint` 底盘 frame，`base_scan` 2D LaserScan，IMU 插件 |
| world | `src/mr_gazebo/worlds/turtlebot3_world.world` |
| map | `src/mr_maps/maps/turtlebot3_world.yaml` |
| 初始位姿 | `x=-2.0`、`y=-0.5`、`z=0.01`、`yaw=0.0` |
| 启动功能 | Gazebo、robot_state_publisher、map_server、AMCL、move_base、DWA、RViz |
| 预期 topic | `/scan`、`/odom`、`/tf`、`/tf_static`、`/map`、`/cmd_vel` |
| 预期 TF | `map -> odom -> base_footprint -> base_link -> base_scan` |

## 5. Gazebo 和 RViz 分别看什么

Gazebo 是物理仿真器。你主要看：

- 机器人是否出现在世界里；
- 机器人是否会响应 `/cmd_vel`；
- 激光、轮子、底盘插件是否正常工作。

RViz 是 ROS 数据可视化工具。你主要看：

- 地图 `/map`；
- 机器人模型 `RobotModel`；
- 激光 `/scan`；
- TF；
- 全局路径和局部路径；
- `2D Pose Estimate` 与 `2D Nav Goal` 工具。

## 6. 在 RViz 中设置初始位姿

如果机器人在 RViz 里和地图对不齐：

1. 点击工具栏 `2D Pose Estimate`。
2. 在地图上按住鼠标左键，拖出机器人朝向。
3. 松开鼠标后，AMCL 会收到 `/initialpose`。

默认启动命令已经把 AMCL 初始位姿设为 `x=-2.0, y=-0.5, yaw=0.0`，通常第一次可以不手动设置。

## 7. 在 RViz 中发送导航目标

1. 点击工具栏 `2D Nav Goal`。
2. 在地图可通行区域按住鼠标左键。
3. 拖出目标朝向后松开。

如果导航链路正常，你应该能看到：

- `/move_base/NavfnROS/plan` 或自定义规划器路径；
- `/move_base/DWAPlannerROS/global_plan`；
- `/move_base/DWAPlannerROS/local_plan`；
- `/cmd_vel` 开始输出速度；
- Gazebo 中机器人开始移动。

## 8. 检查关键 topic

另开一个终端，先加载环境：

```bash
cd ~/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

查看 topic 列表：

```bash
rostopic list
```

查看一条激光数据：

```bash
rostopic echo -n 1 /scan
```

查看一条里程计：

```bash
rostopic echo -n 1 /odom
```

查看地图：

```bash
rostopic echo -n 1 /map
```

查看 TF：

```bash
rostopic echo -n 1 /tf
rosrun tf tf_echo odom base_footprint
```

测试 `/cmd_vel` 是否能发布：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 9. 成功标准

Quick Start 可认为成功，当你确认：

- Gazebo 启动并且 `/gazebo` 节点存在；
- 机器人已经生成；
- `/scan` 有 LaserScan 数据；
- `/odom` 有 Odometry 数据；
- `/tf` 或 `tf_echo odom base_footprint` 有变换；
- `/map` 有 OccupancyGrid 数据；
- `/cmd_vel` 有订阅者，发布速度后机器人或 odom 有响应；
- RViz 配置 `src/mr_navigation/rviz/navigation.rviz` 可加载。

本仓库验证时，headless 命令第二次短时运行通过了上述关键 topic 检查。第一次运行曾出现 Gazebo 早退，重跑后正常；如果你也遇到类似情况，请看 [troubleshooting.md](troubleshooting.md) 中的 Gazebo 排错项。

## 10. 结束进程

回到运行 `roslaunch` 的终端，按：

```text
Ctrl+C
```

等待终端输出 `done` 后再关闭窗口。

如果怀疑有残留进程，可以检查：

```bash
pgrep -fa 'roslaunch|rosmaster|roscore|gzserver|gzclient'
```

## 11. 下一步阅读

完成 Quick Start 后，继续阅读 [launch_reference.md](launch_reference.md) 和 [navigation.md](navigation.md)。

---

<a id="english"></a>

## English

This page is the shortest validated way to start the repository. The default demo uses `burger`, `turtlebot3_world`, `map_server`, AMCL, `move_base`, DWA, Gazebo, and RViz.

Basic run sequence:

- Build the workspace with `catkin_make`.
- Source both `/opt/ros/noetic/setup.bash` and `devel/setup.bash`.
- Launch `roslaunch mr_navigation navigation_sim.launch`.
- In RViz, use `2D Pose Estimate` if localization is misaligned and `2D Nav Goal` to send a target.
- Check `/scan`, `/odom`, `/tf`, `/map`, and `/cmd_vel` if the robot does not move.

The default pose is `x=-2.0`, `y=-0.5`, `yaw=0.0`. The Chinese section above includes the full topic checklist and success criteria.
