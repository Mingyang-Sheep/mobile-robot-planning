<div align="right">

[英文版](../gazebo_simulation.md)

</div>

# Gazebo 仿真

适合读者：想检查 世界文件、机器人 生成、底盘插件和传感器 话题 的用户。

## 1. World 文件

当前仓库 世界文件 文件位于：

```text
src/mr_gazebo/worlds/
```

已存在：

| World | 用途 |
|---|---|
| `empty.world` | 最小空世界，适合模型和插件检查 |
| `turtlebot3_world.world` | 快速上手和默认导航配套 世界文件 |
| `stage_1.world` 到 `stage_4.world` | 学习 / SLAM 阶段场景 |
| `maze/maze_1.world` 到 `maze/maze_6.world` | 迷宫场景 |

导航 需要静态地图配套。当前已有静态地图的组合是：

| map_name | 世界文件 |
|---|---|
| `turtlebot3_world` | `src/mr_gazebo/worlds/turtlebot3_world.world` |
| `maze_2` | `src/mr_gazebo/worlds/maze/maze_2.world` |

## 2. 机器人 生成

主要入口：

```bash
roslaunch mr_navigation simulation.launch
```

它会 包含：

```text
mr_gazebo/spawn_robot.launch
  -> gazebo_ros/empty_world.launch
  -> mr_description/load_robot_description.launch
  -> robot_state_publisher
  -> gazebo_ros/spawn_model
```

导航 仿真使用：

```text
mr_gazebo/spawn_navigation_world.launch
```

两者都会通过 `robot_description` 参数把 URDF 交给 Gazebo。

## 3. 主要 Gazebo 插件

| 功能 | 插件 | 输出/输入 |
|---|---|---|
| 差速底盘 | `libgazebo_ros_diff_drive.so` | 订阅 `/cmd_vel`，发布 `/odom`、`/joint_states` |
| 2D 激光 | `libgazebo_ros_laser.so` | 发布 `/scan` |
| IMU | `libgazebo_ros_imu.so` 或 `libgazebo_ros_imu_sensor.so` | 发布 `/imu` |
| RGB-D 相机 | `libgazebo_ros_depth_camera.so` | 发布相机和深度图像 话题 |
| 机械臂静态关节状态 | `libgazebo_ros_joint_state_publisher.so` | 发布 `/joint_states` |

TurtleBot3 风格模型的 Gazebo 插件在：

```text
src/mr_description/urdf/project/turtlebot3_burger.gazebo.xacro
src/mr_description/urdf/project/turtlebot3_waffle.gazebo.xacro
src/mr_description/urdf/project/turtlebot3_waffle_pi.gazebo.xacro
```

WPB Home 风格模型的插件在：

```text
src/mr_description/urdf/wpb_home/simulation/wpb_home_gazebo_plugins.xacro
```

## 4. 仿真验证命令

启动空世界 + 默认 burger：

```bash
roslaunch mr_navigation simulation.launch
```

低负载启动：

```bash
roslaunch mr_navigation simulation.launch gui:=false headless:=true
```

切换到 WPB Home：

```bash
roslaunch mr_navigation simulation.launch model:=wpb_home robot_model:=wpb_home
```

检查 话题：

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rostopic info /cmd_vel
rosrun tf tf_echo odom base_footprint
```

手动发速度：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 5. 常见状态判断

| 现象 | 优先检查 |
|---|---|
| 没有 `/gazebo` | Gazebo 是否退出、世界文件 是否能单独加载 |
| `spawn_model` 一直等待 | `/gazebo/spawn_urdf_model` 服务是否存在 |
| 没有 `/scan` | laser 插件、坐标系、机器人是否 生成 |
| 没有 `/odom` | diff drive 插件、wheel joint、Gazebo 是否运行 |
| `/cmd_vel` 无订阅者 | 底盘插件未加载或 话题 remap 错误 |

## 6. 下一步阅读

导航 启动看 [navigation.md](navigation.md)，模型适配看 [robot_models.md](robot_models.md)。
