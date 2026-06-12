<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# SLAM 建图

适合读者：想用当前仓库的 Gazebo 场景生成或观察地图的用户。

当前仓库的 SLAM 启动接口位于 `mr_slam`，已接入后端：

- `gmapping`
- `hector`

不要把未接入的 SLAM 算法写入启动命令。

## 1. 启动 SLAM 仿真

推荐入口：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
```

低负载：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping gui:=false use_rviz:=false
```

切换 hector：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=hector
```

也可以从 navigation 包转发：

```bash
roslaunch mr_navigation slam_sim.launch slam_method:=gmapping
```

## 2. 调用关系

```text
mr_slam/slam_sim.launch
  -> mr_gazebo/spawn_robot.launch
  -> mr_slam/slam.launch
       -> gmapping.launch 或 hector.launch
  -> rviz, when use_rviz:=true
```

## 3. 常用参数

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `slam_method` | `gmapping` | `gmapping`, `hector` | SLAM 后端 |
| `stage` | `1` | `1` 到 `4` | 默认 stage world |
| `model` | `burger` | 已支持机器人 key | 导航参数 key |
| `robot_model` | `$(arg model)` | 已支持机器人 key | URDF/Gazebo 模型 |
| `world_name` | `$(find mr_gazebo)/worlds/stage_$(arg stage).world` | world 路径 | Gazebo world |
| `scan_topic` | `/scan` | topic | 激光 topic |
| `odom_topic` | `/odom` | topic | 里程计 topic |
| `base_frame` | `base_footprint` | frame | 底盘 frame |
| `odom_frame` | `odom` | frame | 里程计 frame |
| `map_frame` | `map` | frame | 地图 frame |

## 4. 后端说明

### gmapping

启动文件：

```text
src/mr_slam/launch/gmapping.launch
src/mr_slam/config/gmapping_params.yaml
```

`gmapping` 使用 `/scan` 和 TF/里程计关系建图，发布 `/map`。

### hector

启动文件：

```text
src/mr_slam/launch/hector.launch
src/mr_slam/config/hector.yaml
```

`hector_mapping` 主要依赖激光和 TF。当前仓库只提供启动接入，不额外封装复杂后端参数。

## 5. 建图时移动机器人

可以用键盘控制：

```bash
roslaunch mr_navigation teleop_keyboard.launch
```

这个 launch 使用 `teleop_twist_keyboard.py`，需要在可交互终端中运行。

也可以直接发一个短速度命令：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}"
```

## 6. 保存地图

当前仓库依赖 ROS `map_server` 提供地图保存工具。建图完成后在新终端执行：

```bash
cd ~/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun map_server map_saver -f src/mr_maps/maps/my_map
```

会生成：

```text
src/mr_maps/maps/my_map.yaml
src/mr_maps/maps/my_map.pgm
```

保存后可在 Navigation 中使用：

```bash
roslaunch mr_navigation navigation_sim.launch map_name:=my_map world_name:=<matching_world_path>
```

`map_name` 只写文件名前缀，不写 `.yaml`。

## 7. 成功标准

- `/scan` 有数据；
- `/tf` 有数据；
- `/map` 有 OccupancyGrid；
- RViz 中地图会随着机器人移动逐步更新；
- `map_saver` 能写出 `.yaml` 和 `.pgm`。

## 8. 下一步阅读

已有地图导航看 [navigation.md](navigation.md)，地图/world 配对看 [configuration_reference.md](configuration_reference.md)。

---

<a id="english"></a>

## English

This page describes the current SLAM scope. `mr_slam` provides a unified launch interface for `gmapping` and `hector`.

Typical entry points:

- `roslaunch mr_slam slam.launch slam_method:=gmapping`
- `roslaunch mr_slam slam.launch slam_method:=hector`
- `roslaunch mr_slam slam_sim.launch slam_method:=gmapping`

The current SLAM support is intended for basic mapping, course validation, and topic/TF learning. It is not presented as an advanced multi-sensor SLAM framework.
