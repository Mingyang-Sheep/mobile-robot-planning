<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# mr_slam

`mr_slam` 是本工作区统一的 SLAM 启动与配置包。当前它把 `gmapping` 和 `hector` 作为普通 ROS 依赖接入，主要服务于基础建图、课程验证和 topic/TF 学习。

完整专题文档见 [../../docs/slam_mapping.md](../../docs/slam_mapping.md)。

## 当前后端

| 参数值 | 预期节点 | 说明 |
|---|---|---|
| `gmapping` | `/slam_gmapping` | 依赖 2D 激光与里程计，适合默认仿真建图 |
| `hector` | `/hector_mapping` | 可用于对比不同 SLAM 后端的 topic/TF 行为 |

## 最小命令

```bash
roslaunch mr_slam slam.launch slam_method:=gmapping
roslaunch mr_slam slam.launch slam_method:=hector
```

带 Gazebo 的一体化入口：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
roslaunch mr_slam slam_sim.launch slam_method:=hector
```

低负载运行时可按 launch 参数关闭 GUI 或 RViz：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping gui:=false
roslaunch mr_slam slam_sim.launch slam_method:=gmapping use_rviz:=false
```

## 常用参数

| 参数 | 默认值 |
|---|---|
| `scan_topic` | `/scan` |
| `odom_topic` | `/odom` |
| `base_frame` | `base_footprint` |
| `odom_frame` | `odom` |
| `map_frame` | `map` |

## 快速检查

编译并 source 工作区后：

```bash
rospack find mr_slam
roslaunch --nodes mr_slam slam.launch slam_method:=gmapping
roslaunch --nodes mr_slam slam.launch slam_method:=hector
```

运行时重点检查：

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /tf
rostopic echo -n 1 /map
rosrun tf tf_echo map base_footprint
```

本包不应描述为高级多传感器 SLAM 框架。当前范围是基础 2D SLAM 接入和教学验证。

---

<a id="english"></a>

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
