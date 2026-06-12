<div align="right">

[英文版](../repository_architecture.md)

</div>

# 仓库架构

适合读者：想理解本仓库各 ROS 包 职责、数据流和扩展边界的用户。

## 1. 包职责

| 包 | 职责 |
|---|---|
| `mr_description` | 保存机器人 URDF/Xacro、mesh、RViz 模型显示配置，提供 `robot_description` 加载入口 |
| `mr_gazebo` | 保存 Gazebo 世界文件/model，提供仿真 世界文件 和机器人 生成 入口 |
| `mr_maps` | 保存静态地图 `.yaml/.pgm`，提供 `map_server` 加载入口 |
| `mr_slam` | 统一 SLAM 启动接口，当前接入 `gmapping` 和 `hector` |
| `mr_navigation` | 统一仿真、导航、AMCL、move_base、RViz、teleop 的 启动文件 与参数 |
| `mr_traditional_planner` | 传统规划算法、move_base 全局规划适配器、调试路径节点和覆盖规划节点 |
| `mr_learning` | 阶段 1 DQN 实验性训练入口和 Gazebo 环境封装 |
| `mr_msgs` | benchmark 级消息接口包，当前包含 `PlannerResult.msg` |

## 2. 主数据流

导航链路可以按下面理解：

```text
URDF/Xacro
  -> robot_description
  -> robot_state_publisher
  -> TF

Gazebo 世界文件 + 机器人生成
  -> /scan
  -> /odom
  -> /imu
  -> /cmd_vel 订阅者

map_server 或建图
  -> /map

AMCL
  -> map -> odom

move_base
  -> 全局代价地图
  -> 全局规划器
  -> 局部代价地图
  -> 局部规划器
  -> /cmd_vel
```

典型 TF：

```text
map
  -> odom
    -> base_footprint
      -> base_link
        -> base_scan 或 laser
        -> camera / imu
```

## 3. 普通点到点导航

当前普通导航主链路：

```text
map_server
  -> AMCL
  -> move_base
     -> navfn 或 GlobalPlannerAdapter
     -> 可选 Cubic Spline 平滑器
     -> DWAPlannerROS
     -> /cmd_vel
```

如果 `global_planner:=navfn`，`move_base` 使用 ROS 原生 `navfn/NavfnROS`。

如果 `global_planner:=astar|dijkstra|dstar|dstar_lite|theta_star|rrt_star`，`move_base` 使用 `mr_traditional_planner/GlobalPlannerAdapter`，适配器内部再按参数选择算法。

## 4. 覆盖任务

覆盖规划不是普通点到点目标的替代插件，而是单独的覆盖路径生成和路径点执行链路：

```text
覆盖规划器
  -> /mr_traditional_planner/coverage_path
  -> /move_base 动作目标
  -> DWAPlannerROS
  -> /cmd_vel
```

当前覆盖算法为 BCD 和 STC。RViz 的 `2D Nav Goal` 在覆盖模式中主要作为触发信号，不表示覆盖终点。

## 5. Debug 路径和实际执行路径

本仓库把算法调试显示和机器人实际执行路径分开：

| 路径 | 含义 |
|---|---|
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 算法节点输出的调试路径，默认不直接控制机器人 |
| `/mr_traditional_planner/executed_global_path` | `GlobalPlannerAdapter` 真正返回给 `move_base` 的路径副本 |
| `/move_base/DWAPlannerROS/global_plan` | DWA 内部正在跟踪的全局参考路径 |
| `/move_base/DWAPlannerROS/local_plan` | DWA 当前选择的短时局部轨迹 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 生成的覆盖路径 |

调试时不要只看 `debug_optimal_path`。确认机器人实际运动时，应同时看 `executed_global_path`、DWA 全局/局部规划 和 `/cmd_vel`。

## 6. 当前地图与 世界文件

当前仓库真实存在的静态地图：

- `src/mr_maps/maps/turtlebot3_world.yaml`
- `src/mr_maps/maps/maze_2.yaml`

当前仓库真实存在的 世界文件：

- `src/mr_gazebo/worlds/empty.world`
- `src/mr_gazebo/worlds/turtlebot3_world.world`
- `src/mr_gazebo/worlds/stage_1.world` 到 `stage_4.world`
- `src/mr_gazebo/worlds/maze/maze_1.world` 到 `maze_6.world`

只有 `turtlebot3_world` 和 `maze_2` 在 `mr_maps/maps/` 中有静态地图文件，做 导航 时优先使用这两组配对。

## 7. 下一步阅读

启动入口看 [launch_reference.md](launch_reference.md)，机器人模型看 [robot_models.md](robot_models.md)，路径规划看 [planner_framework.md](planner_framework.md)。
