<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# Navigation 导航

适合读者：想理解并使用 AMCL + move_base 点到点导航链路的用户。

当前仓库的默认导航入口是：

```bash
roslaunch mr_navigation navigation_sim.launch
```

## 1. Navigation 链路

```text
Gazebo
  -> /scan
  -> /odom
  -> /tf

map_server
  -> /map

AMCL
  -> map -> odom

move_base
  -> global costmap
  -> global planner
  -> local costmap
  -> DWAPlannerROS
  -> /cmd_vel
```

## 2. 主要节点

| 节点 | 来源 launch | 作用 |
|---|---|---|
| `/map_server` | `mr_maps/map_loader.launch` | 加载静态地图 |
| `/amcl` | `mr_navigation/amcl.launch` | 发布 `map -> odom` 定位关系 |
| `/move_base` | `mr_navigation/move_base.launch` | 路径规划、局部控制、recovery |
| `/robot_state_publisher` | Gazebo spawn launch | 发布机器人内部 TF |
| `/gazebo` | Gazebo launch | 物理仿真、传感器和底盘插件 |

## 3. 全局规划器

| 参数 | move_base 插件 | 说明 |
|---|---|---|
| `global_planner:=navfn` | `navfn/NavfnROS` | ROS 原生全局规划器 |
| `global_planner:=astar` | `mr_traditional_planner/GlobalPlannerAdapter` | 自定义 A* adapter |
| `global_planner:=dijkstra` | `mr_traditional_planner/GlobalPlannerAdapter` | 自定义 Dijkstra adapter |
| `global_planner:=dstar` | `mr_traditional_planner/GlobalPlannerAdapter` | 静态 costmap 上的 D* 风格实现 |
| `global_planner:=dstar_lite` | `mr_traditional_planner/GlobalPlannerAdapter` | D* Lite adapter |
| `global_planner:=theta_star` | `mr_traditional_planner/GlobalPlannerAdapter` | Theta* adapter |
| `global_planner:=rrt_star` | `mr_traditional_planner/GlobalPlannerAdapter` | RRT* adapter |

示例：

```bash
roslaunch mr_navigation navigation_sim.launch global_planner:=astar local_planner:=dwa
roslaunch mr_navigation navigation_sim.launch global_planner:=theta_star path_smoother:=cubic_spline
```

## 4. 路径平滑器

当前普通导航支持：

| 参数 | 含义 |
|---|---|
| `path_smoother:=none` | 不做后处理 |
| `path_smoother:=cubic_spline` | 对全局路径做 Cubic Spline 平滑；如果碰撞检测失败，adapter 会回退原始路径 |

不要把 `cubic_spline` 写成 `global_planner`。

## 5. 局部规划器

当前导航主链路中的局部规划器是：

```text
dwa_local_planner/DWAPlannerROS
```

由参数控制：

```bash
local_planner:=dwa
```

DWA 相关路径：

- `/move_base/DWAPlannerROS/global_plan`
- `/move_base/DWAPlannerROS/local_plan`
- `/move_base/DWAPlannerROS/trajectory_cloud`
- `/move_base/DWAPlannerROS/cost_cloud`

## 6. Costmap

配置文件位于：

```text
src/mr_navigation/config/
```

常见文件：

- `costmap_common_params_<model>.yaml`
- `global_costmap_params.yaml`
- `local_costmap_params.yaml`
- `dwa_local_planner_params_<model>.yaml`
- `move_base_params.yaml`
- WPB Home 专用覆盖文件

关键参数：

| 参数 | 作用 |
|---|---|
| `footprint` | 机器人占地多边形 |
| `sensor_frame` | 激光 frame，必须匹配 `/scan.header.frame_id` |
| `obstacle_range` | 障碍标记距离 |
| `raytrace_range` | 清除障碍距离 |
| `inflation_radius` | 障碍膨胀半径 |
| `robot_base_frame` | 通常为 `base_footprint` |
| `global_frame` | global costmap 通常为 `map`，local costmap 通常为 `odom` |

## 7. RViz 操作

设置初始位姿：

1. 点击 `2D Pose Estimate`。
2. 在地图上拖出机器人位置和朝向。

发送目标：

1. 点击 `2D Nav Goal`。
2. 在自由空间拖出目标位置和朝向。

观察：

- 地图 `/map`；
- 激光 `/scan`；
- 全局 costmap；
- 局部 costmap；
- DWA global/local plan；
- `/cmd_vel` 是否输出。

## 8. maze_2 导航示例

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  global_planner:=dstar_lite \
  local_planner:=dwa
```

## 9. 下一步阅读

路径话题看 [topics_and_tf.md](topics_and_tf.md)，规划算法看 [planner_framework.md](planner_framework.md) 和 [optimal_path_planners.md](optimal_path_planners.md)。

---

<a id="english"></a>

## English

This page explains the AMCL + `move_base` navigation chain.

Main flow:

- Gazebo publishes `/scan`, `/odom`, and TF.
- `map_server` publishes `/map`.
- AMCL publishes the `map -> odom` localization transform.
- `move_base` runs global and local costmaps, a global planner, `DWAPlannerROS`, and outputs `/cmd_vel`.

Use `global_planner:=navfn` for the ROS baseline, or `global_planner:=astar|dijkstra|dstar|dstar_lite|theta_star|rrt_star` for the custom adapter. `cubic_spline` is a smoother, not a global planner.
