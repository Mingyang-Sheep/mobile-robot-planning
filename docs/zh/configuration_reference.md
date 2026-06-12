<div align="right">

[英文版](../configuration_reference.md)

</div>

# 配置参数参考

适合读者：想调整机器人尺寸、代价地图、DWA、AMCL、SLAM 或 planner 参数的用户。

本文不复制所有 YAML 原文，只整理当前仓库中最常用、最容易影响运行结果的参数。

## 1. 配置目录

| 目录 | 内容 |
|---|---|
| `src/mr_navigation/config/` | 机器人模型注册、代价地图、DWA、AMCL、move_base、planner 参数 |
| `src/mr_slam/config/` | gmapping / hector 参数 |
| `src/mr_maps/maps/` | 静态地图 `.yaml/.pgm` |
| `src/mr_description/urdf/` | URDF/Xacro 和 Gazebo 插件 |

## 2. `robot_models.yaml`

文件：

```text
src/mr_navigation/config/robot_models.yaml
```

主要字段：

| 字段 | 含义 |
|---|---|
| `xacro` | 模型对应的仿真 Xacro |
| `base_frame` | 底盘 坐标系，当前通常是 `base_footprint` |
| `laser_frame` | 激光 坐标系，TurtleBot3 为 `base_scan`，WPB Home 为 `laser` |
| `odom_frame` | 里程计 坐标系，当前为 `odom` |
| `scan_topic` | 激光 话题，当前统一为 `/scan` |
| `cmd_vel_topic` | 速度命令 话题，当前统一为 `/cmd_vel` |
| `odom_topic` | 里程计 话题，当前统一为 `/odom` |
| `nav_config_key` | 导航参数文件 key |

新增机器人时，先把模型登记到这里，再补对应导航参数文件。

## 3. Costmap 参数

常见文件：

```text
costmap_common_params_burger.yaml
costmap_common_params_waffle.yaml
costmap_common_params_waffle_pi.yaml
costmap_common_params_wpb_home.yaml
costmap_common_params_wpb_home_mani.yaml
global_costmap_params.yaml
local_costmap_params.yaml
```

关键参数：

| 参数 | 作用 |
|---|---|
| `footprint` | 机器人占地多边形，直接影响能否通过窄门和贴墙程度 |
| `observation_sources` | 传感器源，当前主要是 `scan` |
| `sensor_frame` | 必须匹配 `/scan.header.frame_id` |
| `topic` | 当前统一为 `/scan` |
| `obstacle_range` | 标记障碍的最大距离 |
| `raytrace_range` | 清除障碍的最大距离 |
| `inflation_radius` | 障碍膨胀半径 |
| `cost_scaling_factor` | 膨胀代价下降速度 |
| `robot_base_frame` | 通常为 `base_footprint` |
| `global_frame` | 全局代价地图 为 `map`，局部代价地图 为 `odom` |

如果路径贴墙，优先看 `footprint`、`inflation_radius` 和 `costmap_cost_weight`。

## 4. DWA 参数

文件示例：

```text
dwa_local_planner_params_burger.yaml
dwa_local_planner_params_wpb_home.yaml
```

关键参数：

| 参数 | 作用 |
|---|---|
| `max_vel_x` | 最大前进速度 |
| `min_vel_x` | 最小 x 速度 |
| `max_vel_theta` | 最大角速度 |
| `min_vel_theta` | 最小角速度采样 |
| `acc_lim_x` | x 加速度限制 |
| `acc_lim_theta` | 角加速度限制 |
| `sim_time` | DWA 前向模拟时间 |
| `vx_samples` | 线速度采样数 |
| `vth_samples` | 角速度采样数 |
| `path_distance_bias` | 贴近全局路径的权重 |
| `goal_distance_bias` | 接近目标的权重 |
| `occdist_scale` | 避障代价权重 |
| `xy_goal_tolerance` | 位置目标容差 |
| `yaw_goal_tolerance` | 朝向目标容差 |

机器人在墙角抖动时，通常看 `sim_time`、速度采样、代价地图占地轮廓和膨胀参数。

## 5. AMCL 参数

通用参数主要在 启动文件 中设置，WPB Home 还有专用覆盖文件：

```text
amcl_params_wpb_home.yaml
amcl_params_wpb_home_mani.yaml
```

关键参数：

| 参数 | 作用 |
|---|---|
| `base_frame_id` | 当前默认 `base_footprint` |
| `odom_frame_id` | 当前默认 `odom` |
| `global_frame_id` | 通常为 `map` |
| `initial_pose_x/y/a` | 初始位姿 |
| `laser_max_range` | 激光最大有效距离 |
| `laser_max_beams` | AMCL 使用的激光束数量 |
| `laser_model_type` | 激光模型 |
| `odom_model_type` | 里程计模型 |

如果 RViz 中机器人和地图对不齐，先用 `2D Pose Estimate`，再检查 AMCL 初始位姿和 TF。

## 6. SLAM 参数

文件：

```text
src/mr_slam/config/gmapping_params.yaml
src/mr_slam/config/hector.yaml
```

当前 启动文件 支持：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
roslaunch mr_slam slam_sim.launch slam_method:=hector
```

SLAM 依赖 `/scan` 和 TF。gmapping 还强依赖可用的里程计关系。

## 7. Planner 参数

导航 中常用：

| 参数 | 作用 |
|---|---|
| `planning_mode` | `normal` 或 `coverage` |
| `global_planner` | `navfn`, `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, `rrt_star` |
| `path_smoother` | `none`, `cubic_spline` |
| `local_planner` | 当前为 `dwa` |
| `coverage_planner` | `stc`, `bcd` |
| `costmap_cost_weight` | 自定义全局规划适配器中 代价地图软代价权重 |

独立调试节点常用：

| 参数 | 作用 |
|---|---|
| `algorithm` | 选择调试算法 |
| `impl` | `cpp` 或 `py` |
| `planner_plugin` | C++ pluginlib class 覆盖 |
| `path_topic` | 调试路径输出 话题 |

## 8. 世界文件/map 配对

当前已有静态地图配对：

| `map_name` | map 文件 | 推荐 世界文件 |
|---|---|---|
| `turtlebot3_world` | `src/mr_maps/maps/turtlebot3_world.yaml` | `src/mr_gazebo/worlds/turtlebot3_world.world` |
| `maze_2` | `src/mr_maps/maps/maze_2.yaml` | `src/mr_gazebo/worlds/maze/maze_2.world` |

启动示例：

```bash
roslaunch mr_navigation navigation_sim.launch \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0
```

## 9. 下一步阅读

启动参数看 [launch_reference.md](launch_reference.md)，话题 和 TF 看 [topics_and_tf.md](topics_and_tf.md)。
