# Navigation 与 Traditional Planner Launch 使用说明

本文档说明当前仓库中导航、仿真、传统规划算法联调相关 launch 的用途、默认值和常用参数。

## 1. 推荐入口

普通导航推荐使用：

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=turtlebot3_world \
  world_name:=$(rospack find mr_gazebo)/worlds/turtlebot3_world.world \
  global_planner:=astar \
  path_smoother:=none \
  local_planner:=dwa
```

传统算法联调和可视化推荐使用：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=turtlebot3_world \
  world_name:=$(rospack find mr_gazebo)/worlds/turtlebot3_world.world \
  algorithm:=astar \
  impl:=cpp
```

`navigation_sim.launch` 只启动 Gazebo、地图、AMCL、move_base 和 RViz。`planner_sim.launch` 在此基础上额外启动传统算法可视化节点，用于对比 `/mr_traditional_planner/debug_optimal_path`、`/mr_traditional_planner/executed_global_path` 或 `/mr_traditional_planner/coverage_path`。路径话题含义见 [planner_path_topics.md](planner_path_topics.md)。

## 2. 默认机器人和地图

`mr_navigation/launch/navigation_sim.launch` 默认：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `model` | `burger` | 底盘参数文件选择 key |
| `robot_model` | `$(arg model)` | Gazebo/URDF 模型选择 |
| `map_name` | `turtlebot3_world` | 加载 `mr_maps/maps/turtlebot3_world.yaml` |
| `world_name` | `$(find mr_gazebo)/worlds/turtlebot3_world.world` | Gazebo world |
| `x` | `-2.0` | 初始 x |
| `y` | `-0.5` | 初始 y |
| `z` | `0.01` | 初始 z |
| `yaw` | `0.0` | 初始 yaw |

`mr_traditional_planner/launch/planner_sim.launch` 默认：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `model` | `burger` | 底盘参数文件选择 key |
| `robot_model` | `$(arg model)` | Gazebo/URDF 模型选择 |
| `map_name` | `turtlebot3_world` | 地图名 |
| `world_name` | `$(find mr_gazebo)/worlds/turtlebot3_world.world` | Gazebo world |
| `algorithm` | `astar` | 额外启动的传统算法可视化节点 |
| `impl` | `cpp` | 可视化节点实现：`cpp` 或 `py` |
| `global_planner` | 随 `algorithm` 自动推导 | 如果 `algorithm` 是全局规划算法，则同步给 move_base |
| `path_smoother` | `none` | 可选路径平滑器 |
| `local_planner` | `dwa` | 局部规划器 |

## 3. 你这条 maze_2 命令还能不能用

可以继续用：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  algorithm:=dijkstra \
  impl:=py
```

现在 `planner_sim.launch` 会做两件事：

1. 启动 `navigation_sim.launch`，也就是 Gazebo、map_server、AMCL、move_base、RViz。
2. 在普通模式下额外启动传统算法节点，`algorithm:=dijkstra impl:=py` 会启动 Python 版 Dijkstra 可视化节点。

由于 `dijkstra` 是合法全局规划算法，`planner_sim.launch` 默认也会把：

```text
global_planner:=dijkstra
```

传给 `move_base`。因此机器人导航实际使用的全局规划器也是 Dijkstra，而不是只在 RViz 中显示 Dijkstra 路径。

如果想写得更明确，也可以这样写：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  algorithm:=dijkstra \
  impl:=py \
  global_planner:=dijkstra \
  local_planner:=dwa
```

## 4. 普通导航参数

普通导航链路：

```text
Global Planner -> Optional Path Smoother -> DWAPlannerROS -> /cmd_vel
```

常用参数：

| 参数 | 可选值 | 说明 |
| --- | --- | --- |
| `planning_mode` | `normal` | 普通点到点导航 |
| `global_planner` | `navfn` | 使用 ROS 原生 `navfn/NavfnROS` |
| `global_planner` | `astar` | 使用 `mr_traditional_planner/GlobalPlannerAdapter` 的 A* |
| `global_planner` | `dijkstra` | 使用 Dijkstra |
| `global_planner` | `dstar` | 使用静态 costmap 搜索适配入口 |
| `global_planner` | `dstar_lite` | 使用静态 costmap 搜索适配入口 |
| `global_planner` | `theta_star` | 使用 Theta* |
| `global_planner` | `rrt_star` | 使用 RRT* |
| `path_smoother` | `none` | 不做平滑 |
| `path_smoother` | `cubic_spline` | 全局路径后处理；碰撞则回退原始路径 |
| `local_planner` | `dwa` | 使用 `dwa_local_planner/DWAPlannerROS` |

示例：

```bash
roslaunch mr_navigation navigation_sim.launch global_planner:=astar local_planner:=dwa
roslaunch mr_navigation navigation_sim.launch global_planner:=dijkstra local_planner:=dwa
roslaunch mr_navigation navigation_sim.launch global_planner:=theta_star path_smoother:=cubic_spline local_planner:=dwa
roslaunch mr_navigation navigation_sim.launch global_planner:=rrt_star local_planner:=dwa
```

## 5. 覆盖导航参数

覆盖导航链路：

```text
Coverage Planner -> /mr_traditional_planner/coverage_path -> move_base action -> DWAPlannerROS -> /cmd_vel
```

常用参数：

| 参数 | 可选值 | 说明 |
| --- | --- | --- |
| `planning_mode` | `coverage` | 覆盖规划模式 |
| `coverage_planner` | `stc` | 启动 STC 覆盖规划器 |
| `coverage_planner` | `bcd` | 启动 BCD 覆盖规划器 |
| `local_planner` | `dwa` | 覆盖路径的每个 waypoint 仍交给 move_base + DWA 执行 |

示例：

```bash
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc local_planner:=dwa
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=bcd local_planner:=dwa
```

## 6. 主要 Launch 文件

### `mr_navigation/launch/navigation_sim.launch`

功能：

```text
Gazebo world
-> robot_description / robot_state_publisher / spawn_model
-> map_server
-> AMCL
-> move_base
-> RViz
```

核心参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `model` | `burger` | 导航参数 key |
| `robot_model` | `$(arg model)` | 机器人模型 |
| `map_name` | `turtlebot3_world` | `mr_maps/maps/<map_name>.yaml` |
| `world_name` | `turtlebot3_world.world` | Gazebo world |
| `x y z yaw` | `-2.0 -0.5 0.01 0.0` | 机器人出生位姿 |
| `wheel_mu` | `1.0` | 轮子摩擦参数 |
| `gui` | `true` | 是否打开 Gazebo GUI |
| `headless` | `false` | Gazebo headless |
| `use_rviz` | `true` | 是否打开 RViz |
| `use_navigation` | `true` | 是否启动 map/AMCL/move_base |
| `global_planner` | `navfn` | 普通导航全局规划器 |
| `path_smoother` | `none` | 路径后处理器 |
| `local_planner` | `dwa` | 局部规划器 |
| `planning_mode` | `normal` | `normal` 或 `coverage` |
| `coverage_planner` | `stc` | 覆盖规划器 |

### `mr_navigation/launch/navigation.launch`

功能：

```text
map_server
-> AMCL
-> planner_compatibility_validator
-> move_base
-> coverage planner, only when planning_mode:=coverage
```

它不启动 Gazebo，不启动 RViz，适合已有仿真或真实机器人底盘时使用。

### `mr_navigation/launch/move_base.launch`

功能：

```text
加载机器人专用 costmap footprint
加载通用 global/local costmap
加载 move_base 参数
加载 DWA 参数
根据 global_planner/local_planner 设置 move_base 插件
```

重要映射：

| 输入参数 | move_base 实际插件 |
| --- | --- |
| `global_planner:=navfn` | `navfn/NavfnROS` |
| `global_planner:=astar/dijkstra/theta_star/rrt_star/dstar/dstar_lite` | `mr_traditional_planner/GlobalPlannerAdapter` |
| `local_planner:=dwa` | `dwa_local_planner/DWAPlannerROS` |

### `mr_traditional_planner/launch/planner_sim.launch`

功能：

```text
navigation_sim.launch
-> 额外启动传统算法可视化节点
```

它适合做算法对比实验。普通导航中，`algorithm` 会启动额外可视化节点；`global_planner` 决定 move_base 实际全局规划器。当前为了兼容旧命令，如果 `algorithm` 是全局规划算法且没有显式覆盖 `global_planner`，`global_planner` 会自动跟随 `algorithm`。

### `mr_traditional_planner/launch/planner.launch`

功能：

```text
algorithm + impl
-> planner_plugin_node, when impl:=cpp
-> python_planner_node.py, when impl:=py
```

它只启动传统算法节点，不启动 Gazebo、map_server、AMCL 或 move_base。

## 7. 兼容性校验

`planner_compatibility_validator.py` 会在 `navigation.launch` 中启动。它会拦截明显错误组合：

| 错误组合 | 结果 |
| --- | --- |
| `global_planner:=cubic_spline` | 退出并报错，Cubic Spline 只能作为 smoother |
| `local_planner:=stc` | 退出并报错，STC 是 coverage planner |
| `local_planner:=bcd` | 退出并报错，BCD 是 coverage planner |
| `local_planner:=astar` | 退出并报错，A* 不输出速度 |
| `planning_mode:=coverage path_smoother:=cubic_spline` | 退出并报错，覆盖模式不使用 smoother |

## 8. 话题关系

普通导航关键话题：

```text
/map
/scan
/odom
/tf
/move_base/GlobalPlannerAdapter/plan, move_base 内部使用
/mr_traditional_planner/executed_global_path, move_base 实际使用的自定义全局路径
/mr_traditional_planner/debug_optimal_path, 独立算法节点调试路径
/move_base/DWAPlannerROS/global_plan, DWA 实际跟踪的全局参考路径
/move_base/DWAPlannerROS/local_plan, DWA 当前选择的局部轨迹
/cmd_vel
```

覆盖导航关键话题：

```text
/map
/scan
/odom
/tf
/mr_traditional_planner/coverage_path
/move_base/goal
/cmd_vel
```

## 9. 常见问题

### 为什么 RViz 里还有 DWAPlannerROS 话题

因为当前真正接入 `nav_core::BaseLocalPlanner` 的局部规划器仍是 ROS 原生 `dwa_local_planner/DWAPlannerROS`。全局路径由 `GlobalPlannerAdapter` 提供，DWA 负责跟踪路径并输出 `/cmd_vel`。

### 为什么 `algorithm` 和 `global_planner` 都存在

`algorithm` 是传统算法可视化节点的参数，主要用于 `planner.launch/planner_sim.launch`。

`global_planner` 是 move_base 实际全局规划器选择参数，主要用于 `navigation.launch/navigation_sim.launch`。

在 `planner_sim.launch` 中，为了兼容旧命令，如果 `algorithm` 是全局规划算法，则 `global_planner` 默认跟随 `algorithm`。

### Cubic Spline 怎么用

不能这样用：

```bash
global_planner:=cubic_spline
```

应该这样用：

```bash
global_planner:=theta_star path_smoother:=cubic_spline
```

### maze_2 怎么跑

确保文件存在：

```text
src/mr_maps/maps/maze_2.yaml
src/mr_maps/maps/maze_2.pgm
src/mr_gazebo/worlds/maze/maze_2.world
```

然后运行：

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  global_planner:=dijkstra \
  local_planner:=dwa
```

需要额外显示 Python Dijkstra 自定义路径时，用：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  algorithm:=dijkstra \
  impl:=py
```
