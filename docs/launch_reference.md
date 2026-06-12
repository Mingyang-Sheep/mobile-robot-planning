# Launch 文件与启动参数参考

适合读者：想复制启动命令、切换机器人/地图/规划算法，或想确认某个 launch 是否适合直接使用的用户。

本文只统计当前项目包 `src/mr_*` 下的 launch，不把 `refer/` 中的外部参考仓库计入当前可维护入口。当前项目共有 40 个 `.launch` 文件。它们已通过静态验证：

- `roslaunch --files`
- `roslaunch --nodes`
- `roslaunch --dump-params`
- 每个展开节点的 `roslaunch --args=<node>`

## 1. 顶层启动入口总表

| Launch | 主要用途 | 默认机器人 | 默认地图/World | Gazebo | RViz | 推荐用户 |
|---|---|---|---|---|---|---|
| `mr_navigation/simulation.launch` | 只启动 Gazebo + 机器人，用于传感器和底盘仿真检查 | `burger` | `empty.world` | 是 | 否 | 初学者、模型调试 |
| `mr_navigation/navigation_sim.launch` | Gazebo + map_server + AMCL + move_base + RViz | `burger` | `turtlebot3_world` | 是 | 是 | 新手首选 |
| `mr_navigation/slam_sim.launch` | 从 navigation 包转发到 SLAM 仿真入口 | `burger` | `stage_1.world` | 是 | 是 | 需要从 navigation 包统一启动 SLAM 的用户 |
| `mr_slam/slam_sim.launch` | Gazebo + SLAM + RViz | `burger` | `stage_1.world` | 是 | 是 | 建图实验 |
| `mr_traditional_planner/planner_sim.launch` | navigation_sim + 独立传统算法调试节点 | `burger` | `turtlebot3_world` | 是 | 是 | 算法对比实验 |
| `mr_learning/dqn_train.launch` | Gazebo stage 1 + DQN 训练脚本 | `burger` | `stage_1.world` | 是 | 否 | Learning 实验开发者 |
| `mr_description/wpb_home_description.launch` | 只显示 WPB Home / WPB Home Mani 描述模型 | `wpb_home` | 无 | 否 | 是 | URDF 检查 |

新手建议先用：

```bash
roslaunch mr_navigation navigation_sim.launch
```

虚拟机或远程环境建议：

```bash
roslaunch mr_navigation navigation_sim.launch gui:=false headless:=true use_rviz:=false
```

## 2. 顶层 Launch 参数

### `mr_navigation/navigation_sim.launch`

用途：完整 Navigation 仿真入口。

调用关系：

```text
navigation_sim.launch
  -> mr_gazebo/spawn_navigation_world.launch
       -> mr_description/load_robot_description.launch
       -> gazebo_ros/gzserver
       -> robot_state_publisher
       -> gazebo_ros/spawn_model
  -> mr_navigation/navigation.launch
       -> mr_maps/map_loader.launch
       -> mr_navigation/amcl.launch
       -> mr_navigation/move_base.launch
  -> rviz, when use_rviz:=true
```

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `model` | `burger` | `burger`, `waffle`, `waffle_pi`, `wpb_home`, `wpb_home_mani` | 导航参数 key |
| `robot_model` | `$(arg model)` | 同上 | URDF/Gazebo 模型 key |
| `map_name` | `turtlebot3_world` | `turtlebot3_world`, `maze_2` | `mr_maps/maps/<map_name>.yaml` |
| `world_name` | `$(find mr_gazebo)/worlds/turtlebot3_world.world` | world 文件路径 | Gazebo world |
| `x` | `-2.0` | 数值 | 初始 x |
| `y` | `-0.5` | 数值 | 初始 y |
| `z` | `0.01` | 数值 | 初始 z |
| `yaw` | `0.0` | 数值 | 初始 yaw |
| `gui` | `true` | `true`, `false` | 是否打开 Gazebo GUI |
| `headless` | `false` | `true`, `false` | Gazebo 是否无头运行 |
| `paused` | `false` | `true`, `false` | Gazebo 是否暂停启动 |
| `use_rviz` | `true` | `true`, `false` | 是否启动 RViz |
| `use_navigation` | `true` | `true`, `false` | 是否启动 map_server、AMCL、move_base |
| `scan_topic` | `/scan` | topic | 激光 topic |
| `odom_topic` | `/odom` | topic | 里程计 topic |
| `cmd_vel_topic` | `/cmd_vel` | topic | 速度命令 topic |
| `planning_mode` | `normal` | `normal`, `coverage` | 普通导航或覆盖模式 |
| `global_planner` | `navfn` | `navfn`, `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, `rrt_star` | move_base 实际全局规划器 |
| `path_smoother` | `none` | `none`, `cubic_spline` | 普通全局路径后处理 |
| `local_planner` | `dwa` | `dwa` | move_base 局部规划器 |
| `coverage_planner` | `stc` | `stc`, `bcd` | 覆盖模式算法 |
| `costmap_cost_weight` | `2.0` | 数值 | 自定义全局规划 adapter 的 costmap 软代价权重 |

最小启动：

```bash
roslaunch mr_navigation navigation_sim.launch
```

maze_2 示例：

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

成功标准：`/gazebo`、`/map_server`、`/amcl`、`/move_base` 存在，`/scan`、`/odom`、`/tf`、`/map` 有数据。

### `mr_navigation/simulation.launch`

用途：只启动仿真 world 和机器人，不启动 map_server、AMCL、move_base、RViz。

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `model` | `burger` | 已支持机器人 key | 导航/底盘参数 key |
| `robot_model` | `$(arg model)` | 已支持机器人 key | URDF/Gazebo 模型 |
| `world_name` | `$(find mr_gazebo)/worlds/empty.world` | world 路径 | Gazebo world |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.01`, `0` | 数值 | 初始位姿 |
| `gui` | `true` | `true`, `false` | Gazebo GUI |
| `headless` | `false` | `true`, `false` | 无头模式 |
| `paused` | `false` | `true`, `false` | 暂停启动 |
| `use_sim_time` | `true` | `true`, `false` | 使用仿真时钟 |
| `enable_laser` | `true` | `true`, `false` | WPB Home 仿真激光开关 |
| `enable_camera` | `true` | `true`, `false` | WPB Home 仿真相机开关 |
| `enable_imu` | `true` | `true`, `false` | WPB Home 仿真 IMU 开关 |

最小启动：

```bash
roslaunch mr_navigation simulation.launch
```

成功标准：`/gazebo` 存在，机器人生成，`/scan`、`/odom`、`/tf` 有数据。

### `mr_slam/slam_sim.launch`

用途：启动 Gazebo + SLAM 后端 + RViz。

调用关系：

```text
mr_slam/slam_sim.launch
  -> mr_gazebo/spawn_robot.launch
  -> mr_slam/slam.launch
       -> gmapping.launch 或 hector.launch
  -> rviz, when use_rviz:=true
```

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `slam_method` | `gmapping` | `gmapping`, `hector` | SLAM 后端 |
| `model` | `burger` | 已支持机器人 key | 导航参数 key |
| `robot_model` | `$(arg model)` | 已支持机器人 key | URDF/Gazebo 模型 |
| `stage` | `1` | `1` 到 `4` | 默认 stage world 编号 |
| `world_name` | `$(find mr_gazebo)/worlds/stage_$(arg stage).world` | world 路径 | Gazebo world |
| `x`, `y`, `z`, `yaw` | `0`, `0`, `0.01`, `0` | 数值 | 初始位姿 |
| `gui` | `true` | `true`, `false` | Gazebo GUI |
| `headless` | `false` | `true`, `false` | 无头模式 |
| `use_rviz` | `true` | `true`, `false` | 是否启动 RViz |
| `scan_topic` | `/scan` | topic | 激光 topic |
| `odom_topic` | `/odom` | topic | 里程计 topic |

示例：

```bash
roslaunch mr_slam slam_sim.launch slam_method:=gmapping
roslaunch mr_slam slam_sim.launch slam_method:=hector gui:=false use_rviz:=false
```

### `mr_navigation/slam_sim.launch`

用途：从 `mr_navigation` 包提供的 SLAM 仿真转发入口。实际 include `mr_slam/slam_sim.launch`。

示例：

```bash
roslaunch mr_navigation slam_sim.launch slam_method:=gmapping
```

### `mr_traditional_planner/planner_sim.launch`

用途：启动完整 Navigation 仿真，并额外启动一个独立 C++/Python 传统算法调试节点。

调用关系：

```text
planner_sim.launch
  -> mr_navigation/navigation_sim.launch
       -> Gazebo
       -> map_server
       -> AMCL
       -> move_base
       -> RViz
  -> mr_traditional_planner/planner.launch, when run_debug_planner:=true
       -> planner_plugin_node, when impl:=cpp
       -> python_planner_node.py, when impl:=py
```

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `algorithm` | `astar` | `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, `rrt_star`, `dwa`, `cubic_spline`, `bcd`, `stc` | 额外调试节点算法 |
| `impl` | `cpp` | `cpp`, `py` | 调试节点实现语言 |
| `global_planner` | 若 algorithm 是普通全局规划则跟随 algorithm，否则 `astar` | 普通全局规划器可选值 | move_base 实际全局规划器 |
| `local_planner` | `dwa` | `dwa` | move_base 局部规划器 |
| `path_smoother` | `none` | `none`, `cubic_spline` | move_base 全局路径平滑器 |
| `debug_path_topic` | 根据 algorithm 推导 | topic | 调试节点输出路径 |
| `run_debug_planner` | `algorithm != dwa` | `true`, `false` | 是否启动额外调试节点 |
| `model`, `robot_model`, `map_name`, `world_name`, `x`, `y`, `yaw` | 同 `navigation_sim.launch` | 见上文 | 仿真和导航配置 |

普通全局规划示例：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  algorithm:=dstar_lite \
  impl:=cpp \
  global_planner:=dstar_lite \
  local_planner:=dwa
```

maze_2 示例：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  algorithm:=dstar_lite \
  impl:=cpp \
  global_planner:=dstar_lite \
  local_planner:=dwa
```

这条命令中，`global_planner:=dstar_lite` 决定 move_base 实际使用 D* Lite adapter；`algorithm:=dstar_lite impl:=cpp` 决定额外 C++ 调试节点发布 `/mr_traditional_planner/debug_optimal_path`。

### `mr_traditional_planner/planner.launch`

用途：只启动独立传统规划调试节点，不启动 Gazebo、map_server、AMCL 或 move_base。

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `algorithm` | `astar` | 所有传统算法 key | 算法选择 |
| `impl` | `cpp` | `cpp`, `py` | 实现语言 |
| `planner_plugin` | 空 | pluginlib class | C++ 插件覆盖 |
| `path_topic` | `/mr_traditional_planner/debug_optimal_path` | topic | 输出路径 topic |

示例：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=theta_star impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=dijkstra impl:=py
```

### `mr_learning/dqn_train.launch`

用途：启动 Gazebo stage 环境并运行 DQN 训练脚本。当前是实验性 Demo，不是成熟强化学习平台。

| 参数 | 默认值 | 可选值 | 含义 |
|---|---|---|---|
| `stage` | `1` | 当前代码只映射 `1` | 训练环境编号 |
| `model` | `burger` | 已支持机器人 key | 仿真机器人 |
| `episodes` | `3000` | 正整数 | 训练 episode 数 |
| `max_steps` | `6000` | 正整数 | 每个 episode 最大步数 |
| `load_model` | `false` | `true`, `false` | 是否加载已有模型 |
| `gui` | `true` | `true`, `false` | Gazebo GUI |

示例：

```bash
roslaunch mr_learning dqn_train.launch stage:=1 gui:=false
```

## 3. 内部 Launch 参考

| Launch | 所属 package | 用途 | 新手直接使用 |
|---|---|---|---|
| `mr_description/load_robot_description.launch` | `mr_description` | 按 `robot_model` 加载 `robot_description` | 否，通常被 include |
| `mr_description/wpb_home_description.launch` | `mr_description` | WPB Home / WPB Home Mani 描述模型检查 | 是，URDF 调试时可用 |
| `mr_gazebo/spawn_navigation_world.launch` | `mr_gazebo` | Navigation 场景 spawn world + robot | 否，被 `navigation_sim` include |
| `mr_gazebo/spawn_robot.launch` | `mr_gazebo` | 通用 Gazebo world + robot spawn | 否，被 simulation/SLAM/Learning include |
| `mr_maps/map_loader.launch` | `mr_maps` | 启动 `map_server` | 否，已有总入口 |
| `mr_navigation/navigation.launch` | `mr_navigation` | 无 Gazebo 的地图、AMCL、move_base 链路 | 有真实底盘或已启动仿真时可用 |
| `mr_navigation/amcl.launch` | `mr_navigation` | 启动 AMCL | 否，通常被 include |
| `mr_navigation/move_base.launch` | `mr_navigation` | 加载 costmap、planner、DWA 并启动 move_base | 否，通常被 include |
| `mr_navigation/teleop_keyboard.launch` | `mr_navigation` | 键盘控制 `/cmd_vel` | 可用，但需要终端交互 |
| `mr_slam/slam.launch` | `mr_slam` | 按 `slam_method` include SLAM 后端 | 已有机器人和传感器时可用 |
| `mr_slam/gmapping.launch` | `mr_slam` | 启动 `slam_gmapping` | 否，通常经 `slam.launch` |
| `mr_slam/hector.launch` | `mr_slam` | 启动 `hector_mapping` | 否，通常经 `slam.launch` |
| `mr_learning/learning_env.launch` | `mr_learning` | 只启动 stage 学习环境 | 开发者可用 |

## 4. 单算法调试 Launch

`mr_traditional_planner/launch/*_cpp.launch` 和 `*_py.launch` 是直接启动单算法调试节点的包装。它们不启动 Gazebo、地图、AMCL 或 move_base，通常需要先启动 `navigation_sim.launch`。

| 算法 | C++ launch | Python launch | 默认输出 |
|---|---|---|---|
| A* | `astar_cpp.launch` | `astar_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| Dijkstra | `dijkstra_cpp.launch` | `dijkstra_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| D* | `dstar_cpp.launch` | `dstar_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| D* Lite | `dstar_lite_cpp.launch` | `dstar_lite_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| Theta* | `theta_star_cpp.launch` | `theta_star_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| RRT* | `rrt_star_cpp.launch` | `rrt_star_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| Cubic Spline | `cubic_spline_cpp.launch` | `cubic_spline_py.launch` | `/mr_traditional_planner/debug_optimal_path` |
| DWA 调试 | `dwa_cpp.launch` | `dwa_py.launch` | `/mr_traditional_planner/debug_optimal_path` 和 `/cmd_vel` |
| BCD | `bcd_cpp.launch` | `bcd_py.launch` | `/mr_traditional_planner/coverage_path` |
| STC | `stc_cpp.launch` | `stc_py.launch` | `/mr_traditional_planner/coverage_path` |

## 5. 全部当前 Launch 分类

| 类别 | 数量 | 文件 |
|---|---:|---|
| 机器人描述 | 2 | `load_robot_description.launch`, `wpb_home_description.launch` |
| Gazebo spawn | 2 | `spawn_navigation_world.launch`, `spawn_robot.launch` |
| 地图 | 1 | `map_loader.launch` |
| Navigation | 7 | `simulation.launch`, `navigation_sim.launch`, `navigation.launch`, `move_base.launch`, `amcl.launch`, `slam_sim.launch`, `teleop_keyboard.launch` |
| SLAM | 4 | `slam.launch`, `slam_sim.launch`, `gmapping.launch`, `hector.launch` |
| Traditional planner | 22 | `planner.launch`, `planner_sim.launch` 和 20 个单算法 C++/Python launch |
| Learning | 2 | `learning_env.launch`, `dqn_train.launch` |
| 合计 | 40 | 当前项目 `src/mr_*` 下所有 `.launch` |

## 6. 常见错误组合

| 错误组合 | 原因 | 正确写法 |
|---|---|---|
| `global_planner:=cubic_spline` | Cubic Spline 是 smoother 或调试节点，不是全局规划器 | `global_planner:=theta_star path_smoother:=cubic_spline` |
| `local_planner:=astar` | A* 输出路径，不输出速度 | `global_planner:=astar local_planner:=dwa` |
| `local_planner:=bcd` | BCD 是覆盖规划器 | `planning_mode:=coverage coverage_planner:=bcd` |
| `planning_mode:=coverage path_smoother:=cubic_spline` | 覆盖模式当前不使用 smoother | 去掉 `path_smoother` |

## 7. 下一步阅读

普通导航看 [navigation.md](navigation.md)，传统规划看 [planner_framework.md](planner_framework.md)，覆盖规划看 [coverage_path_planning.md](coverage_path_planning.md)。
