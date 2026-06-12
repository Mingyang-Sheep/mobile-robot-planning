<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# 文档导航

适合读者：第一次接触本仓库的同学、课程项目使用者，以及准备移植新机器人或新规划算法的开发者。

主目录 `README.md` 负责 GitHub 首页总览，`docs/` 负责专题说明。所有页面只描述当前仓库中已经存在、静态展开通过，或在源码中可以明确确认的内容。

## 按使用目标阅读

| 你想做什么 | 建议阅读 |
|---|---|
| 第一次安装和编译 | [installation.md](installation.md) |
| 快速确认环境能跑 | [quick_start.md](quick_start.md) |
| 理解仓库分包职责 | [repository_architecture.md](repository_architecture.md) |
| 找启动命令和参数 | [launch_reference.md](launch_reference.md) |
| 切换机器人模型 | [robot_models.md](robot_models.md) |
| 只看 Gazebo 仿真 | [gazebo_simulation.md](gazebo_simulation.md) |
| 做 SLAM 建图 | [slam_mapping.md](slam_mapping.md) |
| 做 AMCL + move_base 导航 | [navigation.md](navigation.md) |
| 切换传统规划算法 | [planner_framework.md](planner_framework.md), [optimal_path_planners.md](optimal_path_planners.md) |
| 使用覆盖路径规划 | [coverage_path_planning.md](coverage_path_planning.md) |
| 了解 Learning 模块 | [learning_planning.md](learning_planning.md) |
| 移植新 URDF 到导航链路 | [import_robot_urdf_to_navigation.md](import_robot_urdf_to_navigation.md) |
| 查参数、topic、TF | [configuration_reference.md](configuration_reference.md), [topics_and_tf.md](topics_and_tf.md) |
| 启动失败或路径异常 | [troubleshooting.md](troubleshooting.md) |
| 查看来源和外部参考 | [references.md](references.md) |
| 维护 README / docs / package 文档 | [documentation_management.md](documentation_management.md) |

## 推荐学习路径

### ROS 初学者

1. [installation.md](installation.md)
2. [quick_start.md](quick_start.md)
3. [topics_and_tf.md](topics_and_tf.md)
4. [troubleshooting.md](troubleshooting.md)

### 移动机器人导航初学者

1. [repository_architecture.md](repository_architecture.md)
2. [gazebo_simulation.md](gazebo_simulation.md)
3. [slam_mapping.md](slam_mapping.md)
4. [navigation.md](navigation.md)
5. [planner_framework.md](planner_framework.md)

### MEE5115 课程项目使用者

1. [quick_start.md](quick_start.md)
2. [launch_reference.md](launch_reference.md)
3. [optimal_path_planners.md](optimal_path_planners.md)
4. [coverage_path_planning.md](coverage_path_planning.md)
5. [configuration_reference.md](configuration_reference.md)

### 准备扩展仓库的开发者

1. [repository_architecture.md](repository_architecture.md)
2. [robot_models.md](robot_models.md)
3. [import_robot_urdf_to_navigation.md](import_robot_urdf_to_navigation.md)
4. [planner_framework.md](planner_framework.md)
5. [learning_planning.md](learning_planning.md)
6. [documentation_management.md](documentation_management.md)

## 当前已确认的主要能力

- Gazebo 11 仿真入口：`mr_navigation/simulation.launch`、`mr_navigation/navigation_sim.launch`、`mr_slam/slam_sim.launch`、`mr_traditional_planner/planner_sim.launch`。
- 机器人模型选择：`burger`、`waffle`、`waffle_pi`、`wpb_home`、`wpb_home_mani`。
- 地图文件：`turtlebot3_world` 和 `maze_2` 两组静态地图。
- SLAM 后端：`gmapping` 和 `hector`。
- Navigation：`map_server`、`AMCL`、`move_base`、`navfn` 或自定义全局规划 adapter、`DWAPlannerROS`。
- 普通全局规划：A*、Dijkstra、D*、D* Lite、Theta*、RRT*。
- 路径后处理调试：Cubic Spline。
- 局部控制调试：DWA 独立调试节点；导航主链路仍使用 ROS `dwa_local_planner/DWAPlannerROS`。
- 覆盖规划：BCD 和 STC。
- Learning：stage 1 DQN 训练实验入口，当前应视为实验性 Demo。

## 下一步阅读

第一次使用请继续阅读 [installation.md](installation.md) 和 [quick_start.md](quick_start.md)。

---

<a id="english"></a>

## English

This page is the documentation hub for the workspace. Start here if you are new to the repository, need a reading path, or want to find the right topic page quickly.

Key entry points:

- Installation and first run: `installation.md`, `quick_start.md`, and `launch_reference.md`.
- Robot models and simulation: `robot_models.md` and `gazebo_simulation.md`.
- SLAM and navigation: `slam_mapping.md`, `navigation.md`, `topics_and_tf.md`, and `configuration_reference.md`.
- Planning: `planner_framework.md`, `optimal_path_planners.md`, `coverage_path_planning.md`, and `learning_planning.md`.
- Development and troubleshooting: `import_robot_urdf_to_navigation.md`, `documentation_management.md`, `troubleshooting.md`, and `references.md`.

The Chinese section above remains the most detailed table of contents. File paths, commands, topics, and parameter names are shared by both languages.
