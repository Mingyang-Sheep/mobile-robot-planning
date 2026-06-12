<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# mobile-robot-planning

面向自主机器人课程项目和 ROS 初学者的移动机器人仿真、建图、导航与路径规划学习框架。

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange)
![Gazebo 11](https://img.shields.io/badge/Gazebo-11-green)
![C++ / Python](https://img.shields.io/badge/C%2B%2B%20%2F%20Python-planning-lightgrey)

本仓库源于 `MEE5115 Autonomous Robotic Systems` 自主机器人系统课程项目。课程 baseline 包含 Gazebo 迷宫环境、ROS SLAM 建图、定位与 Navigation 导航；本仓库在此基础上做了 package 标准化、多机器人模型接入、传统路径规划算法扩展、覆盖路径规划和基础 learning demo，目标是形成一个便于复用、学习和继续开发的 ROS Noetic 工作区。

<p align="center">
  <img src="docs/mobile-robot-planning.png" width="100%" alt="mobile-robot-planning system overview">
</p>

系统从机器人模型、Gazebo 传感器、SLAM/Localization、全局规划、局部规划、覆盖规划、Learning demo 一直串到 RViz/Gazebo 输出。图片只概括主链路，具体启动参数和限制请看后面的专题文档。

<p align="center">
  <a href="docs/mobile_robot_planning_readme_demo.mp4">
    <img src="docs/mobile-robot-planning.png" width="85%" alt="Watch project demo">
  </a>
</p>

<p align="center">
  <a href="docs/mobile_robot_planning_readme_demo.mp4">Watch the project demo video</a>
</p>

## 覆盖规划学习札记

下面两篇覆盖路径规划博客由作者本人撰写，适合作为理解 BSA、Spiral-STC 和本仓库覆盖规划模块的前置阅读。当前仓库已经接入的是 BCD 与 STC；博客中的 BSA / Spiral-STC 目前作为学习资料，不写成仓库已有 launch 入口。

| Topic | Blog |
|---|---|
| BSA | [【全覆盖路径规划】回溯螺旋算法 Backtracking Spiral Algorithm (BSA)：基于优先级状态机的底层逻辑深入解析](https://blog.csdn.net/weixin_66211313/article/details/159582434) |
| Spiral-STC | [【全覆盖路径规划】螺旋生成树覆盖算法（Spiral-STC）：基于双层栅格与宏观拓扑的在线路径规划解析](https://blog.csdn.net/weixin_66211313/article/details/159733957) |

## 当前范围

本仓库重点是移动机器人导航与路径规划实验，不把自己包装成完整通用机器人平台。

| Area | 当前状态 |
|---|---|
| Gazebo 仿真 | 已有 world、机器人 spawn、差速底盘、laser、odom、IMU/相机接口 |
| SLAM | 基础接入 `gmapping` 与 `hector`，主要用于建图教学和验证 |
| Navigation | `map_server`、AMCL、`move_base`、DWA 主链路可用 |
| 全局路径规划 | `navfn` 与自定义 A*、Dijkstra、D*、D* Lite、Theta*、RRT* adapter |
| 覆盖路径规划 | BCD、STC 生成覆盖路径，并通过 `/move_base` action 逐 waypoint 执行 |
| Learning | stage 1 DQN 实验性 demo，不是成熟强化学习规划平台 |
| 视觉 | 部分模型保留 camera/depth camera topic，当前没有完整视觉感知或视觉 SLAM 链路 |
| WPB Home Mani | 可作为移动底盘模型接入仿真、SLAM、Navigation；没有完整机械臂规划、MoveIt 或抓取任务 |

## 软件架构

普通点到点导航链路：

```text
Robot Description
  -> Gazebo Simulation
  -> /scan /odom /tf
  -> Map Server or SLAM
  -> AMCL
  -> Global Planner
  -> Optional Smoother
  -> DWAPlannerROS
  -> /cmd_vel
```

覆盖任务链路：

```text
Coverage Planner
  -> Coverage Path
  -> move_base action waypoints
  -> DWAPlannerROS
  -> /cmd_vel
```

路径话题的含义需要区分：

| Topic | 含义 | 是否直接参与控制 |
|---|---|---|
| `/mr_traditional_planner/executed_global_path` | `GlobalPlannerAdapter` 真正返回给 `move_base` 的全局路径副本 | 是 |
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 调试节点输出的算法路径 | 默认否 |
| `/move_base/DWAPlannerROS/global_plan` | DWA 内部跟踪的全局参考 | 是 |
| `/move_base/DWAPlannerROS/local_plan` | DWA 当前生成的短时局部轨迹 | 是 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 生成的覆盖路径 | 间接参与 waypoint 执行 |

## 仓库结构

```text
mobile-robot-planning/
|-- docs/                         # 仓库级教程、参考和媒体文件
|-- src/
|   |-- mr_description/           # URDF/Xacro、mesh、robot_description 入口
|   |-- mr_gazebo/                # Gazebo world/model 与 spawn launch
|   |-- mr_maps/                  # 静态地图与 map_server 入口
|   |-- mr_slam/                  # gmapping / hector 统一 SLAM 入口
|   |-- mr_navigation/            # AMCL、move_base、DWA、RViz 和统一仿真入口
|   |-- mr_traditional_planner/   # 全局规划、覆盖规划、DWA/样条调试与 pluginlib adapter
|   |-- mr_learning/              # stage 1 DQN 实验性训练入口
|   `-- mr_msgs/                  # benchmark 级 PlannerResult.msg 消息包
|-- tools/                        # 环境检查和依赖安装脚本
`-- refer/                        # 参考材料或上游片段，非主工作区核心包
```

## 支持的机器人

| Robot key | Description | Gazebo | SLAM | Navigation | Notes |
|---|---|---|---|---|---|
| `burger` | TurtleBot3 Burger 风格差速底盘 | 是 | 是 | 是 | Quick Start 默认模型，虚拟机上最稳 |
| `waffle` | TurtleBot3 Waffle 风格底盘 | 是 | 是 | 是 | 有独立 footprint 与 DWA 参数 |
| `waffle_pi` | TurtleBot3 Waffle Pi 风格模型 | 是 | 是 | 是 | 保留相机 topic，导航仍主要依赖 `/scan` |
| `wpb_home` | WPB Home 官方模型 + simulation 适配层 | 是 | 是 | 是 | 保留官方 URDF/mesh，补充 diff-drive、laser、IMU/camera 插件 |
| `wpb_home_mani` | WPB Home Mani 官方模型 + simulation 适配层 | 是 | 是 | 是 | 当前按移动底盘处理，不包含机械臂规划与抓取 |

WPB Home 的迁移原则是：保留官方 URDF/mesh；单独建立 simulation-only 适配层；补充标准 Gazebo diff-drive、laser、camera/IMU 等接口；为 footprint、costmap 和 DWA 使用独立参数。

## 路径规划能力

| Algorithm | Role | C++ | Python | move_base execution | Debug visualization | Purpose | Status |
|---|---|---|---|---|---|---|---|
| `navfn` | ROS 原生全局规划器 | ROS | 否 | 是 | RViz 中由 move_base 显示 | baseline navigation | 可用 |
| A* | 普通全局规划 | 是 | 是 | 是 | 是 | 栅格搜索与教学对比 | 可用 |
| Dijkstra | 普通全局规划 | 是 | 是 | 是 | 是 | 保守栅格搜索 | 可用 |
| D* | 普通全局规划 | 是 | 是 | 是 | 是 | D* 风格搜索实验 | 静态 costmap 适配，不等同完整动态 D* |
| D* Lite | 普通全局规划 | 是 | 是 | 是 | 是 | 状态化增量搜索实验 | 已接入，动态障碍效果仍需实验数据 |
| Theta* | 普通全局规划 | 是 | 是 | 是 | 是 | 任意角路径 | 可用，依赖视线检测与地图分辨率 |
| RRT* | 普通全局规划 | 是 | 是 | 是 | 是 | 采样式路径搜索 | 可用，受采样参数影响 |
| Cubic Spline | 路径平滑器 | 是 | 是 | 作为 smoother | 是 | 后处理和路径平滑 | 不是独立全局规划器 |
| DWA | 局部规划 / 调试 | 是 | 是 | 主链路使用 ROS `DWAPlannerROS` | 是 | 局部轨迹采样与控制 | 独立 debug DWA 不等同 move_base 内部 DWA |
| BCD | 覆盖路径规划 | 是 | 是 | 通过 waypoints 间接执行 | 是 | 自由空间覆盖 | 可用 |
| STC | 覆盖路径规划 | 是 | 是 | 通过 waypoints 间接执行 | 是 | 树/螺旋式覆盖 | 可用 |

C++ 主要负责 `pluginlib`、`nav_core::BaseGlobalPlanner` adapter、实时导航主链路和可执行节点。Python 主要用于算法教学、快速验证、`debug_optimal_path` 对比和结果排查。`impl:=py` 启动的是 Python 调试节点，不代表 Python 节点天然就是 `move_base` 的全局规划插件。

判断机器人实际执行的是哪个算法，请优先看 `global_planner` 参数、`base_global_planner`、`/mr_traditional_planner/executed_global_path` 和 DWA 的 global/local plan，而不是只看 debug path。

## Quick Start

环境目标：Ubuntu 20.04 + ROS Noetic + Gazebo 11。

```bash
git clone https://github.com/Mingyang-Sheep/mobile-robot-planning.git
cd mobile-robot-planning
source /opt/ros/noetic/setup.bash
bash tools/check_environment.sh
sudo bash tools/install_dependencies.sh
catkin_make
source devel/setup.bash
roslaunch mr_navigation navigation_sim.launch
```

默认演示配置：

| Item | Value |
|---|---|
| Robot | `burger` |
| World | `src/mr_gazebo/worlds/turtlebot3_world.world` |
| Map | `src/mr_maps/maps/turtlebot3_world.yaml` |
| Initial pose | `x=-2.0`, `y=-0.5`, `yaw=0.0` |
| Launch entry | `mr_navigation/navigation_sim.launch` |

RViz 中常用两个工具：

1. `2D Pose Estimate`：必要时给 AMCL 重新设置初始位姿。
2. `2D Nav Goal`：发送普通导航目标；覆盖模式下作为触发覆盖任务的信号。

常见切换：

```bash
roslaunch mr_navigation navigation_sim.launch model:=waffle robot_model:=waffle
roslaunch mr_navigation navigation_sim.launch model:=wpb_home robot_model:=wpb_home
roslaunch mr_navigation navigation_sim.launch global_planner:=theta_star path_smoother:=cubic_spline
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc
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

## 扩展新算法

普通全局规划算法的 C++ 主入口位于 `src/mr_traditional_planner/`：

1. 在合适目录实现 `mr_traditional_planner::PlannerPlugin`。
2. 在 `planner_plugins.xml` 注册 pluginlib class。
3. 按需要更新 `CMakeLists.txt` 和 `package.xml`。
4. 让 `GlobalPlannerAdapter` 或调试 launch 能映射到新的算法 key。
5. 增加 RViz path topic 和失败日志，避免失败时复用旧路径冒充成功。
6. 更新算法能力矩阵和对应专题文档。

Python 调试算法经由 `scripts/python_planner_node.py` 分发，具体脚本放在 `src/mr_traditional_planner/scripts/`。它适合教学与结果对比，默认输出 debug path。

## 移植新机器人

推荐迁移链路：

```text
Official URDF / Xacro / mesh
  -> mr_description
  -> simulation adaptation layer
  -> Gazebo plugins
  -> topic and TF check
  -> costmap / footprint / DWA params
  -> SLAM
  -> Navigation
```

详细过程请看 [docs/import_robot_urdf_to_navigation.md](docs/import_robot_urdf_to_navigation.md) 和 [docs/URDF_MIGRATION_GUIDE.md](docs/URDF_MIGRATION_GUIDE.md)。

## Documentation

| Category | Documents |
|---|---|
| Getting Started | [Installation](docs/installation.md), [Quick Start](docs/quick_start.md), [Launch Reference](docs/launch_reference.md) |
| Robots and Simulation | [Robot Models](docs/robot_models.md), [Gazebo Simulation](docs/gazebo_simulation.md), [URDF Migration](docs/import_robot_urdf_to_navigation.md), [URDF Migration Guide](docs/URDF_MIGRATION_GUIDE.md) |
| Mapping and Navigation | [SLAM Mapping](docs/slam_mapping.md), [Navigation](docs/navigation.md), [Navigation Launch Reference](docs/navigation_launch_reference.md), [Topics and TF](docs/topics_and_tf.md), [Configuration Reference](docs/configuration_reference.md) |
| Planning | [Planner Framework](docs/planner_framework.md), [Optimal Path Planners](docs/optimal_path_planners.md), [Coverage Path Planning](docs/coverage_path_planning.md), [Planner Path Topics](docs/planner_path_topics.md), [Learning Planning](docs/learning_planning.md) |
| Development and Troubleshooting | [Repository Architecture](docs/repository_architecture.md), [Documentation Management](docs/documentation_management.md), [Troubleshooting](docs/troubleshooting.md), [References](docs/references.md) |

## References

本仓库依赖和参考了 ROS Navigation、TurtleBot3/ROBOTIS、WPB Home、WPR Simulation、PythonRobotics 以及课程相关资料。第三方算法来源说明见 [src/mr_traditional_planner/THIRD_PARTY_NOTICES.md](src/mr_traditional_planner/THIRD_PARTY_NOTICES.md)，完整参考资料见 [docs/references.md](docs/references.md)。

---

<a id="english"></a>

# mobile-robot-planning

A ROS Noetic workspace for mobile robot simulation, SLAM, navigation, path planning, coverage planning, and robot model migration.

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange)
![Gazebo 11](https://img.shields.io/badge/Gazebo-11-green)
![C++ / Python](https://img.shields.io/badge/C%2B%2B%20%2F%20Python-planning-lightgrey)

This repository grew out of the `MEE5115 Autonomous Robotic Systems` course project. The course baseline covers Gazebo maze simulation, ROS SLAM mapping, localization, and Navigation. This repository standardizes that baseline into reusable ROS packages and extends it with multiple robot models, traditional planners, coverage planners, and a basic learning demo.

<p align="center">
  <img src="docs/mobile-robot-planning.png" width="100%" alt="mobile-robot-planning system overview">
</p>

<p align="center">
  <a href="docs/mobile_robot_planning_readme_demo.mp4">Watch the project demo video</a>
</p>

## Coverage Planning Notes

The following coverage-planning blog posts were written by the repository author. They are useful companion notes for understanding BSA, Spiral-STC, and the coverage-planning ideas behind this workspace. The currently integrated coverage planners are BCD and STC; BSA and Spiral-STC are referenced as learning material, not as existing launch entries.

| Topic | Blog |
|---|---|
| BSA | [Backtracking Spiral Algorithm (BSA): priority-state-machine logic](https://blog.csdn.net/weixin_66211313/article/details/159582434) |
| Spiral-STC | [Spiral-STC: online coverage planning with two-level grids and macro topology](https://blog.csdn.net/weixin_66211313/article/details/159733957) |

## Current Scope

This is a learning and development workspace for mobile robot navigation and path-planning experiments. It should not be read as a complete general-purpose robotics platform.

| Area | Status |
|---|---|
| Gazebo simulation | Worlds, robot spawning, differential drive, laser, odom, IMU/camera interfaces |
| SLAM | Basic `gmapping` and `hector` integration for mapping and course validation |
| Navigation | `map_server`, AMCL, `move_base`, and DWA are wired together |
| Global planning | `navfn` plus custom A*, Dijkstra, D*, D* Lite, Theta*, and RRT* adapters |
| Coverage planning | BCD and STC generate coverage paths and execute waypoints through `/move_base` |
| Learning | Stage 1 DQN experimental demo, not a mature RL planning framework |
| Vision | Some robot models expose camera topics; no complete perception or visual SLAM pipeline is implemented |
| WPB Home Mani | Usable as a mobile-base model for simulation/navigation; arm planning and grasping are not implemented |

## Architecture

Point-to-point navigation:

```text
Robot Description
  -> Gazebo Simulation
  -> /scan /odom /tf
  -> Map Server or SLAM
  -> AMCL
  -> Global Planner
  -> Optional Smoother
  -> DWAPlannerROS
  -> /cmd_vel
```

Coverage tasks:

```text
Coverage Planner
  -> Coverage Path
  -> move_base action waypoints
  -> DWAPlannerROS
  -> /cmd_vel
```

The debug path is not necessarily the executed path. To inspect actual motion, check `global_planner`, `/mr_traditional_planner/executed_global_path`, DWA global/local plans, and `/cmd_vel`.

## Quick Start

Target environment: Ubuntu 20.04, ROS Noetic, and Gazebo 11.

```bash
git clone https://github.com/Mingyang-Sheep/mobile-robot-planning.git
cd mobile-robot-planning
source /opt/ros/noetic/setup.bash
bash tools/check_environment.sh
sudo bash tools/install_dependencies.sh
catkin_make
source devel/setup.bash
roslaunch mr_navigation navigation_sim.launch
```

Default demo:

| Item | Value |
|---|---|
| Robot | `burger` |
| World | `src/mr_gazebo/worlds/turtlebot3_world.world` |
| Map | `src/mr_maps/maps/turtlebot3_world.yaml` |
| Initial pose | `x=-2.0`, `y=-0.5`, `yaw=0.0` |
| Launch entry | `mr_navigation/navigation_sim.launch` |

## Planner Support

| Algorithm | Role | C++ | Python | move_base execution | Notes |
|---|---|---|---|---|---|
| `navfn` | ROS global planner | ROS | No | Yes | Baseline navigation planner |
| A*, Dijkstra, Theta*, RRT* | Global planners | Yes | Yes | Yes | Used through `GlobalPlannerAdapter` or debug nodes |
| D*, D* Lite | Global planners | Yes | Yes | Yes | D* is static-style here; D* Lite still needs more dynamic-obstacle evaluation |
| Cubic Spline | Smoother | Yes | Yes | As smoother | Not a standalone global planner |
| DWA | Local planning/debug | Yes | Yes | ROS `DWAPlannerROS` in the main chain | Independent debug DWA is not the same as move_base's local planner |
| BCD, STC | Coverage planners | Yes | Yes | Through waypoint execution | Publish coverage paths and call `/move_base` |

## Documentation

| Category | Documents |
|---|---|
| Getting Started | [Installation](docs/installation.md), [Quick Start](docs/quick_start.md), [Launch Reference](docs/launch_reference.md) |
| Robots and Simulation | [Robot Models](docs/robot_models.md), [Gazebo Simulation](docs/gazebo_simulation.md), [URDF Migration](docs/import_robot_urdf_to_navigation.md), [URDF Migration Guide](docs/URDF_MIGRATION_GUIDE.md) |
| Mapping and Navigation | [SLAM Mapping](docs/slam_mapping.md), [Navigation](docs/navigation.md), [Navigation Launch Reference](docs/navigation_launch_reference.md), [Topics and TF](docs/topics_and_tf.md), [Configuration Reference](docs/configuration_reference.md) |
| Planning | [Planner Framework](docs/planner_framework.md), [Optimal Path Planners](docs/optimal_path_planners.md), [Coverage Path Planning](docs/coverage_path_planning.md), [Planner Path Topics](docs/planner_path_topics.md), [Learning Planning](docs/learning_planning.md) |
| Development and Troubleshooting | [Repository Architecture](docs/repository_architecture.md), [Documentation Management](docs/documentation_management.md), [Troubleshooting](docs/troubleshooting.md), [References](docs/references.md) |

## References

This workspace references ROS Navigation, TurtleBot3/ROBOTIS, WPB Home, WPR Simulation, PythonRobotics, and course material. Third-party notices are kept in [src/mr_traditional_planner/THIRD_PARTY_NOTICES.md](src/mr_traditional_planner/THIRD_PARTY_NOTICES.md), and the full reference list is in [docs/references.md](docs/references.md).
