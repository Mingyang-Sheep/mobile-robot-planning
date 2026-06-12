<div align="right">

[英文版](README.md)

</div>

# mobile-robot-planning

面向移动机器人仿真、建图、导航、传统路径规划、覆盖路径规划和机器人模型迁移的 ROS Noetic 工作区。

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange)
![Gazebo 11](https://img.shields.io/badge/Gazebo-11-green)
![C++ / Python](https://img.shields.io/badge/C%2B%2B%20%2F%20Python-planning-lightgrey)

本仓库源于 `MEE5115 Autonomous Robotic Systems` 自主机器人系统课程项目。课程基线包含 Gazebo 迷宫环境、ROS 建图、定位和导航；本仓库在此基础上整理 ROS 包，接入多种机器人模型，扩展传统路径规划算法、覆盖路径规划和基础学习演示，目标是形成一个便于复用、学习和继续开发的工作区。

<p align="center">
  <img src="docs/mobile-robot-planning.png" width="100%" alt="mobile-robot-planning system overview">
</p>

系统结构图概括了机器人描述、Gazebo 传感器、建图和定位、全局规划、局部规划、覆盖规划、学习演示以及 RViz/Gazebo 输出。具体启动参数、话题含义和能力边界请阅读专题文档。

<p align="center">
  <video src="docs/mobile_robot_planning_readme_demo.mp4" controls muted playsinline width="85%">
    当前浏览器不支持直接播放嵌入视频。
  </video>
</p>

## 覆盖规划学习札记

下面两篇覆盖路径规划博客由作者本人撰写，适合作为理解 BSA、Spiral-STC 和本仓库覆盖规划模块的前置阅读。当前仓库已经接入的是 BCD 与 STC；博客中的 BSA / Spiral-STC 目前作为学习资料，不是仓库已有启动入口。

| 主题 | 博客 |
|---|---|
| BSA | [【全覆盖路径规划】回溯螺旋算法 Backtracking Spiral Algorithm (BSA)：基于优先级状态机的底层逻辑深入解析](https://blog.csdn.net/weixin_66211313/article/details/159582434) |
| Spiral-STC | [【全覆盖路径规划】螺旋生成树覆盖算法（Spiral-STC）：基于双层栅格与宏观拓扑的在线路径规划解析](https://blog.csdn.net/weixin_66211313/article/details/159733957) |

## 状态图例

| 图标 | 含义 |
|---|---|
| ✅ | 当前仓库已经支持或已经存在 |
| ❌ | 当前仓库没有实现 |
| 🟣 | 部分支持、实验性质或仅模型接入 |

## 当前范围

本仓库重点是移动机器人导航与路径规划实验，不应被理解为完整通用机器人平台。

| 方向 | 状态 | 说明 |
|---|---:|---|
| Gazebo 仿真 | ✅ | 包含世界、机器人生成、差速底盘、激光、里程计、IMU 和相机接口 |
| 建图 | ✅ | 基础接入 `gmapping` 和 `hector`，用于建图教学和课程验证 |
| 导航 | ✅ | 已串联 `map_server`、AMCL、`move_base` 和 DWA |
| 全局路径规划 | ✅ | 支持 `navfn` 以及自定义 A*、Dijkstra、D*、D* Lite、Theta*、RRT* 适配器 |
| 覆盖路径规划 | ✅ | BCD 和 STC 生成覆盖路径，并通过 `/move_base` 执行路径点 |
| 学习模块 | 🟣 | 阶段 1 DQN 实验演示，不是成熟强化学习平台 |
| 视觉感知 | ❌ | 部分模型保留相机话题，但没有完整视觉感知或视觉建图链路 |
| WPB Home Mani 机械臂规划 | ❌ | 可作为移动底盘模型使用，没有实现机械臂规划、MoveIt 或抓取 |

## 软件架构

普通点到点导航链路：

```text
机器人描述
  -> Gazebo 仿真
  -> /scan /odom /tf
  -> 地图服务器或建图
  -> AMCL
  -> 全局规划器
  -> 可选平滑器
  -> DWAPlannerROS
  -> /cmd_vel
```

覆盖任务链路：

```text
覆盖规划器
  -> 覆盖路径
  -> move_base 动作路径点
  -> DWAPlannerROS
  -> /cmd_vel
```

路径话题的含义需要区分：

| 话题 | 参与控制 | 含义 |
|---|---:|---|
| `/mr_traditional_planner/executed_global_path` | ✅ | `GlobalPlannerAdapter` 返回给 `move_base` 的实际全局路径副本 |
| `/mr_traditional_planner/debug_optimal_path` | ❌ | 独立 C++/Python 调试节点输出，用于算法对比和排查 |
| `/move_base/DWAPlannerROS/global_plan` | ✅ | DWA 跟踪的全局参考路径 |
| `/move_base/DWAPlannerROS/local_plan` | ✅ | DWA 当前选择的短时局部轨迹 |
| `/mr_traditional_planner/coverage_path` | 🟣 | 覆盖路径，通过路径点执行间接参与控制 |

## 仓库结构

```text
mobile-robot-planning/
|-- README.md
|-- README_zh.md
|-- docs/
|   |-- index.md
|   |-- zh/
|   |-- mobile-robot-planning.png
|   `-- mobile_robot_planning_readme_demo.mp4
|-- src/
|   |-- mr_description/
|   |-- mr_gazebo/
|   |-- mr_maps/
|   |-- mr_slam/
|   |-- mr_navigation/
|   |-- mr_traditional_planner/
|   |-- mr_learning/
|   `-- mr_msgs/
|-- tools/
`-- refer/
```

## 支持的机器人

| 机器人 key | Gazebo | 建图 | 导航 | 机械臂规划 | 说明 |
|---|---:|---:|---:|---:|---|
| `burger` | ✅ | ✅ | ✅ | ❌ | 默认快速上手模型 |
| `waffle` | ✅ | ✅ | ✅ | ❌ | 较大的 TurtleBot3 风格底盘 |
| `waffle_pi` | ✅ | ✅ | ✅ | ❌ | 保留相机话题，导航仍主要依赖 `/scan` |
| `wpb_home` | ✅ | ✅ | ✅ | ❌ | 官方模型加仿真适配层 |
| `wpb_home_mani` | ✅ | ✅ | ✅ | ❌ | 仅按移动底盘接入，没有机械臂规划或抓取 |

WPB Home 迁移保留官方 URDF 和 mesh，单独增加仿真适配层，并补充 Gazebo 差速底盘、激光、相机/深度、IMU、占地轮廓、代价地图和 DWA 参数。

## 路径规划能力

| 算法 | 角色 | C++ | Python | `move_base` 执行 | 调试显示 | 状态 |
|---|---|---:|---:|---:|---:|---|
| `navfn` | ROS 原生全局规划器 | ✅ | ❌ | ✅ | ✅ | ✅ 基线 |
| A* | 全局规划器 | ✅ | ✅ | ✅ | ✅ | ✅ 可用 |
| Dijkstra | 全局规划器 | ✅ | ✅ | ✅ | ✅ | ✅ 可用 |
| D* | 全局规划器 | ✅ | ✅ | ✅ | ✅ | 🟣 静态代价地图适配 |
| D* Lite | 全局规划器 | ✅ | ✅ | ✅ | ✅ | 🟣 已接入适配器，动态障碍效果仍需评估 |
| Theta* | 全局规划器 | ✅ | ✅ | ✅ | ✅ | ✅ 可用 |
| RRT* | 全局规划器 | ✅ | ✅ | ✅ | ✅ | ✅ 可用 |
| Cubic Spline | 路径平滑器 | ✅ | ✅ | 🟣 | ✅ | 🟣 平滑器，不是独立全局规划器 |
| DWA | 局部规划和调试 | ✅ | ✅ | ✅ | ✅ | 🟣 主链路使用 ROS `DWAPlannerROS` |
| BCD | 覆盖规划器 | ✅ | ✅ | 🟣 | ✅ | ✅ 通过 `/move_base` 执行路径点 |
| STC | 覆盖规划器 | ✅ | ✅ | 🟣 | ✅ | ✅ 通过 `/move_base` 执行路径点 |

C++ 主要负责 pluginlib 接入、`nav_core::BaseGlobalPlanner` 和导航主链路执行。Python 主要用于教学、快速验证、调试可视化和算法对比。

## 快速上手

目标环境：Ubuntu 20.04、ROS Noetic、Gazebo 11。

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

默认演示：

| 项目 | 值 |
|---|---|
| 机器人 | `burger` |
| 世界 | `src/mr_gazebo/worlds/turtlebot3_world.world` |
| 地图 | `src/mr_maps/maps/turtlebot3_world.yaml` |
| 初始位姿 | `x=-2.0`、`y=-0.5`、`yaw=0.0` |
| 启动入口 | `mr_navigation/navigation_sim.launch` |

常见切换：

```bash
roslaunch mr_navigation navigation_sim.launch model:=waffle robot_model:=waffle
roslaunch mr_navigation navigation_sim.launch model:=wpb_home robot_model:=wpb_home
roslaunch mr_navigation navigation_sim.launch global_planner:=theta_star path_smoother:=cubic_spline
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc
```

## 文档

| 分类 | 文档 |
|---|---|
| 入门 | [安装](docs/zh/installation.md)、[快速上手](docs/zh/quick_start.md)、[启动参数参考](docs/zh/launch_reference.md) |
| 机器人与仿真 | [机器人模型](docs/zh/robot_models.md)、[Gazebo 仿真](docs/zh/gazebo_simulation.md)、[URDF 迁移](docs/zh/import_robot_urdf_to_navigation.md)、[URDF 迁移入口](docs/zh/URDF_MIGRATION_GUIDE.md) |
| 建图与导航 | [建图](docs/zh/slam_mapping.md)、[导航](docs/zh/navigation.md)、[话题与 TF](docs/zh/topics_and_tf.md)、[配置参数](docs/zh/configuration_reference.md) |
| 规划 | [规划框架](docs/zh/planner_framework.md)、[普通路径规划](docs/zh/optimal_path_planners.md)、[覆盖路径规划](docs/zh/coverage_path_planning.md)、[学习规划模块](docs/zh/learning_planning.md) |
| 开发与排错 | [仓库架构](docs/zh/repository_architecture.md)、[文档管理规范](docs/zh/documentation_management.md)、[排错](docs/zh/troubleshooting.md)、[参考资料](docs/zh/references.md) |

## 参考资料

本仓库参考 ROS 导航、TurtleBot3/ROBOTIS、WPB Home、WPR Simulation、PythonRobotics 和课程资料。第三方算法来源说明见 [src/mr_traditional_planner/THIRD_PARTY_NOTICES_zh.md](src/mr_traditional_planner/THIRD_PARTY_NOTICES_zh.md)，完整参考资料见 [docs/zh/references.md](docs/zh/references.md)。
