<div align="right">

[英文](README.md)

</div>

# mobile-robot-planning：go2w-navigation 课程 Bonus 分支

![ROS Noetic](https://img.shields.io/badge/ROS-Noetic-blue)
![Ubuntu 20.04](https://img.shields.io/badge/Ubuntu-20.04-orange)
![Gazebo 11](https://img.shields.io/badge/Gazebo-11-green)
![分支](https://img.shields.io/badge/Branch-Course%20Bonus-lightgrey)

本分支用于保留 `MEE5115 Autonomous Robotic Systems` 课程项目的 Go2W
加分仿真实现。它把 Go2W 模型、RL-SAR 运动控制、Gazebo、AMCL、
`move_base`、Navfn、DWA 和 RViz 目标点输入整合到同一条 ROS Noetic
启动链路中。

## 分支定位

`go2w-navigation` 是课程加分展示分支，不是推荐的后续开发主线。

| 分支 | 定位 |
|---|---|
| `main` | 基线框架、后续开发和学习的推荐入口 |
| `go2w-navigation` | Go2W + RL-SAR + ROS Navigation 仿真链路的课程加分分支 |

本分支用于展示课程项目中基线和加分内容的完成结果，不计划合并回 `main`。

## 与 `main` 的关系

后续开发和最新基线框架请从
[`main` 分支](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
开始。

如果你是第一次学习本仓库，建议先阅读 `main` 分支 README。`main` 分支说明的是
可复用基线工作空间，包括 TurtleBot3 / WPB Home 仿真、SLAM、导航、传统路径
规划、覆盖路径规划和通用文档体系。本分支更聚焦，只保留 Go2W 课程加分链路的
可复现实验说明。

## 演示

![Go2W 导航加分演示](docs/assets/08_go2w_bonus.gif)

该 GIF 展示 Go2W 在 `maze_2` 场景中接收 RViz `2D Nav Goal` 后，由 ROS
Navigation 生成速度命令，再由 RL-SAR 策略把速度命令转换为 Gazebo 中的轮足运动控制。

## 本分支新增内容

相对基线导航工作空间，本分支新增并保留了以下真实存在的 Go2W 相关内容：

| 类别 | 文件或包 |
|---|---|
| Go2W 机器人模型 | `src/go2w_description/` |
| RL-SAR 仿真控制器 | `src/rl_sar/` |
| 电机消息和 Gazebo 关节控制器 | `src/robot_msgs/`、`src/robot_joint_controller/` |
| Go2W 策略文件 | `policy/go2w/base.yaml`、`policy/go2w/robot_lab/policy.pt` |
| 推理运行时下载脚本 | `scripts/download_inference_runtime.sh` |
| Go2W 导航启动入口 | `src/mr_navigation/launch/go2w_navigation_sim.launch` |
| 速度适配层 | `src/mr_navigation/scripts/cmd_vel_filter.py` |
| Gazebo 里程计桥接 | `src/mr_gazebo/scripts/gazebo_model_odom.py` |
| Go2W 代价地图和 DWA 参数 | `src/mr_navigation/config/*_go2w.yaml` |
| 迷宫导航配置 | `src/mr_gazebo/worlds/maze_2.world`、`src/mr_maps/maps/maze_2_hector.yaml` |

## 系统概览

本分支使用标准 ROS Navigation 栈完成全局和局部规划，然后把速度命令适配给
RL-SAR 运动控制：

```text
RViz 2D Nav Goal
  -> map_server + AMCL
  -> NavfnROS
  -> DWAPlannerROS
  -> /move_base_cmd_vel
  -> cmd_vel_filter.py
  -> /cmd_vel
  -> rl_sar / rl_sim
  -> policy/go2w/robot_lab/policy.pt
  -> robot_joint_controller
  -> Gazebo Go2W
```

## 数据流

命令链路：

```text
RViz 2D Nav Goal
-> NavfnROS
-> DWAPlannerROS
-> /move_base_cmd_vel
-> cmd_vel_filter.py
-> /cmd_vel
-> rl_sar / rl_sim
-> policy.pt
-> robot_joint_controller
-> Gazebo Go2W
```

反馈链路：

```text
Gazebo model states
-> gazebo_model_odom.py
-> /odom + odom -> base_footprint
-> AMCL
-> map -> odom
```

激光链路：

```text
Gazebo Go2W base_scan
-> /scan
-> AMCL + 局部/全局代价地图
```

## 环境要求

目标环境：

| 项目 | 期望值 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| 仿真器 | Gazebo 11 |
| 主要策略运行时 | LibTorch |
| 导航栈 | `map_server`、AMCL、`move_base`、Navfn、DWA |

TorchScript 策略文件应位于：

```text
policy/go2w/robot_lab/policy.pt
```

## 编译

```bash
cd ~/mobile-robot-planning
source /opt/ros/noetic/setup.bash

bash scripts/download_inference_runtime.sh libtorch
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
source devel/setup.bash
```

使用 `BUILD_RL_REAL_TARGETS=OFF` 是因为本分支只运行 Gazebo 仿真中的 RL-SAR
控制链路，不编译依赖厂商 SDK 的硬件目标。

## 快速启动

```bash
cd ~/mobile-robot-planning
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch mr_navigation go2w_navigation_sim.launch
```

默认启动参数：

| 项目 | 值 |
|---|---|
| 世界 | `mr_gazebo/worlds/maze_2.world` |
| 地图 | `maze_2_hector` |
| 机器人 | `go2w` |
| Gazebo 模型 | `go2w_gazebo` |
| 策略 | `policy/go2w/robot_lab/policy.pt` |
| 速度链路 | `/move_base_cmd_vel -> /cmd_vel` |
| 导航规划器 | `NavfnROS + DWAPlannerROS` |

在 RViz 中：

1. 等待 Gazebo、RL-SAR 起立、运动模式、AMCL 和 `move_base` 稳定。
2. 如果机器人位姿和地图不重合，使用 `2D Pose Estimate`。
3. 使用 `2D Nav Goal` 发送目标。

## 常用命令

```bash
rostopic echo /move_base_cmd_vel
rostopic echo /cmd_vel
rostopic hz /scan
rostopic hz /odom
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo base_footprint base
```

常用启动参数：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch x:=1.7 y:=0.8 yaw:=1.5708
roslaunch mr_navigation go2w_navigation_sim.launch rviz:=false
roslaunch mr_navigation go2w_navigation_sim.launch auto_locomotion_delay:=9.0
roslaunch mr_navigation go2w_navigation_sim.launch use_cmd_vel_filter:=false
```

## 如何验证导航链路

启动后从感知到控制依次检查：

| 检查项 | 命令 | 期望结果 |
|---|---|---|
| 激光 | `rostopic hz /scan` | 约 10 Hz |
| 里程计 | `rostopic hz /odom` | 约 30 Hz |
| DWA 原始速度 | `rostopic echo /move_base_cmd_vel` | 有效目标发送后出现非零命令 |
| RL 过滤后速度 | `rostopic echo /cmd_vel` | 限幅和平滑后的命令 |
| 定位 TF | `rosrun tf tf_echo map base_footprint` | 连续变换 |
| 底盘高度 TF | `rosrun tf tf_echo base_footprint base` | 静态高度偏移 |

期望 TF 链路为：

```text
map -> odom -> base_footprint -> base -> trunk -> base_scan
```

更详细的检查见
[docs/zh/go2w_rl_sar_navigation.md](docs/zh/go2w_rl_sar_navigation.md)。

## 文档整理

原根目录 Go2W 说明已整理为正式分支文档：

| 文档 | 用途 |
|---|---|
| [docs/go2w_rl_sar_navigation.md](docs/go2w_rl_sar_navigation.md) | 英文使用者文档 |
| [docs/go2w_integration_guide.md](docs/go2w_integration_guide.md) | 英文开发者文档 |
| [docs/zh/go2w_rl_sar_navigation.md](docs/zh/go2w_rl_sar_navigation.md) | 中文使用者文档 |
| [docs/zh/go2w_integration_guide.md](docs/zh/go2w_integration_guide.md) | 中文开发者文档 |

分支演示资源保存在：

```text
docs/assets/08_go2w_bonus.gif
```

## 说明和限制

- 本分支是课程加分分支，重点是 Go2W 仿真导航闭环。
- 默认链路是 Navfn + DWA + RL-SAR 运动策略。
- Go2W 底层运动由预训练策略产生，本仓库不重新训练该策略。
- 真机部署不属于本分支文档范围。
- 本分支不是 `main` 的替代开发分支。
- 如果要继续扩展移动机器人规划框架，请从 `main` 分支开始。
- `dwa_local_planner_params_go2w.yaml` 中仍保留私有参数
  `DWAPlannerROS/controller_frequency: 10.0`；启动文件会把 `move_base`
  控制频率覆盖为 20 Hz。

## 排错

### `unitree_sdk2` 缺少 `CMakeLists.txt`

**原因：** Gazebo 仿真演示不需要硬件 SDK 源码，但未关闭的硬件目标可能会查找这些源码。

**处理：**

```bash
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

### `/move_base_cmd_vel` 有变化但 `/cmd_vel` 太小

**原因：** 速度过滤器正在限制线速度、角速度或加速度。

**处理：** 检查 `go2w_navigation_sim.launch` 中传给 `cmd_vel_filter.py` 的参数，
尤其是 `cmd_vel_min_x`、`cmd_vel_max_x`、`cmd_vel_max_yaw` 和加速度限制。

### `/move_base_cmd_vel` 接近零

**原因：** 局部规划器可能没有有效定位、有效目标或有效局部代价地图。

**处理：** 在修改 RL-SAR 参数前，先检查 AMCL、RViz 目标、`/scan`、`/odom` 和 TF 对齐。

### 激光点和地图不重合

**原因：** AMCL 初始位姿、地图原点或 `map -> odom -> base_scan` 变换可能不正确。

**处理：** 使用 `2D Pose Estimate`，然后检查：

```bash
rostopic echo -n 1 /scan/header
rosrun tf tf_echo map base_scan
```

### Go2W 到达目标区域后持续旋转

**原因：** 对当前轮足迷宫演示来说，严格最终航向对齐并不必要。

**处理：** 本分支使用较宽松的航向容差，并在 `cmd_vel_filter.py` 中启用到点停车锁存。
请确认启动文件中的 `goal_stop_enabled` 和 `goal_xy_tolerance`。

## 参考

- [仓库 `main` 分支](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
- [fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar)：本分支 Go2W 课程
  加分集成参考的上游仓库
- [ROS Navigation 栈](http://wiki.ros.org/navigation)
- 导入到 `src/rl_sar`、`src/go2w_description`、`policy/go2w` 和
  `src/robot_joint_controller` 的 RL-SAR、Go2W 策略和模型资源
- 本分支文档：
  [Go2W 导航说明](docs/zh/go2w_rl_sar_navigation.md) 和
  [Go2W 集成说明](docs/zh/go2w_integration_guide.md)
