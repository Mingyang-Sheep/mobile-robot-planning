<div align="right">

[英文](../go2w_integration_guide.md)

</div>

# Go2W + RL-SAR 集成说明

本文总结 `go2w-navigation` 分支背后的文件级集成内容，面向需要理解 Go2W 课程
加分演示如何接入原移动机器人规划工作空间的开发者。

如果只是运行演示，请先阅读
[go2w_rl_sar_navigation.md](go2w_rl_sar_navigation.md)。如果要继续开发基线
框架，请从
[`main` 分支](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
开始。

## 集成目标

本分支集成了：

- `mr_gazebo` 中的 `maze_2.world`。
- `mr_maps` 中的 `maze_2_hector`。
- Go2W URDF、网格、Gazebo 配置和激光坐标系。
- RL-SAR Gazebo 仿真控制。
- `policy/go2w/robot_lab/policy.pt`。
- 基于 `map_server`、AMCL、Navfn、DWA 和 RViz 目标的 ROS Navigation。
- 从 DWA 到 RL-SAR 的过滤速度桥接。
- 从 Gazebo 模型状态到 ROS Navigation 的平面里程计桥接。

## 导入的源码集合

| 源码集合 | 目标路径 | 用途 |
|---|---|---|
| RL-SAR 控制器 | `src/rl_sar` | 策略推理、状态机、Gazebo 仿真节点 |
| 机器人消息 | `src/robot_msgs` | 电机和机器人状态命令消息 |
| Gazebo 关节控制器 | `src/robot_joint_controller` | 把 RL-SAR 命令作用到 Gazebo 关节 |
| Go2W 描述包 | `src/go2w_description` | URDF、xacro、网格、Gazebo 传感器和控制配置 |
| Go2W 策略 | `policy/go2w` | 机器人基础参数和 `robot_lab` TorchScript 策略 |
| 运行时脚本 | `scripts` | LibTorch 和 ONNX Runtime 下载脚本 |

Go2W 课程加分集成参考
[fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar) 作为 RL-SAR 上游仓库。
本地包元数据也在 `src/rl_sar/package.xml` 中记录了 RL-SAR 包维护者和 Apache-2.0
许可证信息。

## 为导航新增的自定义文件

| 文件 | 作用 |
|---|---|
| `src/mr_navigation/launch/go2w_navigation_sim.launch` | 统一启动 Gazebo、Go2W、RL-SAR、地图、AMCL、`move_base`、速度过滤器和 RViz |
| `src/mr_navigation/scripts/cmd_vel_filter.py` | 对 DWA 速度命令限幅、平滑并重新发布给 RL-SAR |
| `src/mr_gazebo/scripts/gazebo_model_odom.py` | 把 `/gazebo/model_states` 转换为平面 `/odom` 和 `odom -> base_footprint` |
| `src/mr_navigation/config/costmap_common_params_go2w.yaml` | Go2W 轮廓、障碍物、清障、膨胀和 `/scan` 观测配置 |
| `src/mr_navigation/config/dwa_local_planner_params_go2w.yaml` | Go2W 专用 DWA 速度、加速度、容差和评分配置 |
| `src/mr_maps/maps/maze_2.*` | 迷宫地图集合 |
| `src/mr_maps/maps/maze_2_hector.*` | Go2W 启动入口使用的导航地图 |

## Go2W 复用的已有启动文件

| 文件 | Go2W 用途 |
|---|---|
| `src/mr_maps/launch/map_loader.launch` | 加载 `$(find mr_maps)/maps/$(arg map_name).yaml` |
| `src/mr_navigation/launch/navigation.launch` | 连接地图、AMCL 和 `move_base` |
| `src/mr_navigation/launch/amcl.launch` | 接收激光话题、里程计模型和初始位姿 |
| `src/mr_navigation/launch/move_base.launch` | 通过 `model:=go2w` 加载 `*_go2w.yaml` 参数文件 |

## Go2W 启动组成

`go2w_navigation_sim.launch` 启动：

1. 带 `maze_2.world` 的 Gazebo 空世界。
2. 通过 `robot_description` 加载 Go2W xacro。
3. `robot_state_publisher`。
4. 静态 `base_footprint -> base` 变换。
5. 以 `go2w_gazebo` 名称生成 Gazebo 模型。
6. `go2w_description/config/robot_control.yaml`。
7. `gazebo_model_odom.py`。
8. `rl_sar/rl_sim`。
9. 启用时启动 `cmd_vel_filter.py`。
10. `navigation.launch`。
11. 使用现有导航视图的 RViz。

重要默认值：

```text
model: go2w
map_name: maze_2_hector
world_name: $(find mr_gazebo)/worlds/maze_2.world
robot_name: go2w
gazebo_model_name: go2w_gazebo
x: 1.7
y: 0.8
z: 0.55
yaw: 1.5708
robot_base_height: 0.34
auto_start_rl: true
start_navigation_mode: true
use_cmd_vel_filter: true
```

## 策略和运行时

策略目录必须包含：

```text
policy/go2w/
|-- base.yaml
`-- robot_lab/
    |-- config.yaml
    `-- policy.pt
```

`base.yaml` 定义 16 个自由度、轮子索引、PD 增益、关节名称，以及：

```text
dt: 0.005
decimation: 4
```

因此策略周期为 `0.02 s`，也就是 50 Hz。导航速度接口默认更慢：

```text
move_base controller loop: 20 Hz
cmd_vel_filter output: 20 Hz
rl_sim cmd_vel input limit: 20 Hz
Gazebo odometry bridge: 30 Hz
laser: 10 Hz
```

使用以下命令把 LibTorch 下载到 `library/inference_runtime`：

```bash
bash scripts/download_inference_runtime.sh libtorch
```

`src/rl_sar/CMakeLists.txt` 会从工作空间根目录解析 `policy` 和
`library/inference_runtime`。

## 硬件目标编译开关

当前分支使用：

```bash
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

`src/rl_sar/CMakeLists.txt` 定义：

```cmake
option(
  BUILD_RL_REAL_TARGETS
  "Build real-robot hardware executables that require vendor SDKs"
  OFF
)
```

依赖硬件 SDK 的可执行目标由该选项和 SDK 路径检查共同保护，使 Gazebo 仿真编译不依赖
缺失的厂商 SDK 子模块。

## 包和 CMake 调整

导入的 RL-SAR 源码集合在同一个 Catkin 工作空间中和基线移动机器人包一起编译。
关键集成点如下：

| 包 | 调整 |
|---|---|
| `src/rl_sar` | 使用 Catkin 包清单，依赖 `robot_msgs` 和 `robot_joint_controller`，并为 Gazebo 链路编译 `rl_sim` |
| `src/robot_msgs` | 在依赖目标编译前生成 `MotorCommand`、`MotorState`、`RobotCommand` 和 `RobotState` 消息 |
| `src/robot_joint_controller` | 链接 Catkin 库，并依赖导出的消息目标 |
| `src/go2w_description` | 增加 Go2W 模型需要的 Gazebo、Gazebo 插件和 xacro 依赖 |
| `src/mr_gazebo` | 安装 `scripts/gazebo_model_odom.py` Python 节点 |
| `src/mr_navigation` | 安装 `scripts/cmd_vel_filter.py` Python 节点 |

消息生成顺序很重要。如果 `robot_joint_controller` 或 `rl_sim` 在 `robot_msgs`
头文件生成前编译，就可能出现 `robot_msgs/MotorCommand.h` 缺失。当前分支通过给依赖
目标增加 Catkin 导出目标依赖来避免该问题。

Go2W 描述包需要仿真和 xacro 依赖，因为启动文件会展开：

```text
$(find go2w_description)/xacro/robot.xacro
```

其中包含的 Gazebo xacro 定义了本演示使用的射线传感器、IMU 插件和控制接口。

## 激光集成

`src/go2w_description/xacro/robot.xacro` 在 `trunk` 上添加 `base_scan`：

```text
trunk -> base_scan
origin: xyz="0 0 0.20"
```

`src/go2w_description/xacro/gazebo.xacro` 添加 Gazebo 射线传感器：

| 属性 | 值 |
|---|---|
| 话题 | `/scan` |
| 坐标系 | `base_scan` |
| 更新频率 | 10 Hz |
| 量程 | 0.12 m 到 10.0 m |
| 水平采样数 | 720 |
| 视场角 | 360 度 |

Go2W 代价地图配置通过 `observation_sources: scan` 使用该激光。

## 里程计桥接

`gazebo_model_odom.py` 订阅：

```text
/gazebo/model_states
```

并发布：

```text
/odom
odom -> base_footprint
```

该桥接节点：

- 按名称查找 Gazebo 模型，而不是使用固定数组下标。
- 使用模型的 `x`、`y` 和 yaw。
- 将里程计 `z` 设为 `0.0`。
- 从导航位姿中移除 roll 和 pitch。
- 将世界坐标系线速度转换到机体坐标系后写入 `Odometry.twist` 字段。

这样二维导航栈就不会受到轮足机器人高度、俯仰和横滚运动的直接影响。

## 速度过滤器

`cmd_vel_filter.py` 在 RL-SAR 接收 DWA 命令前进行适配。

默认命令路径：

```text
move_base cmd_vel remap
-> /move_base_cmd_vel
-> cmd_vel_filter.py
-> /cmd_vel
-> rl_sar / rl_sim
```

主要功能：

- 限制线速度和角速度。
- 通过 `max_y=0.0` 禁止侧向速度。
- 平滑加速和减速。
- 按固定频率发布。
- 命令超时后停车。
- 当前目标进入 `goal_xy_tolerance` 后停车。
- 可选支持原地转向行为，默认关闭。

启动文件中的重要默认值：

```text
cmd_vel_min_x: 0.16
cmd_vel_max_x: 0.65
cmd_vel_max_yaw: 1.00
cmd_vel_max_acc_x: 0.60
cmd_vel_max_acc_yaw: 1.00
cmd_vel_max_decel_x: 1.50
cmd_vel_max_decel_yaw: 2.00
goal_xy_tolerance: 0.22
goal_stop_enabled: true
goal_stop_latch: true
```

## RL-SAR 导航输入

仿真节点从 `go2w_navigation_sim.launch` 接收私有参数：

```text
cmd_vel_topic
cmd_vel_input_rate
auto_start
start_navigation
auto_locomotion_delay
```

该节点订阅 `/cmd_vel`，限制命令更新频率，并把最近一次速度命令复制到策略观测命令：

```text
commands = [linear.x, linear.y, angular.z]
```

启动文件设置了 `auto_start=true` 和 `start_navigation=true`，因此演示不需要在
RViz 目标导航前手动通过键盘切换状态。

## 代价地图和 DWA 参数

Go2W 代价地图参数：

| 参数 | 值 |
|---|---|
| `obstacle_range` | `4.0` |
| `raytrace_range` | `5.0` |
| `footprint` | `[[-0.35,-0.22], [-0.35,0.22], [0.35,0.22], [0.35,-0.22]]` |
| `inflation_radius` | `0.35` |
| `cost_scaling_factor` | `3.0` |
| `scan.sensor_frame` | `base_scan` |
| `scan.topic` | `/scan` |

Go2W DWA 参数：

| 参数 | 值 |
|---|---|
| `max_vel_x` | `0.65` |
| `min_vel_x` | `0.16` |
| `max_vel_y` | `0.0` |
| `max_vel_theta` | `1.0` |
| `xy_goal_tolerance` | `0.22` |
| `yaw_goal_tolerance` | `3.14159` |
| `path_distance_bias` | `32.0` |
| `goal_distance_bias` | `24.0` |
| `occdist_scale` | `0.08` |

DWA 参数文件仍保留：

```text
DWAPlannerROS/controller_frequency: 10.0
```

启动文件会把 `move_base` 控制频率覆盖为 20 Hz。如果后续继续收敛该分支，可以把这个
DWA 私有参数也同步为启动参数。

## 地图

Go2W 演示使用：

```text
src/mr_gazebo/worlds/maze_2.world
src/mr_maps/maps/maze_2_hector.yaml
src/mr_maps/maps/maze_2_hector.pgm
```

地图 YAML 应使用相对图片路径，例如：

```yaml
image: maze_2_hector.pgm
```

避免使用机器相关的绝对路径，以保证分支可移植。

## 与 TurtleBot3 共存

Go2W 使用独立启动入口：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch
```

已有 TurtleBot3 导航入口保持独立：

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=burger \
  map_name:=maze_2_hector
```

| 项目 | TurtleBot3 路径 | Go2W 路径 |
|---|---|---|
| 模型 | `mr_description` | `go2w_description` |
| 底层控制 | 差速驱动插件 | RL-SAR 策略 |
| 速度话题 | `/cmd_vel` | `/move_base_cmd_vel -> /cmd_vel` |
| 里程计 | 差速驱动插件 | `gazebo_model_odom.py` |
| 局部参数 | `*_burger.yaml` | `*_go2w.yaml` |
| 起立状态 | 不需要 | RL-SAR 状态机 |

## 验收清单

编译：

- `catkin_make -DBUILD_RL_REAL_TARGETS=OFF` 成功。
- 编译不需要缺失的厂商 SDK 源码。
- `robot_msgs` 头文件在依赖目标编译前生成。

模型和控制：

- Go2W 在 Gazebo 中以 `go2w_gazebo` 生成。
- RL-SAR 自动进入运动模式。
- 没有其他节点发布 `/cmd_vel` 时，手动 `/cmd_vel` 可以驱动机器人。

感知：

- `/scan` 约 10 Hz。
- `/scan.header.frame_id` 为 `base_scan`。
- AMCL 位姿修正后，激光点与迷宫墙体基本重合。

定位和 TF：

- `/odom` 约 30 Hz。
- `map -> odom -> base_footprint -> base -> base_scan` 连通。
- RViz RobotModel 不位于地图平面以下。

导航：

- Navfn 发布全局路径。
- DWA 发布局部路径。
- 有效目标发送后 `/move_base_cmd_vel` 发生变化。
- `/cmd_vel` 被平滑，且 `linear.y = 0`。
- Go2W 向目标移动，并在目标附近停止。

## 参考

- [仓库 `main` 分支](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
- [ROS Navigation 栈](http://wiki.ros.org/navigation)
- [fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar)
- 导入到 `src/rl_sar` 的 RL-SAR 项目源码
- 导入到 `src/go2w_description`、`src/robot_joint_controller` 和 `policy/go2w`
  的 Go2W / Unitree 相关模型、控制和策略资源
