<div align="right">

[英文](../go2w_rl_sar_navigation.md)

</div>

# Go2W RL-SAR 导航说明

本文是 `go2w-navigation` 分支中 Go2W 课程加分演示的使用者文档，重点说明
如何编译、启动、验证和排错。

如果要学习更完整的基线框架或继续开发，请从
[`main` 分支](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
开始。

## 范围

本分支运行以下 Gazebo 仿真链路：

```text
Go2W model
-> Gazebo laser and model states
-> AMCL + move_base
-> NavfnROS + DWAPlannerROS
-> cmd_vel_filter.py
-> rl_sar / rl_sim
-> policy/go2w/robot_lab/policy.pt
-> robot_joint_controller
```

本文不是 Go2W 真机部署说明，也不覆盖策略训练。

## 编译

```bash
cd ~/mobile-robot-planning
source /opt/ros/noetic/setup.bash

bash scripts/download_inference_runtime.sh libtorch
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
source devel/setup.bash
```

该编译参数会关闭依赖厂商 SDK 的硬件目标。Gazebo 仿真节点 `rl_sar/rl_sim`
仍会参与编译。

## 启动

```bash
roslaunch mr_navigation go2w_navigation_sim.launch
```

默认启动链路使用：

| 项目 | 值 |
|---|---|
| 世界 | `$(find mr_gazebo)/worlds/maze_2.world` |
| 地图 | `maze_2_hector` |
| 机器人键名 | `go2w` |
| Gazebo 模型名 | `go2w_gazebo` |
| 初始位姿 | `x=1.7`、`y=0.8`、`z=0.55`、`yaw=1.5708` |
| 底盘高度 TF | `base_footprint -> base`，`z=0.34` |
| RL 策略 | `policy/go2w/robot_lab/policy.pt` |
| 原始速度话题 | `/move_base_cmd_vel` |
| 过滤后速度话题 | `/cmd_vel` |

## RViz 操作流程

1. 等待 Gazebo 加载 `maze_2.world`。
2. 等待 Go2W 模型和控制器生成。
3. 等待 RL-SAR 完成起立并进入运动模式。
4. 确认 AMCL、`move_base` 和 RViz 已运行。
5. 如果位姿没有和地图对齐，使用 `2D Pose Estimate`。
6. 使用 `2D Nav Goal` 发送目标。

## 常用启动参数

设置初始位姿：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  x:=1.7 y:=0.8 z:=0.55 yaw:=1.5708
```

不启动 RViz：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch rviz:=false
```

调整自动进入运动模式的等待时间：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  auto_locomotion_delay:=9.0
```

调整统一导航命令频率：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  rl_cmd_vel_input_rate:=10.0
```

关闭速度过滤器进行对比：

```bash
roslaunch mr_navigation go2w_navigation_sim.launch \
  use_cmd_vel_filter:=false
```

该模式只建议用于定位问题；默认演示使用速度过滤链路。

## 数据流

导航命令链路：

```text
RViz 2D Nav Goal
-> NavfnROS global plan
-> DWAPlannerROS local command
-> /move_base_cmd_vel
-> cmd_vel_filter.py
-> /cmd_vel
-> rl_sar / rl_sim
-> policy.pt
-> RobotCommand / MotorCommand
-> robot_joint_controller
-> Gazebo Go2W
```

定位和反馈链路：

```text
/gazebo/model_states
-> gazebo_model_odom.py
-> /odom
-> odom -> base_footprint
-> AMCL
-> map -> odom
```

感知链路：

```text
base_scan 上的 Gazebo 射线传感器
-> /scan
-> AMCL
-> 全局/局部代价地图
```

期望 TF 链路为：

```text
map
-> odom
-> base_footprint
-> base
-> trunk
-> base_scan
```

## 运行频率

| 信号 | 期望频率 |
|---|---:|
| `/scan` | 约 10 Hz |
| `/odom` | 约 30 Hz |
| `move_base` 控制循环 | 启动参数默认为 20 Hz |
| `cmd_vel_filter.py` 输出 | 默认 20 Hz |
| RL-SAR 策略推理 | 由 `dt=0.005` 和 `decimation=4` 得到 50 Hz |

RL 策略可以在两次导航命令更新之间复用最近一次速度命令，这是正常行为。

## 验证命令

检查节点：

```bash
rosnode list | grep -E "gazebo|rl_sar|cmd_vel|amcl|move_base|map_server|robot_state"
```

关键节点应包括：

```text
/gazebo
/rl_sar
/go2w_cmd_vel_filter
/go2w_gazebo_odom
/amcl
/move_base
/map_server
/robot_state_publisher
```

检查话题：

```bash
rostopic list | grep -E "/scan|/odom|cmd_vel|model_states|move_base"
```

检查频率：

```bash
rostopic hz /scan
rostopic hz /odom
rostopic hz /move_base_cmd_vel
rostopic hz /cmd_vel
```

对比原始速度和过滤后速度：

```bash
rostopic echo /move_base_cmd_vel
rostopic echo /cmd_vel
```

检查 TF：

```bash
rosrun tf tf_echo map odom
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint base
rosrun tf tf_echo base base_scan
```

检查路径：

```bash
rostopic echo /move_base/NavfnROS/plan
rostopic echo /move_base/DWAPlannerROS/local_plan
```

## 手动 `/cmd_vel` 测试

只在 `move_base` 和速度过滤器没有同时向 `/cmd_vel` 发布时使用该测试。

```bash
rostopic pub -r 4 /cmd_vel geometry_msgs/Twist \
"linear:
  x: 0.30
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.20"
```

建议手动测试范围：

```text
linear.x: 0.20 to 0.50
linear.y: 0.0
angular.z: -0.50 to 0.50
rate: 4 to 20 Hz
```

手动控制前检查发布者：

```bash
rostopic info /cmd_vel
```

## 排错

### `unitree_sdk2` 缺少 `CMakeLists.txt`

**原因：** 当前 Gazebo 演示不需要硬件 SDK 源码，但硬件目标未关闭时可能会查找它。

**处理：**

```bash
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

### 缺少 `robot_msgs/MotorCommand.h`

**原因：** 依赖目标编译时，消息头文件可能还没有生成完成。

**处理：** 确认 `src/robot_msgs/msg/MotorCommand.msg` 存在，然后重新编译。如果怀疑
CMake 缓存过期，可先清理 `build` 和 `devel`。

```bash
rm -rf build devel
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
```

### Go2W 运动异常

**原因：** 常见原因包括多个 `/cmd_vel` 发布者、RL-SAR 尚未进入运动模式、
Gazebo 模型名错误或速度命令跳变过大。

**处理：**

```bash
rostopic info /cmd_vel
rostopic echo /cmd_vel
rosparam get /gazebo_model_name
```

默认 Gazebo 模型名应为：

```text
go2w_gazebo
```

### 激光和地图不重合

**原因：** 激光是 Gazebo 实时测量，不是静态地图。错位通常来自 AMCL 初始位姿、
地图原点或 TF 对齐问题。

**处理：**

```bash
rostopic echo -n 1 /scan/header
rosrun tf tf_echo map base_scan
```

如果机器人位姿明显偏移，使用 RViz 的 `2D Pose Estimate`。

### 有效目标发送后机器人几乎不动

**原因：** DWA 输出速度可能过低，或速度过滤器在 RL 策略接收前压低了命令。

**处理：** 对比：

```bash
rostopic echo /move_base_cmd_vel
rostopic echo /cmd_vel
```

如果 `/move_base_cmd_vel` 非零但 `/cmd_vel` 太小，调整 `cmd_vel_filter.py`
相关启动参数。如果 `/move_base_cmd_vel` 几乎为零，先检查 AMCL、局部代价地图和
RViz 目标。

### 到达目标后仍持续旋转

**原因：** 对当前轮足迷宫演示来说，严格最终航向对齐并不必要。

**处理：** 保留本分支使用的宽松航向容差和到点停车锁存：

```text
xy_goal_tolerance: 0.22
yaw_goal_tolerance: 3.14159
goal_stop_enabled: true
goal_stop_latch: true
```

### RViz 中 RobotModel 位于地图平面以下

**原因：** 二维导航应使用平面 `base_footprint`，而 URDF 根链接需要在其上方保持高度偏移。

**处理：**

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_footprint base
```

期望值：

```text
odom -> base_footprint: z = 0
base_footprint -> base: z = 0.34
```

## 说明和限制

- 本分支展示 Gazebo 中的 Go2W 导航闭环。
- RL-SAR 策略是预训练策略，位于 `policy/go2w/robot_lab`。
- 本分支不重新训练策略。
- 本分支不替代 `main` 作为推荐开发入口。
- 当前默认使用前进加偏航控制；启动配置会把侧向速度限制为 `0.0`。

## 参考

- [仓库 `main` 分支](https://github.com/Mingyang-Sheep/mobile-robot-planning/tree/main)
- [ROS Navigation 栈](http://wiki.ros.org/navigation)
- [fan-ziqi / rl_sar](https://github.com/fan-ziqi/rl_sar)：Go2W 课程加分集成中
  RL-SAR 部分参考的上游仓库
- 本地 RL-SAR 和 Go2W 资源位于 `src/rl_sar`、`src/go2w_description`、
  `policy/go2w` 和 `src/robot_joint_controller`
