<div align="right">

[英文版](../import_robot_urdf_to_navigation.md)

</div>

# 外部机器人 URDF 移植到 Gazebo / SLAM / 导航

适合读者：已经有一个外部机器人 URDF/mesh，希望接入本仓库 Gazebo、SLAM、导航 和规划链路的开发者。

移植目标不是“RViz 能看到模型”这么简单。可导航机器人至少需要：

```text
URDF/Xacro
  -> robot_description
  -> TF
  -> Gazebo 运动插件
  -> /scan
  -> /odom
  -> /cmd_vel
  -> SLAM 或 map_server
  -> AMCL / move_base
```

## 1. 推荐原则

- 保留官方原始 URDF、mesh 和 RViz 配置。
- 单独增加 `simulation/` Xacro 层，放 Gazebo 插件和仿真专用 joint。
- 不用 box/cylinder 代替官方 mesh。
- 不把仿真插件直接写回官方原始 URDF。
- 不随意改 坐标系 名；如果必须改，必须同步更新 代价地图、AMCL、SLAM 和 RViz。

## 2. 推荐目录

```text
src/mr_description/
  urdf/<robot_name>/
    original files
    simulation/
      <robot>_model.urdf.xacro
      <robot>_sim.urdf.xacro
      <robot>_gazebo_plugins.xacro
  meshes/<robot_name>/
  rviz/<robot_name>/
  config/<robot_name>/

src/mr_navigation/config/
  costmap_common_params_<robot>.yaml
  dwa_local_planner_params_<robot>.yaml
  amcl_params_<robot>.yaml, if needed
```

## 3. 最小导航闭环

一个机器人要能跑 导航，至少检查：

| 接口 | 要求 |
|---|---|
| `/map` | 来自 `map_server` 或 SLAM |
| `/scan` | 2D LaserScan，坐标系 与代价地图一致 |
| `/odom` | 里程计，提供 `odom -> base_footprint` |
| `/tf` | 有 `map -> odom -> base_footprint -> sensor` |
| `/cmd_vel` | 底盘插件订阅并驱动机器人 |

只在 RViz 看到模型，不代表导航闭环完成。

## 4. 接入步骤

### 步骤 1：复制官方资源

复制官方 URDF/Xacro、mesh、RViz 和配置文件：

```text
official_package/urdf/
official_package/meshes/
official_package/rviz/
official_package/config/
```

将 `package://official_pkg/...` 替换为当前仓库路径，例如：

```text
package://wpb_home_bringup/meshes/... -> package://mr_description/meshes/wpb_home/...
```

### 步骤 2：新增 simulation Xacro

如果官方模型没有标准差速轮 joint，需要在 simulation 层新增隐藏轮 joint，并使用：

```text
libgazebo_ros_diff_drive.so
```

底盘插件至少要对齐：

```text
command话题: cmd_vel
odometry话题: odom
odometryFrame: odom
robotBaseFrame: base_footprint
```

### 步骤 3：新增传感器插件

导航至少需要 2D 激光：

```text
libgazebo_ros_laser.so
topicName: scan
frameName: <laser_frame>
```

如果有相机或 IMU，再增加对应 Gazebo 插件。导航主链路当前主要使用 `/scan`。

### 步骤 4：登记模型

编辑：

```text
src/mr_navigation/config/robot_models.yaml
```

新增字段至少包括：

```text
xacro
base_frame
laser_frame
odom_frame
scan_topic
cmd_vel_topic
odom_topic
nav_config_key
```

### 步骤 5：增加导航参数

至少新增：

```text
costmap_common_params_<robot>.yaml
dwa_local_planner_params_<robot>.yaml
```

重点检查：

- footprint；
- laser `sensor_frame`；
- 激光话题；
- 最大速度和加速度；
- 局部代价地图 宽高；
- inflation 半径。

### 步骤 6：验证 Xacro

```bash
cd ~/mobile-robot-planning
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun xacro xacro $(rospack find mr_description)/urdf/<robot>/simulation/<robot>_sim.urdf.xacro > /tmp/<robot>_sim.urdf
check_urdf /tmp/<robot>_sim.urdf
```

### 步骤 7：验证仿真 话题

```bash
roslaunch mr_navigation simulation.launch model:=<robot> robot_model:=<robot>
```

另开终端检查：

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rostopic info /cmd_vel
rosrun tf tf_echo odom base_footprint
```

### 步骤 8：接入 导航

使用已有地图/世界文件 配对先验证：

```bash
roslaunch mr_navigation navigation_sim.launch \
  model:=<robot> \
  robot_model:=<robot> \
  map_name:=turtlebot3_world \
  world_name:=$(rospack find mr_gazebo)/worlds/turtlebot3_world.world
```

成功后再迁移到自己的 世界文件 和 map。

## 5. WPB Home 当前移植记录

官方来源：

```text
https://github.com/6-robot/wpb_home.git
https://github.com/6-robot/wpr_simulation.git
```

原始文件：

```text
src/mr_description/urdf/wpb_home/wpb_home.urdf
src/mr_description/urdf/wpb_home/wpb_home_mani.urdf
src/mr_description/meshes/wpb_home/
src/mr_description/rviz/wpb_home/
src/mr_description/config/wpb_home/wpb_home.yaml
```

仿真适配：

```text
src/mr_description/urdf/wpb_home/simulation/wpb_home_sim.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_mani_sim.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_gazebo_plugins.xacro
```

当前处理：

- 官方 WPB Home / WPR 仿真模型没有标准左右轮 joint；
- 本仓库在 simulation 层增加隐藏轮 joint；
- 使用标准 `gazebo_ros_diff_drive` 接入 `/cmd_vel` 和 `/odom`；
- WPB Home laser 坐标系 为 `laser`；
- `wpb_home_mani` 当前只作为移动底盘导航模型，未实现机械臂控制。

验证启动：

```bash
roslaunch mr_navigation navigation_sim.launch model:=wpb_home robot_model:=wpb_home
roslaunch mr_navigation navigation_sim.launch model:=wpb_home_mani robot_model:=wpb_home_mani
```

## 6. 常见错误

### 只有 RViz 模型，没有 `/scan`

说明 URDF 显示正常，但 Gazebo laser 插件没有正常接入。检查 `gazebo_plugins.xacro` 和 `frameName`。

### 有 `/scan`，AMCL 仍失败

检查：

```bash
rostopic echo -n 1 /scan
rosrun tf tf_echo base_footprint <laser_frame>
```

`/scan.header.frame_id` 必须能通过 TF 连到 `base_footprint`。

### `/cmd_vel` 有数据但不动

检查 diff drive 插件 wheel joint、话题 和 Gazebo 接触参数。官方展示模型通常没有适合仿真的轮子 joint。

### footprint 照抄其他机器人

会导致路径贴墙、穿障碍或窄通道误判。必须按新机器人尺寸重新设置。

## 7. 下一步阅读

模型说明看 [robot_models.md](robot_models.md)，Gazebo 插件看 [gazebo_simulation.md](gazebo_simulation.md)，导航参数看 [configuration_reference.md](configuration_reference.md)。
