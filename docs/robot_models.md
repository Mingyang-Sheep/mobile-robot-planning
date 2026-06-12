<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# 机器人模型与切换

适合读者：想在同一套 Gazebo / SLAM / Navigation 链路中切换机器人模型的用户。

当前模型选择由 `robot_model` 和 `model` 两个参数共同完成：

- `robot_model` 决定加载哪个 URDF/Xacro。
- `model` 决定选择哪套导航参数文件。

大多数情况下二者保持一致。

## 1. 当前支持模型

| 模型 key | 来源 | 主要文件 | base frame | 激光 frame | 导航参数 key |
|---|---|---|---|---|---|
| `burger` | TurtleBot3 Burger 风格模型 | `src/mr_description/urdf/project/turtlebot3_burger.simulation.urdf.xacro` | `base_footprint` | `base_scan` | `burger` |
| `waffle` | TurtleBot3 Waffle 风格模型 | `src/mr_description/urdf/project/turtlebot3_waffle.simulation.urdf.xacro` | `base_footprint` | `base_scan` | `waffle` |
| `waffle_pi` | TurtleBot3 Waffle Pi 风格模型 | `src/mr_description/urdf/project/turtlebot3_waffle_pi.simulation.urdf.xacro` | `base_footprint` | `base_scan` | `waffle_pi` |
| `wpb_home` | 6-robot WPB Home 官方模型 + 本仓库 simulation 适配 | `src/mr_description/urdf/wpb_home/simulation/wpb_home_sim.urdf.xacro` | `base_footprint` | `laser` | `wpb_home` |
| `wpb_home_mani` | 6-robot WPB Home Mani 官方模型 + 本仓库 simulation 适配 | `src/mr_description/urdf/wpb_home/simulation/wpb_home_mani_sim.urdf.xacro` | `base_footprint` | `laser` | `wpb_home_mani` |

模型注册表位于：

```text
src/mr_navigation/config/robot_models.yaml
```

## 2. TurtleBot3 风格模型

### `burger`

用途：默认 Quick Start 模型，体积小，虚拟机上最稳。

主要特性：

- 差速底盘；
- 2D 激光 `/scan`，frame 为 `base_scan`；
- 里程计 `/odom`；
- 速度输入 `/cmd_vel`；
- IMU topic `/imu`；
- 导航 footprint 最小，适合初学者验证。

启动：

```bash
roslaunch mr_navigation navigation_sim.launch model:=burger robot_model:=burger
```

### `waffle`

用途：更大的 TurtleBot3 风格底盘。

主要特性：

- 差速底盘；
- 2D 激光 `/scan`；
- URDF 中含相机相关 frame；
- 使用独立 footprint 和 DWA 参数。

启动：

```bash
roslaunch mr_navigation navigation_sim.launch model:=waffle robot_model:=waffle
```

### `waffle_pi`

用途：带 Pi Camera 风格配置的 TurtleBot3 模型。

主要特性：

- 差速底盘；
- 2D 激光 `/scan`；
- `robot_models.yaml` 记录相机 topic `/camera/rgb/image_raw`；
- 导航仍主要依赖 `/scan`。

启动：

```bash
roslaunch mr_navigation navigation_sim.launch model:=waffle_pi robot_model:=waffle_pi
```

## 3. WPB Home

WPB Home 原始模型来自 6-robot 官方仓库。当前仓库保留官方 URDF 和 mesh，并在 `simulation/` 目录中增加仿真适配层。

官方原始文件：

```text
src/mr_description/urdf/wpb_home/wpb_home.urdf
src/mr_description/meshes/wpb_home/wpb_home.dae
src/mr_description/rviz/wpb_home/
src/mr_description/config/wpb_home/wpb_home.yaml
```

仿真适配文件：

```text
src/mr_description/urdf/wpb_home/simulation/wpb_home_model.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_sim.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_gazebo_plugins.xacro
```

主要特性：

- 官方模型无标准左右轮 joint，本仓库在 simulation 层增加隐藏轮 joint；
- 使用 `libgazebo_ros_diff_drive.so` 接入 `/cmd_vel` 和 `/odom`；
- 使用 `libgazebo_ros_laser.so` 发布 `/scan`，frame 为 `laser`；
- 使用 depth camera 插件发布相机/深度相关 topic；
- 使用 IMU 插件发布 `/imu`；
- 导航 footprint 和 costmap 参数独立于 TurtleBot3。

启动：

```bash
roslaunch mr_navigation navigation_sim.launch model:=wpb_home robot_model:=wpb_home
```

## 4. WPB Home Mani

`wpb_home_mani` 保留官方移动操作平台视觉模型，并增加 simulation-only 模型以便 Gazebo 能正常加载。

主要文件：

```text
src/mr_description/urdf/wpb_home/wpb_home_mani.urdf
src/mr_description/urdf/wpb_home/simulation/wpb_home_mani_model.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_mani_sim.urdf.xacro
```

当前状态：

- 可以作为移动底盘模型接入 Gazebo、SLAM 和 Navigation；
- 机械臂部分保持静态或通过 Gazebo joint state publisher 发布关节状态；
- 当前仓库没有实现机械臂运动控制、抓取任务或 MoveIt 控制链路；
- 导航 footprint 仍按移动底盘处理，不把机械臂工作空间作为独立规划对象。

启动：

```bash
roslaunch mr_navigation navigation_sim.launch model:=wpb_home_mani robot_model:=wpb_home_mani
```

## 5. Description-only 检查

只检查 WPB Home 模型显示：

```bash
roslaunch mr_description wpb_home_description.launch model:=wpb_home
roslaunch mr_description wpb_home_description.launch model:=wpb_home_mani
```

这类启动不会生成 `/scan`、`/odom` 或 `/cmd_vel`，只能说明 URDF、mesh 和 TF 发布基本正常。

## 6. 模型切换注意事项

切换机器人时至少同步检查：

- `robot_model` 是否有对应 Xacro；
- `model` 是否有对应 `costmap_common_params_<model>.yaml`；
- DWA 参数文件是否存在；
- 激光 frame 是否和 costmap 的 `sensor_frame` 一致；
- footprint 是否适合真实尺寸；
- 初始位姿是否落在地图自由空间。

## 7. 下一步阅读

Gazebo 插件看 [gazebo_simulation.md](gazebo_simulation.md)，移植新模型看 [import_robot_urdf_to_navigation.md](import_robot_urdf_to_navigation.md)。

---

<a id="english"></a>

## English

This page documents the robot model switch system. `robot_model` selects the URDF/Xacro model, while `model` selects navigation parameters such as footprint, costmap, and DWA settings.

Currently documented model keys are `burger`, `waffle`, `waffle_pi`, `wpb_home`, and `wpb_home_mani`. TurtleBot3-style models use `base_scan`; WPB Home models use `laser` as the laser frame. WPB Home and WPB Home Mani keep the official URDF/mesh assets and add a simulation-only adaptation layer for Gazebo diff-drive, laser, camera/depth, and IMU interfaces.

The Mani model is treated as a mobile base for this repository. Arm planning, MoveIt, and grasping are not implemented.
