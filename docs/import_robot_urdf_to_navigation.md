# 外部机器人 URDF 移植到 Gazebo / SLAM / Navigation 的完整流程

## 1. 移植目标

外部机器人移植不是只让模型在 RViz 中显示。一个完整可测试的机器人需要同时完成：

```text
URDF 模型
TF
Gazebo 运动插件
传感器 topic
SLAM
Navigation
RViz
```

本仓库推荐先保留官方原始模型，再单独增加仿真适配层。这样模型来源可追溯，Gazebo 插件和导航参数也不会污染官方文件。

## 2. 最小导航闭环

一个机器人要能跑 Navigation，至少需要：

```text
/map      # map_server 或 SLAM 输出的全局占据栅格
/scan     # 2D 激光，供 AMCL、SLAM、costmap 使用
/odom     # 里程计，提供 odom -> base 的连续运动估计
/tf       # map、odom、base、laser 等坐标变换
/cmd_vel  # move_base 或键盘控制输出到底盘的速度命令
```

RViz 能看到模型只说明 `robot_description` 和部分 TF 正常，不代表 `/scan`、`/odom`、`/cmd_vel` 已经闭环。

## 3. 目录结构规范

推荐目录：

```text
src/mr_description/
  urdf/
    <robot_name>/
      original/       # 可选，保存未经适配的官方文件
      simulation/     # 仿真专用 xacro 和 Gazebo 插件
  meshes/
    <robot_name>/
  rviz/
    <robot_name>/

src/mr_navigation/
  config/
    <robot_name> 或 *_<robot_name>.yaml
  launch/
```

当前仓库已经采用的 WPB Home 目录：

```text
src/mr_description/
  urdf/wpb_home/
  urdf/wpb_home/simulation/
  meshes/wpb_home/
  rviz/wpb_home/
  config/wpb_home/
```

## 4. 原始模型和仿真模型分离

原始文件只保存官方 URDF / Xacro / mesh / rviz：

```text
original/ 或当前官方文件目录
```

仿真层单独加入 Gazebo 插件：

```text
simulation/
  <robot>_sim.urdf.xacro
  <robot>_gazebo_plugins.xacro
```

禁止直接把 Gazebo 插件写进官方原始 URDF。官方文件只允许做必要包名和 mesh 路径适配。

## 5. 包名和 mesh 路径适配

常见替换：

```text
package://wpb_home_bringup/... -> package://mr_description/...
$(find wpb_home_bringup)/...   -> $(find mr_description)/...
```

路径报错时应该修正 package 路径或补齐官方 mesh，不要把 mesh 改成 box。

## 6. Gazebo 插件补齐

完整仿真通常需要：

```text
gazebo_ros_diff_drive 或同类底盘插件 -> /cmd_vel, /odom
ray sensor + gazebo_ros_laser          -> /scan
camera/depth camera 插件               -> /camera/rgb/image_raw, /camera/depth/image_raw
imu 插件                               -> /imu
joint state publisher 插件             -> 非固定关节 /joint_states
```

如果官方 URDF 没有左右轮 joint，不能把它说成官方模型已有。应在 `simulation/` 层补仿真专用隐藏轮 joint，并在文档中说明这是为了接入标准 Gazebo diff drive。

## 7. Frame 统一

导航链路必须统一：

```text
map
odom
base_footprint
base_link
laser
camera frame
```

costmap 中的 `sensor_frame` 要和激光插件 `frameName` 一致。AMCL、SLAM、move_base 的 `base_frame_id` / `robot_base_frame` 要和 TF 树一致。

## 8. 导航参数适配

每个机器人至少要独立检查：

```text
costmap_common_params
global_costmap_params
local_costmap_params
dwa_local_planner_params
amcl_params
move_base_params
```

重点不要照抄其他机器人：

```text
footprint / robot_radius
laser frame
scan topic
最大线速度和角速度
加速度限制
局部 costmap 窗口大小
```

本仓库当前 TurtleBot3 使用通用 `global_costmap_params.yaml`、`local_costmap_params.yaml`、`move_base_params.yaml`，并为每个模型提供 `costmap_common_params_<model>.yaml` 和 `dwa_local_planner_params_<model>.yaml`。WPB Home 在此基础上增加了专用覆盖文件。

## 9. SLAM 接入

gmapping / hector / slam_toolbox 等建图至少依赖：

```text
/scan
/odom
/tf
```

检查：

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rosrun tf tf_echo odom base_footprint
```

## 10. Navigation 接入

AMCL + move_base 至少依赖：

```text
/map
/scan
/odom
/tf
/cmd_vel
```

完成后应能通过统一入口切换：

```bash
roslaunch mr_navigation navigation_sim.launch model:=wpb_home robot_model:=wpb_home
```

## 11. 检查命令

```bash
rostopic list | grep -E "/map|/scan|/odom|/cmd_vel|/tf"
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rosrun tf tf_echo odom base_footprint
rosrun rqt_tf_tree rqt_tf_tree
rosparam get /robot_description | head
```

模型和 URDF 检查：

```bash
rosrun xacro xacro $(rospack find mr_description)/urdf/<robot>/simulation/<robot>_sim.urdf.xacro > /tmp/<robot>_sim.urdf
check_urdf /tmp/<robot>_sim.urdf
```

## 12. 常见问题

```text
RViz 能显示但 Gazebo 不能动:
  只有 robot_description，没有底盘 Gazebo 插件或插件没订阅 /cmd_vel。

有模型但没有 /scan:
  URDF 没有 ray sensor，或者 gazebo_ros_laser 的 topicName/frameName 配错。

有 /scan 但 move_base 不动:
  move_base 没收到目标、costmap 无法变换 laser frame，或局部规划器参数不适合底盘。

cmd_vel 发了但机器人不走:
  底盘插件 joint 名称错误、轮子没有接触地面、摩擦过低、插件库未加载。

TF extrapolation error:
  use_sim_time、/clock、TF 发布频率或 transform_tolerance 不一致。

base_link / base_footprint 不一致:
  AMCL、costmap、底盘插件的 base frame 参数没有统一。

laser frame 和 costmap 配置不一致:
  costmap 的 sensor_frame 必须匹配激光消息 frame_id。

mesh 路径找不到:
  package:// 路径没有替换到当前包，或官方 mesh 没有完整拷贝。

直接复制 URDF 但没有 Gazebo 插件:
  只能显示模型，不能产生 /scan、/odom，也不能响应 /cmd_vel。
```

## 13. WPB Home 本次移植记录

官方来源：

```text
https://github.com/6-robot/wpb_home.git
https://github.com/6-robot/wpr_simulation.git
```

原始模型文件：

```text
src/mr_description/urdf/wpb_home/wpb_home.urdf
src/mr_description/urdf/wpb_home/wpb_home_mani.urdf
src/mr_description/meshes/wpb_home/
src/mr_description/rviz/wpb_home/
src/mr_description/config/wpb_home/wpb_home.yaml
```

仿真适配文件：

```text
src/mr_description/urdf/wpb_home/simulation/wpb_home_sim.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_mani_sim.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_mani_model.urdf.xacro
src/mr_description/urdf/wpb_home/simulation/wpb_home_gazebo_plugins.xacro
```

WPB Home 官方 bringup URDF 和 WPR 官方仿真模型都没有左右轮 joint。WPR 官方仿真使用自定义 `libwpr_plugin.so` 直接控制底盘。本仓库为了避免引入外部自定义插件，在 simulation 层增加隐藏左右轮 joint，并使用 `libgazebo_ros_diff_drive.so` 接入标准 `/cmd_vel`、`/odom`。

`wpb_home_mani` 的官方 bringup URDF 里机械臂和 Kinect 含有无惯性的非固定 link，Gazebo 会在 URDF 转 SDF 时丢弃这些 link。当前仓库保留原始 URDF 不动，另外增加 `wpb_home_mani_model.urdf.xacro` 作为 simulation-only 模型，参考 WPR 官方仿真模型补齐机械臂 inertial/collision，并加入不可见低摩擦 caster 支撑，避免标准 diff-drive 物理轮仿真时底盘俯仰漂移。

新增导航参数：

```text
src/mr_navigation/config/costmap_common_params_wpb_home.yaml
src/mr_navigation/config/dwa_local_planner_params_wpb_home.yaml
src/mr_navigation/config/global_costmap_params_wpb_home.yaml
src/mr_navigation/config/local_costmap_params_wpb_home.yaml
src/mr_navigation/config/move_base_params_wpb_home.yaml
src/mr_navigation/config/amcl_params_wpb_home.yaml
src/mr_navigation/config/costmap_common_params_wpb_home_mani.yaml
src/mr_navigation/config/dwa_local_planner_params_wpb_home_mani.yaml
src/mr_navigation/config/global_costmap_params_wpb_home_mani.yaml
src/mr_navigation/config/local_costmap_params_wpb_home_mani.yaml
src/mr_navigation/config/move_base_params_wpb_home_mani.yaml
src/mr_navigation/config/amcl_params_wpb_home_mani.yaml
src/mr_navigation/config/robot_models.yaml
```

新增或接入的启动入口：

```text
roslaunch mr_description wpb_home_description.launch model:=wpb_home
roslaunch mr_navigation simulation.launch model:=wpb_home robot_model:=wpb_home
roslaunch mr_navigation slam_sim.launch model:=wpb_home robot_model:=wpb_home
roslaunch mr_navigation navigation_sim.launch model:=wpb_home robot_model:=wpb_home
roslaunch mr_navigation navigation_sim.launch robot_model:=wpb_home_mani
```

`wpb_home_mani` 使用独立导航参数，导航 footprint 仍然只按底盘处理，机械臂保持静止并通过 Gazebo joint state publisher 发布 TF。
