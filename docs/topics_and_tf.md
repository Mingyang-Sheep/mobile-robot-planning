<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# Topic 与 TF 参考

适合读者：想用 `rostopic`、RViz 和 TF 工具检查运行状态的用户。

## 1. 核心 topic

| Topic | 类型 | 来源 | 用途 |
|---|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | `map_server` 或 SLAM | 全局地图，供 AMCL、costmap、规划器使用 |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo laser 插件 | AMCL、SLAM、costmap 的主要传感器输入 |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff drive 插件 | 底盘局部连续运动估计 |
| `/cmd_vel` | `geometry_msgs/Twist` | move_base、teleop 或调试控制器发布 | 速度控制命令 |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher、Gazebo、AMCL | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | 静态坐标变换 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz `2D Nav Goal` | 普通导航目标或覆盖规划触发 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz `2D Pose Estimate` | 给 AMCL 初始位姿 |

## 2. 路径 topic

| Topic | 来源 | 是否表示实际执行路径 |
|---|---|---|
| `/mr_traditional_planner/executed_global_path` | `GlobalPlannerAdapter` | 是，自定义全局规划器返回给 `move_base` 的路径副本 |
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 调试规划器 | 否，主要用于算法对比显示 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 覆盖规划器 | 间接参与执行，覆盖节点会按 waypoint 发给 `/move_base` |
| `/move_base/DWAPlannerROS/global_plan` | ROS DWAPlannerROS | 是，DWA 内部跟踪的全局参考 |
| `/move_base/DWAPlannerROS/local_plan` | ROS DWAPlannerROS | 是，当前局部轨迹 |
| `/move_base/NavfnROS/plan` | ROS NavfnROS | 当 `global_planner:=navfn` 时是 Navfn 输出 |

## 3. 传感器 topic

| 模型 | 激光 | 相机 | 深度 | IMU |
|---|---|---|---|---|
| `burger` | `/scan` frame `base_scan` | 未接入相机 topic | 未接入 | `/imu` |
| `waffle` | `/scan` frame `base_scan` | URDF 有相机 frame，当前导航主要使用 `/scan` | 取决于 Gazebo 插件配置 | `/imu` |
| `waffle_pi` | `/scan` frame `base_scan` | `/camera/rgb/image_raw` 在模型配置中登记 | 未作为导航输入 | `/imu` |
| `wpb_home` | `/scan` frame `laser` | `/camera/rgb/image_raw` | `/camera/depth/image_raw` | `/imu` |
| `wpb_home_mani` | `/scan` frame `laser` | `/camera/rgb/image_raw` | `/camera/depth/image_raw` | `/imu` |

导航 costmap 当前主要使用 2D LaserScan。

## 4. 典型 TF

TurtleBot3 风格模型：

```text
map
  -> odom
    -> base_footprint
      -> base_link
        -> base_scan
        -> imu_link
        -> camera_link
```

WPB Home 风格模型：

```text
map
  -> odom
    -> base_footprint
      -> base_link
        -> body_link
        -> laser
        -> kinect2_rgb_optical_frame
        -> kinect2_ir_optical_frame
```

`map -> odom` 通常由 AMCL 或 SLAM 发布，`odom -> base_footprint` 通常由底盘插件发布，机器人内部固定关节由 `robot_state_publisher` 发布。

## 5. 常用检查命令

查看 topic：

```bash
rostopic list
```

查看关键数据：

```bash
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rostopic echo -n 1 /map
rostopic echo -n 1 /tf
```

查看 topic 发布者和订阅者：

```bash
rostopic info /cmd_vel
rostopic info /mr_traditional_planner/debug_optimal_path
```

查看 TF：

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo map base_footprint
rosrun rqt_tf_tree rqt_tf_tree
```

发布一个很小的速度命令：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 6. 下一步阅读

路径话题的具体含义继续看 [planner_framework.md](planner_framework.md)。启动失败时看 [troubleshooting.md](troubleshooting.md)。

---

<a id="english"></a>

## English

This page collects the important ROS topics and TF frames used by the workspace.

Typical checks include `/scan`, `/odom`, `/tf`, `/tf_static`, `/map`, `/cmd_vel`, `/move_base/*`, `/mr_traditional_planner/debug_optimal_path`, `/mr_traditional_planner/executed_global_path`, and `/mr_traditional_planner/coverage_path`.

The expected TF chain for navigation is usually `map -> odom -> base_footprint -> base_link -> sensor frames`. When navigation fails, verify topics first, then TF, then planner outputs.
