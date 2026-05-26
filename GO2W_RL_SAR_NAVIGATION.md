# Go2W + rl_sar 导航集成说明

本工作空间已把 `rl_sar` 的 Go2W 仿真控制链路整合进 `mobile-robot-planning`：

- 世界：`mr_gazebo/worlds/maze_2.world`
- 导航地图：`mr_maps/maps/maze_2_hector.yaml`
- 机器人：`go2w_description`
- 策略：`policy/go2w/robot_lab/policy.pt`
- 控制：`rl_sar/rl_sim` 订阅 `/cmd_vel`

## 依赖与编译

```bash
cd ~/mobile-robot-planning
source /opt/ros/noetic/setup.bash

# 首次使用需要下载 LibTorch/ONNX Runtime 到 library/inference_runtime。
# 如果只跑 policy.pt，至少需要 libtorch。
bash scripts/download_inference_runtime.sh libtorch

# 当前任务只跑 Gazebo 中的 rl_sim，不编译依赖厂商 SDK 的真实机器人接口。
catkin_make -DBUILD_RL_REAL_TARGETS=OFF
source devel/setup.bash
```

## 启动 Go2W 导航

```bash
roslaunch mr_navigation go2w_navigation_sim.launch
```

默认参数已经对应本课设目标：

- `world_name:=$(find mr_gazebo)/worlds/maze_2.world`
- `map_name:=maze_2_hector`
- `model:=go2w`
- `auto_start_rl:=true`
- `start_navigation_mode:=true`
- `use_cmd_vel_filter:=true`

启动后 `rl_sim` 会自动从 passive 进入 get-up，再切到 `go2w/robot_lab` 的 RL locomotion，并持续使用 `/cmd_vel` 作为策略命令输入。RViz 中继续用 `2D Nav Goal` 发送目标点。

## 常用调参入口

```bash
roslaunch mr_navigation go2w_navigation_sim.launch x:=1.7 y:=0.8 yaw:=1.5708
roslaunch mr_navigation go2w_navigation_sim.launch map_name:=maze_2_hector rviz:=false
roslaunch mr_navigation go2w_navigation_sim.launch auto_locomotion_delay:=7.0
roslaunch mr_navigation go2w_navigation_sim.launch use_cmd_vel_filter:=false
```

如果 AMCL 初始位姿和地图不重合，优先在 RViz 中使用 `2D Pose Estimate` 重新给定位姿，或者修改启动参数 `x`、`y`、`yaw` 让仿真出生点与 `maze_2_hector` 地图坐标对齐。

## 判断是否真的向目标运动

发送 RViz `2D Nav Goal` 后，分别观察规划器原始速度和送入 RL 的速度：

```bash
rostopic echo /move_base_cmd_vel
rostopic echo /cmd_vel
rostopic echo /move_base/status
rostopic echo /move_base/NavfnROS/plan
```

如果 `/move_base_cmd_vel` 有速度但 `/cmd_vel` 很小，调 `cmd_vel_filter.py` 的 `max_x`、`max_yaw`、加速度限制；如果 `/move_base_cmd_vel` 本身接近 0，优先检查 AMCL、目标点和 local costmap。
