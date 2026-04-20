# mr_traditional_planner 调试笔记

这份笔记只描述当前工作区里已经落地并验证过的调试方式。

当前可切换算法：

- `astar`
- `dijkstra`
- `bcd`
- `stc`

当前可切换实现：

- `cpp`
- `py`

## 1. 调试前准备

先编译并加载工作区环境：

```bash
cd /home/lmy/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
catkin_make
source /home/lmy/mobile_robot_benchmark/devel/setup.bash
```

如果你开了多个终端，每个终端都要重新执行：

```bash
source /opt/ros/noetic/setup.bash
source /home/lmy/mobile_robot_benchmark/devel/setup.bash
```

## 2. 最短联调路径

最推荐直接使用总入口 [planner_sim.launch](/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/launch/planner_sim.launch)。  
它会一次性拉起：

- Gazebo 仿真世界
- 地图
- AMCL
- move_base
- 指定传统算法节点

示例：

```bash
roslaunch mr_traditional_planner planner_sim.launch algorithm:=astar impl:=cpp
```

切换成别的算法时，只改两个参数：

```bash
roslaunch mr_traditional_planner planner_sim.launch algorithm:=dijkstra impl:=py
roslaunch mr_traditional_planner planner_sim.launch algorithm:=bcd impl:=cpp
roslaunch mr_traditional_planner planner_sim.launch algorithm:=stc impl:=py
```

参数说明：

- `algorithm:=astar|dijkstra|bcd|stc`
- `impl:=cpp|py`

## 3. 不重启仿真时如何切算法

如果你不想每次都重新拉起 Gazebo，可以把环境和算法分开启动。

终端 1：只启动导航仿真环境

```bash
source /opt/ros/noetic/setup.bash
source /home/lmy/mobile_robot_benchmark/devel/setup.bash
roslaunch mr_navigation navigation_sim.launch
```

终端 2：只启动算法

```bash
source /opt/ros/noetic/setup.bash
source /home/lmy/mobile_robot_benchmark/devel/setup.bash
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp
```

切换算法时，直接停掉终端 2 的算法节点，再换参数重启即可：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=stc impl:=cpp
```

统一切换入口是 [planner.launch](/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/launch/planner.launch)。

## 4. 不同算法如何触发

当前四类算法的调试入口已经统一：

- 都订阅 `/move_base_simple/goal`
- 都通过 RViz 的 `2D Nav Goal` 触发

差别只在于：

- `A* / Dijkstra` 会把你点击的位置当作真实规划终点
- `BCD / STC` 只把这次点击当作“开始覆盖”的触发信号，真正的覆盖起点仍然取机器人当前位姿

如果你用的是当前仓库里的 RViz 配置 [navigation.rviz](/home/lmy/mobile_robot_benchmark/src/mr_navigation/rviz/navigation.rviz)，工具栏里显示的就是 `2D Nav Goal`。

### 4.1 A* / Dijkstra

这两类是最优路径算法，订阅：

- `/map`
- `/move_base_simple/goal`

调试方法：

1. 启动仿真和算法。
2. 打开 RViz。
3. 点击 `2D Nav Goal`。
4. 在地图上给一个目标点。
5. 查看 `/mr_traditional_planner/optimal_path`。

快速观察：

```bash
rostopic echo -n 1 /mr_traditional_planner/optimal_path
```

### 4.2 BCD / STC

这两类是覆盖算法，交互方式现在已经和 A* / Dijkstra 完全统一：

- 订阅 `/map`
- 订阅 `/move_base_simple/goal`
- 发布 `/mr_traditional_planner/coverage_path`
- 作为 `/move_base` 的 Action Client 顺序发送 waypoint

调试方法：

1. 启动仿真和算法。
2. 打开 RViz。
3. 点击 `2D Nav Goal`。
4. 在地图上任意点击一次，作为覆盖规划触发信号。
5. 算法会从机器人当前位姿开始生成覆盖路径，并接管 `/move_base`。

这里要注意两点：

- 你点击的坐标不会被当作覆盖终点使用，它只负责触发算法
- RViz 发给 `/move_base` 的原始单点导航 goal 会被覆盖算法取消，随后由算法自己的 waypoint 序列接管

快速观察：

```bash
rostopic echo -n 1 /mr_traditional_planner/coverage_path
```

## 5. 当前推荐的调试顺序

建议每次都按这个顺序排查，效率最高。

### 步骤 1：先确认核心节点都活着

```bash
rosnode list
```

重点看这些节点是否存在：

- `/gazebo`
- `/amcl`
- `/move_base`
- 当前算法节点

### 步骤 2：确认地图、雷达、里程计都在发

```bash
rostopic list
```

重点检查这些话题：

- `/map`
- `/scan`
- `/odom`
- `/cmd_vel`
- `/move_base_simple/goal`
- `/tf`
- `/tf_static`

再进一步看是否真的有数据：

```bash
rostopic echo -n 1 /map
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
```

### 步骤 3：确认 TF 链接通了

当前导航和算法默认依赖这些坐标系：

- `map`
- `odom`
- `base_footprint`

检查：

```bash
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo odom base_footprint
```

如果 `map -> base_footprint` 不通，A* / Dijkstra / BCD / STC 都会失败，因为它们都要从 TF 里拿机器人起点。

### 步骤 4：确认 move_base Action Server 正常

覆盖算法会等待 `/move_base`，所以要确认它已经起来：

```bash
rostopic list | rg move_base
```

至少应该能看到类似这些接口：

- `/move_base/goal`
- `/move_base/status`
- `/move_base/result`

### 步骤 5：再看算法自己的输出

最优路径算法看：

```bash
rostopic echo -n 1 /mr_traditional_planner/optimal_path
```

覆盖算法看：

```bash
rostopic echo -n 1 /mr_traditional_planner/coverage_path
```

如果要确认 RViz 点击确实发出来了，也可以直接看：

```bash
rostopic echo -n 1 /move_base_simple/goal
```

## 6. 当前最常见的故障与处理

### 6.1 `waitForService: Service [/gazebo/set_physics_properties] has not been advertised`

这通常表示 Gazebo 还没完全起来，或者 world/plugin 初始化卡住了。

先看：

```bash
rosnode list
rostopic list | rg gazebo
```

如果 `/gazebo` 都没起来，优先怀疑：

- Gazebo 进程还在初始化
- world 文件加载过慢
- 某个 Gazebo 插件报错

处理方式：

1. 先等几秒再看一次。
2. 看终端里 Gazebo 的报错。
3. 重新启动 `navigation_sim.launch` 或 `planner_sim.launch`。

### 6.2 `Timed out waiting for transform from base_footprint to map`

这说明 `map -> base_footprint` 没建立起来。

常见原因：

- `amcl` 没起来
- `/scan` 没有数据，AMCL 没法更新位姿
- `robot_state_publisher` 没起来
- Gazebo 机器人没有正确生成

优先检查：

```bash
rosnode list
rostopic echo -n 1 /scan
rosrun tf tf_echo map base_footprint
```

### 6.3 `No laser scan received`

这说明 `/scan` 没数据，后续的 AMCL 和 costmap 都会连锁失效。

优先检查：

```bash
rostopic echo -n 1 /scan
```

如果没有数据，继续看：

- Gazebo 里的机器人是否真的生成了
- 激光雷达传感器插件是否正常
- `spawn_navigation_world.launch` 是否正常执行

相关文件：

- [spawn_navigation_world.launch](/home/lmy/mobile_robot_benchmark/src/mr_gazebo/launch/spawn_navigation_world.launch)
- [navigation_sim.launch](/home/lmy/mobile_robot_benchmark/src/mr_navigation/launch/navigation_sim.launch)

### 6.4 点了 `2D Nav Goal` 但覆盖机器人不动

先区分是“没规划出路径”还是“路径有了但 move_base 没执行”。

先看覆盖路径有没有发出来：

```bash
rostopic echo -n 1 /mr_traditional_planner/coverage_path
```

如果没有路径，优先检查：

- `/map` 是否存在
- `map -> base_footprint` 是否存在
- 起点是否落在膨胀障碍物里
- 当前自由空间是否被膨胀后切得过碎，导致扫描段之间无法安全连接

如果路径已经有了，但机器人不动，优先检查：

```bash
rostopic echo -n 1 /move_base/status
```

再看 `move_base` 终端里是否出现：

- goal timeout
- costmap transform error
- local planner failure

### 6.5 覆盖路径明显变短、提前结束，或者不再穿墙

这通常不是新 bug，而是当前安全连接逻辑在生效。

现在的 `BCD / STC` 有两个强约束：

- 扫描线必须逐栅格检查，撞到障碍立即截断
- 不同扫描段或不同组件之间必须通过安全连接路径衔接，不能再做直线跳接

所以如果地图经过 `0.15m` 膨胀后通道过窄，可能出现：

- 路径长度明显变短
- 某些 Cell 被提前放弃
- STC 某些粗栅格节点被跳过

这时优先检查：

```bash
rostopic echo -n 1 /mr_traditional_planner/coverage_path
rostopic echo -n 1 /move_base/local_costmap/costmap
```

再结合终端日志看是否出现类似信息：

- `无法安全连接`
- `提前结束覆盖`
- `跳过该节点`

## 7. 当前推荐的最小验证命令

### 验证 launch 能否正确解析

```bash
roslaunch --files mr_traditional_planner planner.launch algorithm:=stc impl:=cpp
roslaunch --files mr_traditional_planner planner_sim.launch algorithm:=astar impl:=py
```

### 验证 Python 节点语法

```bash
python3 -m py_compile /home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/optimal/astar_planner_py.py
python3 -m py_compile /home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/optimal/dijkstra_planner_py.py
python3 -m py_compile /home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/coverage/bcd_planner_py.py
python3 -m py_compile /home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/coverage/stc_planner_py.py
```

### 验证 C++ 构建

```bash
cd /home/lmy/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
catkin_make --pkg mr_traditional_planner
```

## 8. 当前建议的实际测试组合

为了减少变量，建议先固定这些条件：

- 机器人：`burger`
- 地图：`turtlebot3_world`
- 实现：先测 `cpp`，再对比 `py`

建议测试顺序：

1. `astar cpp`
2. `dijkstra cpp`
3. `bcd cpp`
4. `stc cpp`
5. 再把四种算法全部切到 `py`

最优路径算法主要看：

- 是否能成功生成路径
- 路径是否穿障碍
- 切换不同目标点是否稳定

覆盖算法主要看：

- 是否能成功发布覆盖路径
- 路径是否严格贴合自由空间、不再穿墙
- 是否能持续向 `/move_base` 发送 waypoint
- 单点失败时是否会跳过并继续

## 9. 当前相关入口文件

- [planner_sim.launch](/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/launch/planner_sim.launch)
- [planner.launch](/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/launch/planner.launch)
- [navigation_sim.launch](/home/lmy/mobile_robot_benchmark/src/mr_navigation/launch/navigation_sim.launch)
- [navigation.launch](/home/lmy/mobile_robot_benchmark/src/mr_navigation/launch/navigation.launch)
- [README.md](/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/README.md)
