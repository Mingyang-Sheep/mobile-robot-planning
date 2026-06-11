# 导航路径话题与调试路径说明

本文档说明当前仓库中几类路径显示的来源、含义和调试价值，重点解释新增的：

```text
/mr_traditional_planner/executed_global_path
/mr_traditional_planner/debug_optimal_path
```

这两个话题的目标是把“机器人实际执行的全局路径”和“独立算法节点用于调试显示的路径”分开，避免 RViz 中看到一条蓝线后误以为它一定被 `move_base` 和 DWA 正在执行。

## 1. 路径话题总览

| 路径话题 | 来源 | 是否参与实际导航控制 | 主要用途 |
| --- | --- | --- | --- |
| `/mr_traditional_planner/executed_global_path` | `mr_traditional_planner/GlobalPlannerAdapter` | 支持。当 `move_base` 使用该 adapter 作为全局规划器时，这就是 adapter 返回给导航栈的路径副本 | 显示自定义全局规划器实际交给 `move_base` 的路径 |
| `/move_base/DWAPlannerROS/global_plan` | `DWAPlannerROS` | 是。DWA 跟踪的全局路径，通常是全局路径在局部规划器内部转换、裁剪后的版本 | 判断 DWA 实际在跟踪哪条全局参考路径 |
| `/move_base/DWAPlannerROS/local_plan` | `DWAPlannerROS` | 是。局部规划器当前选择的短时轨迹 | 判断机器人下一段运动趋势、是否贴墙、是否转弯失败 |
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 传统算法节点 | 否，默认只用于显示和对比 | 对比单个算法输出，检查算法实现、路径质量和穿墙问题 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 覆盖规划器 | 间接参与。覆盖 executor 会按 waypoint 交给 `move_base` 执行 | 调试覆盖规划顺序和覆盖路径完整性 |

## 2. Executed Global Path 的意义

`/mr_traditional_planner/executed_global_path` 表示自定义全局规划器 adapter 在 `makePlan()` 中真正返回给 `move_base` 的路径。

当使用如下命令时：

```bash
roslaunch mr_navigation navigation_sim.launch \
  global_planner:=astar \
  path_smoother:=none \
  local_planner:=dwa
```

`move_base` 会加载：

```text
base_global_planner: mr_traditional_planner/GlobalPlannerAdapter
```

此时 adapter 内部根据 `global_planner` 选择 A*、Dijkstra、D*、D* Lite、Theta* 或 RRT* 等算法，并把生成的路径返回给 `move_base`。同一条路径也会发布到：

```text
/mr_traditional_planner/executed_global_path
```

因此它的核心含义是：

```text
这是自定义全局规划器实际交给导航栈的全局路径。
```

如果开启：

```bash
path_smoother:=cubic_spline
```

则流程为：

```text
Global Planner -> Cubic Spline smoother -> collision check -> move_base
```

`executed_global_path` 显示的是最终返回给 `move_base` 的路径。如果 Cubic Spline 平滑后发生碰撞并回退原始路径，那么这里显示的也是回退后的实际执行路径。

## 3. Debug Optimal Path 的意义

`/mr_traditional_planner/debug_optimal_path` 来自独立传统算法节点，例如：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=dijkstra impl:=py
```

或：

```bash
roslaunch mr_traditional_planner planner_sim.launch algorithm:=theta_star impl:=cpp
```

这些节点用于单独验证 C++/Python 算法插件是否能接收 `/map` 和 RViz `2D Nav Goal`，并发布一条可视化路径。默认情况下，这条路径不会被 `move_base` 直接执行。

它的核心含义是：

```text
这是传统算法节点独立算出来的调试路径，不一定是机器人正在执行的路径。
```

这条路径适合用来回答这些问题：

```text
某个算法自身是否能找到路径？
C++ 和 Python 版本输出是否一致？
算法路径是否贴墙、穿墙、乱序或出现孤立线段？
Cubic Spline 平滑结果是否安全？
RRT* 是否经常找不到路径？
```

但它不能单独证明机器人正在按这条路径走。要确认实际执行路径，应同时查看：

```text
/mr_traditional_planner/executed_global_path
/move_base/DWAPlannerROS/global_plan
/move_base/DWAPlannerROS/local_plan
/cmd_vel
```

## 4. 和实际算法规划的关系

当前仓库里同一个算法可能有两种运行方式：

| 运行方式 | 示例参数 | 输出话题 | 是否被机器人执行 |
| --- | --- | --- | --- |
| move_base 全局规划器 | `global_planner:=dijkstra` | `/mr_traditional_planner/executed_global_path` | 是 |
| 独立调试节点 | `algorithm:=dijkstra impl:=py` | `/mr_traditional_planner/debug_optimal_path` | 否 |

在 `planner_sim.launch` 中，如果 `algorithm` 是全局规划算法，launch 默认会把它同步给 `global_planner`。例如：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  algorithm:=dijkstra \
  impl:=py
```

这会同时启动两条链路：

```text
move_base 链路:
global_planner:=dijkstra -> GlobalPlannerAdapter -> executed_global_path -> DWA -> /cmd_vel

调试显示链路:
algorithm:=dijkstra impl:=py -> python_planner_node.py -> debug_optimal_path
```

两条路径可能非常接近，也可能不同。常见原因包括：

```text
move_base 使用 costmap 和 inflation 代价，独立调试节点可能只读取 OccupancyGrid；
move_base 会使用机器人当前位姿作为起点，调试节点可能使用不同触发逻辑；
adapter 可能启用了 path_smoother，调试节点可能没有；
C++ 和 Python 实现细节、采样间隔或碰撞检测策略不同。
```

因此调试时不要只看 `debug_optimal_path`。它是算法参考线，不是执行承诺。

## 5. 对机器人的具体意义

对机器人来说，路径链路可以理解为：

```text
全局路径：告诉机器人从当前位置到目标点大体应该走哪条走廊。
局部路径：告诉机器人接下来一两秒如何避障、转弯、贴合全局方向。
速度命令：最终真正驱动底盘运动的 /cmd_vel。
```

`executed_global_path` 主要影响机器人“走哪边”。如果它贴墙，DWA 往往也会被迫贴墙；如果它穿过障碍，局部规划器会在附近频繁失败或触发 recovery。

`DWAPlannerROS/local_plan` 主要反映机器人“下一小段怎么走”。如果 local plan 在墙角抖动、很短或消失，通常说明局部窗口、footprint、inflation、速度采样或障碍代价存在问题。

`debug_optimal_path` 主要服务算法开发。它可以帮助判断问题出在算法本身，还是出在 move_base、costmap、DWA 或 launch 参数接入上。

## 6. 对整体调试的帮助

建议调试时按下面顺序观察：

```text
1. /mr_traditional_planner/debug_optimal_path
   先看独立算法是否能生成合理路径。

2. /mr_traditional_planner/executed_global_path
   再看 move_base 实际拿到的自定义全局路径是否合理。

3. /move_base/DWAPlannerROS/global_plan
   确认 DWA 内部正在跟踪的全局路径是否与 executed_global_path 一致。

4. /move_base/DWAPlannerROS/local_plan
   检查局部轨迹是否能在墙角、窄通道和障碍附近保持安全。

5. /cmd_vel
   最后确认局部规划器是否真正输出速度命令。
```

这样可以把问题拆开：

| 现象 | 优先怀疑 |
| --- | --- |
| `debug_optimal_path` 已穿墙 | 算法本身或调试节点碰撞检测有问题 |
| `debug_optimal_path` 正常，`executed_global_path` 异常 | `GlobalPlannerAdapter`、costmap 或 smoother 接入有问题 |
| `executed_global_path` 正常，`global_plan` 异常 | DWA 接收、裁剪、坐标变换或 TF 有问题 |
| `global_plan` 正常，`local_plan` 贴墙或消失 | DWA 参数、local costmap、footprint 或 inflation 有问题 |
| `local_plan` 正常但机器人不动 | `/cmd_vel`、底盘插件、topic remap 或 Gazebo 控制链路有问题 |

## 7. RViz 显示建议

当前 `mr_navigation/rviz/navigation.rviz` 已加入这些显示项：

```text
Executed Global Path -> /mr_traditional_planner/executed_global_path
Debug Optimal Path   -> /mr_traditional_planner/debug_optimal_path
DWA Global Plan      -> /move_base/DWAPlannerROS/global_plan
DWA Local Plan       -> /move_base/DWAPlannerROS/local_plan
```

建议同时打开：

```text
Global Costmap
Local Costmap
RobotModel
TF
LaserScan
```

这样可以直接观察路径是否远离膨胀区、局部轨迹是否穿过 footprint、以及 TF 是否和地图对齐。

## 8. 常用检查命令

查看当前实际使用的全局和局部规划器：

```bash
rosparam get /move_base/base_global_planner
rosparam get /move_base/global_planner
rosparam get /move_base/path_smoother
rosparam get /move_base/base_local_planner
```

查看路径是否发布：

```bash
rostopic echo -n 1 /mr_traditional_planner/executed_global_path
rostopic echo -n 1 /mr_traditional_planner/debug_optimal_path
rostopic echo -n 1 /move_base/DWAPlannerROS/global_plan
rostopic echo -n 1 /move_base/DWAPlannerROS/local_plan
```

查看最终控制是否输出：

```bash
rostopic echo -n 5 /cmd_vel
```

检查某个话题由谁发布：

```bash
rostopic info /mr_traditional_planner/executed_global_path
rostopic info /mr_traditional_planner/debug_optimal_path
```

`executed_global_path` 应只由 `move_base` 内的 `GlobalPlannerAdapter` 发布；`debug_optimal_path` 则通常由 `planner_plugin_node` 或 `python_planner_node` 发布。

## 9. 本次修改记录

本次路径话题相关修改的核心是：

```text
1. GlobalPlannerAdapter 发布实际执行全局路径：
   /mr_traditional_planner/executed_global_path

2. 独立 C++/Python 传统算法节点默认发布调试路径：
   /mr_traditional_planner/debug_optimal_path

3. RViz 中分别显示执行路径和调试路径，避免混淆。

4. GlobalPlannerAdapter 增加 costmap 软代价权重：
   costmap_cost_weight:=2.0
   用于让全局搜索更倾向远离墙体和膨胀区。

5. DWA 参数做了保守低速转弯优化：
   允许更低平移/角速度采样，增加角速度采样数，
   让墙角和窄通道处更容易生成合法 local_plan。
```

调试结论应以实际控制链路为准：

```text
executed_global_path -> DWAPlannerROS/global_plan -> DWAPlannerROS/local_plan -> /cmd_vel
```

`debug_optimal_path` 是非常有价值的算法对比参考，但不是机器人实际运动链路的最终证据。

## 10. 和本次导航小修的关系

本次导航小修并不是只改了 RViz 显示颜色，而是把路径来源拆清楚，并让全局路径和局部轨迹更适合真实控制。

全局层面：

```text
GlobalPlannerAdapter
-> 读取 move_base global costmap
-> 几何距离代价 + costmap 软代价
-> executed_global_path
```

`costmap_cost_weight` 会影响 `executed_global_path`。权重越大，路径越不愿意贴近高代价膨胀区；权重太大时，狭窄通道可能变得更难通过。因此它是调试全局路径贴墙问题时的重要旋钮。

局部层面：

```text
DWAPlannerROS
-> 跟踪 global_plan
-> 在 local costmap 中采样速度轨迹
-> local_plan
-> /cmd_vel
```

DWA 的 `min_vel_trans`、`min_vel_theta`、`vx_samples`、`vth_samples`、`sim_time`、`path_distance_bias`、`goal_distance_bias` 和 `occdist_scale` 会影响 `local_plan`。如果全局路径已经合理，但机器人在墙角仍然卡住，应重点看这些参数和 local costmap。

调试时可以这样判断：

| 需要判断的问题 | 优先观察的话题 |
| --- | --- |
| 全局算法是否真的进入 move_base | `/mr_traditional_planner/executed_global_path` |
| DWA 是否收到并跟踪这条路径 | `/move_base/DWAPlannerROS/global_plan` |
| DWA 是否能在墙角生成安全轨迹 | `/move_base/DWAPlannerROS/local_plan` |
| 独立算法实现是否有问题 | `/mr_traditional_planner/debug_optimal_path` |
| 机器人底盘是否真的运动 | `/cmd_vel`、Gazebo 中机器人位姿 |

所以，`executed_global_path` 是“导航主链路证据”，`debug_optimal_path` 是“算法开发证据”。两者同时显示，能快速判断问题是在算法实现、nav_core 接入、costmap 参数，还是 DWA 控制阶段。
