<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# 普通路径规划算法

适合读者：想在普通点到点导航中切换 A*、Dijkstra、D* Lite、Theta*、RRT* 等算法的用户。

当前普通全局规划算法可通过两种方式运行：

1. 作为 move_base 的实际全局规划器，输出 `/mr_traditional_planner/executed_global_path`。
2. 作为独立 C++/Python 调试节点，输出 `/mr_traditional_planner/debug_optimal_path`。

## 1. 算法总表

| 算法 | 参数 key | C++ 调试节点 | Python 调试节点 | 接入 move_base | Debug Path | 使用 costmap | 当前限制 |
|---|---|---|---|---|---|---|---|
| A* | `astar` | 是 | 是 | 是 | 是 | move_base adapter 使用 costmap | 栅格搜索，路径可能贴近膨胀区，受 `costmap_cost_weight` 影响 |
| Dijkstra | `dijkstra` | 是 | 是 | 是 | 是 | move_base adapter 使用 costmap | 栅格搜索，通常比 A* 更保守但开销更大 |
| D* | `dstar` | 是 | 是 | 是 | 是 | move_base adapter 使用 costmap | 当前 adapter 标注为静态 D* 风格，并非完整动态重规划实现 |
| D* Lite | `dstar_lite` | 是 | 是 | 是 | 是 | move_base adapter 使用 costmap | adapter 中有状态化 D* Lite 接入，但仍需在真实动态障碍场景中继续评估 |
| Theta* | `theta_star` | 是 | 是 | 是 | 是 | move_base adapter 使用 costmap | 使用视线检测，结果依赖碰撞检测和地图分辨率 |
| RRT* | `rrt_star` | 是 | 是 | 是 | 是 | move_base adapter 使用 costmap | 采样算法，结果和随机采样、迭代上限有关 |
| Cubic Spline | `cubic_spline` | 是 | 是 | 作为 smoother | 是 | adapter 平滑后做碰撞检查 | 不是全局规划器，不能写成 `global_planner` |
| DWA 调试 | `dwa` | 是 | 是 | 主链路使用 ROS DWAPlannerROS | 是 | 独立调试节点读取地图/里程计 | 独立 DWA 调试节点不等同于 move_base 内部 DWA |

## 2. move_base 实际执行

普通导航示例：

```bash
roslaunch mr_navigation navigation_sim.launch global_planner:=astar local_planner:=dwa
```

这时 move_base 使用：

```text
base_global_planner: mr_traditional_planner/GlobalPlannerAdapter
global_planner: astar
base_local_planner: dwa_local_planner/DWAPlannerROS
```

实际路径发布到：

```text
/mr_traditional_planner/executed_global_path
```

## 3. 独立 Debug Path

只启动调试节点：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=theta_star impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=theta_star impl:=py
```

需要已有 `/map` 和 `/move_base_simple/goal`。通常先启动：

```bash
roslaunch mr_navigation navigation_sim.launch
```

然后在另一个终端启动算法调试节点。

输出：

```text
/mr_traditional_planner/debug_optimal_path
```

## 4. planner_sim 一体化实验

如果希望同时看到实际执行路径和调试路径：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  algorithm:=dstar_lite \
  impl:=cpp \
  global_planner:=dstar_lite \
  local_planner:=dwa
```

此时：

- `global_planner:=dstar_lite` 决定 move_base 实际全局规划；
- `algorithm:=dstar_lite impl:=cpp` 决定额外 C++ 调试节点；
- C++ 调试节点发布 `/mr_traditional_planner/debug_optimal_path`；
- move_base adapter 发布 `/mr_traditional_planner/executed_global_path`。

## 5. maze_2 示例

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  model:=burger \
  robot_model:=burger \
  map_name:=maze_2 \
  world_name:=$(rospack find mr_gazebo)/worlds/maze/maze_2.world \
  x:=1.7 \
  y:=1.0 \
  yaw:=0.0 \
  algorithm:=dstar_lite \
  impl:=cpp \
  global_planner:=dstar_lite \
  local_planner:=dwa
```

RViz 中应同时观察：

- `Debug Optimal Path`：独立调试算法路径；
- `Executed Global Path`：move_base 实际全局路径；
- `DWA Global Plan`；
- `DWA Local Plan`。

## 6. 已知限制

- `D*` 当前应视为静态 costmap 上的 D* 风格搜索适配，不要描述成完整动态 D* 系统。
- `D* Lite` 已有 move_base adapter 和调试节点，但动态障碍持续重规划效果仍需实验数据支持。
- `RRT*` 采样结果可能随参数和随机性变化。
- Debug 节点多以 `/map` 和 RViz goal 为输入，和 move_base costmap 中的起点、代价、膨胀处理不一定完全相同。
- `Cubic Spline` 是 smoother 或调试节点，不是普通全局规划器。

## 7. 下一步阅读

路径含义看 [planner_framework.md](planner_framework.md)，topic 检查看 [topics_and_tf.md](topics_and_tf.md)。

---

<a id="english"></a>

## English

This page documents point-to-point planners and their current support matrix.

Current global planner keys include `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, and `rrt_star`. They can be used through the C++ `GlobalPlannerAdapter` for `move_base` and through C++/Python debug nodes for visualization. `cubic_spline` is a path smoother, and DWA is the local planning/control layer.

Known limitations are explicitly listed in the Chinese section. In particular, D* is a static-style adaptation here, while D* Lite still needs stronger dynamic-obstacle evaluation before being described as a complete dynamic replanning system.
