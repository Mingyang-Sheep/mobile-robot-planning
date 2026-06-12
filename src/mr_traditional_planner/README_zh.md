<div align="right">

[英文版](README.md)

</div>

# mr_traditional_planner

`mr_traditional_planner` 是本仓库的传统规划包。它负责普通全局规划、覆盖路径规划、路径平滑/局部控制调试、`move_base` 全局规划适配器，以及 C++ / Python 双实现的算法对比入口。

仓库级完整文档请优先阅读：

| 主题 | 文档 |
|---|---|
| 规划链路与路径含义 | [../../docs/zh/planner_framework.md](../../docs/zh/planner_framework.md) |
| 普通全局路径规划 | [../../docs/zh/optimal_path_planners.md](../../docs/zh/optimal_path_planners.md) |
| 覆盖路径规划 | [../../docs/zh/coverage_path_planning.md](../../docs/zh/coverage_path_planning.md) |
| 启动参数 | [../../docs/zh/launch_reference.md](../../docs/zh/launch_reference.md) |
| 话题与 TF | [../../docs/zh/topics_and_tf.md](../../docs/zh/topics_and_tf.md) |

## 包职责

| 模块 | 当前作用 |
|---|---|
| `src/nav_core/global_planner_adapter.cpp` | 将自定义全局规划算法接入 `nav_core::BaseGlobalPlanner`，供 `move_base` 实际调用 |
| `include/mr_traditional_planner/optimal/` | A*、Dijkstra、D*、D* Lite、Theta*、RRT*、DWA、Cubic Spline 等 C++ 插件声明 |
| `src/optimal/` | 普通规划、DWA 调试、样条平滑的 C++ 节点入口 |
| `include/mr_traditional_planner/coverage/` 与 `src/coverage/` | BCD / STC 覆盖规划插件和节点入口 |
| `scripts/` | Python 调试规划器、兼容性校验和工具函数 |
| `planner_plugins.xml` | C++ pluginlib 注册表 |
| `launch/` | 单算法调试和一体化仿真启动入口 |

## 路径话题边界

| 话题 | 含义 |
|---|---|
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 调试节点输出，不代表机器人实际执行 |
| `/mr_traditional_planner/executed_global_path` | `GlobalPlannerAdapter` 返回给 `move_base` 的实际全局路径副本 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 生成的覆盖路径，随后通过 `/move_base` 动作接口逐个路径点执行 |
| `/move_base/DWAPlannerROS/global_plan` | DWA 内部跟踪的全局参考 |
| `/move_base/DWAPlannerROS/local_plan` | DWA 当前选择的短时局部轨迹 |

确认机器人实际怎么走时，应优先看 `executed_global_path`、DWA 全局/局部规划 和 `/cmd_vel`，不要只看 调试路径。

## 最小命令

单算法调试：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=py
```

完整仿真 + 调试路径：

```bash
roslaunch mr_traditional_planner planner_sim.launch algorithm:=dstar_lite impl:=cpp global_planner:=dstar_lite
```

覆盖规划：

```bash
roslaunch mr_navigation navigation_sim.launch planning_mode:=coverage coverage_planner:=stc
```

## 当前算法 key

```text
astar
dijkstra
dstar
dstar_lite
theta_star
rrt_star
dwa
cubic_spline
bcd
stc
```

`cubic_spline` 是平滑器 / 调试节点，不应写成独立 `global_planner`。主导航链路中的局部规划器使用 ROS `dwa_local_planner/DWAPlannerROS`，包内 DWA 节点主要用于调试和对比。

## 第三方来源

部分算法参考或改写自 PythonRobotics。许可证与来源说明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
