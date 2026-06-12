<div align="right">

[英文版](../planner_framework.md)

</div>

# 规划框架

适合读者：想理解普通点到点规划、覆盖规划、调试路径和实际执行路径区别的用户。

本仓库中“规划”有两条不同链路：普通点到点导航链路和覆盖规划链路。不要把调试路径直接理解成机器人正在执行的路径。

## 1. 普通点到点导航

主链路：

```text
Global Planner
  -> Optional Smoother
  -> DWAPlannerROS
  -> /cmd_vel
```

在 move_base 中实际执行：

```text
move_base
  -> base_global_planner
  -> base_local_planner
  -> /cmd_vel
```

当前支持：

| 层级 | 当前实现 |
|---|---|
| Global Planner | `navfn`, A*, Dijkstra, D*, D* Lite, Theta*, RRT* |
| Optional Smoother | Cubic Spline |
| Local Planner | ROS `dwa_local_planner/DWAPlannerROS` |

## 2. 覆盖任务

覆盖规划不只是从 A 点到 B 点，而是尽量遍历自由空间。

当前覆盖链路：

```text
Coverage Planner
  -> Coverage Path
  -> move_base 动作 路径点
  -> DWAPlannerROS
  -> /cmd_vel
```

当前覆盖算法：

- BCD
- STC

RViz 的 `2D Nav Goal` 在覆盖模式中主要是触发信号，不是覆盖终点。

## 3. 实际执行路径与调试路径

| 话题 | 来源 | 是否参与实际控制 | 用途 |
|---|---|---|---|
| `/mr_traditional_planner/executed_global_path` | `GlobalPlannerAdapter` | ✅ | 显示自定义全局规划器实际返回给 move_base 的路径 |
| `/move_base/DWAPlannerROS/global_plan` | `DWAPlannerROS` | ✅ | DWA 内部跟踪的全局参考 |
| `/move_base/DWAPlannerROS/local_plan` | `DWAPlannerROS` | ✅ | DWA 当前局部轨迹 |
| `/mr_traditional_planner/debug_optimal_path` | 独立 C++/Python 算法节点 | 否，默认只显示 | 对比算法输出、检查路径质量 |
| `/mr_traditional_planner/coverage_path` | BCD/STC 覆盖节点 | 🟣 | 覆盖路径和 路径点 执行参考 |

判断机器人实际怎么走，应优先看：

```text
executed_global_path
  -> DWAPlannerROS/global_plan
  -> DWAPlannerROS/local_plan
  -> /cmd_vel
```

## 4. `algorithm` 和 `global_planner` 的区别

`algorithm` 用于 `mr_traditional_planner/planner.launch` 或 `planner_sim.launch` 的独立调试节点。

`global_planner` 用于 `mr_navigation/navigation.launch` 或 `navigation_sim.launch`，决定 move_base 实际使用哪个全局规划器。

在 `planner_sim.launch` 中，如果 `algorithm` 是普通全局规划算法且没有显式覆盖 `global_planner`，启动文件 会让 `global_planner` 默认跟随 `algorithm`。为了实验结果清晰，建议仍然显式写出：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  algorithm:=dstar_lite \
  impl:=cpp \
  global_planner:=dstar_lite \
  local_planner:=dwa
```

## 5. C++ 与 Python 调试节点

统一入口：

```bash
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=cpp
roslaunch mr_traditional_planner planner.launch algorithm:=astar impl:=py
```

C++ 入口：

- 节点：`planner_plugin_node`
- 插件注册：`src/mr_traditional_planner/planner_plugins.xml`
- 默认路径：`/mr_traditional_planner/debug_optimal_path`

Python 入口：

- 统一 relay：`python_planner_node.py`
- 具体算法脚本位于 `src/mr_traditional_planner/scripts/`
- 默认路径：`/mr_traditional_planner/debug_optimal_path`

覆盖算法默认路径：

```text
/mr_traditional_planner/coverage_path
```

## 6. planner_compatibility_validator

`mr_navigation/navigation.launch` 会启动兼容性校验节点，阻止明显错误组合。

示例：

| 错误组合 | 结果 |
|---|---|
| `global_planner:=cubic_spline` | 报错退出 |
| `local_planner:=stc` | 报错退出 |
| `local_planner:=bcd` | 报错退出 |
| `local_planner:=astar` | 报错退出 |
| `planning_mode:=coverage path_smoother:=cubic_spline` | 报错退出 |

## 7. 推荐调试顺序

1. 看 `/scan`、`/odom`、`/tf`、`/map` 是否正常。
2. 看 `/mr_traditional_planner/debug_optimal_path`，确认算法本身能生成路径。
3. 看 `/mr_traditional_planner/executed_global_path`，确认 move_base 实际拿到的路径。
4. 看 `/move_base/DWAPlannerROS/global_plan` 和 `local_plan`。
5. 看 `/cmd_vel` 和 Gazebo 中机器人是否运动。

## 8. 下一步阅读

普通全局算法看 [optimal_path_planners.md](optimal_path_planners.md)，覆盖算法看 [coverage_path_planning.md](coverage_path_planning.md)。
