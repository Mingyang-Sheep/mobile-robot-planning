<div align="right">

[英文版](../coverage_path_planning.md)

</div>

# 覆盖路径规划

适合读者：想使用 BCD/STC 做自由空间覆盖，或想扩展覆盖规划算法的用户。

覆盖规划与普通点到点导航不同。普通导航追求从当前位置到目标点；覆盖规划追求遍历地图中的可通行区域。

## 作者本人覆盖规划博客

下面两篇博客由作者本人撰写，适合在阅读本仓库 BCD/STC 覆盖规划代码前先理解 BSA、Spiral-STC 等覆盖规划思路。当前仓库没有独立接入 BSA 或 Spiral-STC 启动文件，博客内容作为学习参考。

- [【全覆盖路径规划】回溯螺旋算法 Backtracking Spiral Algorithm (BSA)：基于优先级状态机的底层逻辑深入解析](https://blog.csdn.net/weixin_66211313/article/details/159582434)
- [【全覆盖路径规划】螺旋生成树覆盖算法（Spiral-STC）：基于双层栅格与宏观拓扑的在线路径规划解析](https://blog.csdn.net/weixin_66211313/article/details/159733957)

## 1. 当前实现

当前仓库真实接入的覆盖算法：

| 算法 | 参数 key | C++ | Python | 输出 话题 | 执行方式 |
|---|---|---|---|---|---|
| BCD | `bcd` | ✅ | ✅ | `/mr_traditional_planner/coverage_path` | 通过 `/move_base` 动作接口逐个路径点执行 |
| STC | `stc` | ✅ | ✅ | `/mr_traditional_planner/coverage_path` | 通过 `/move_base` 动作接口逐个路径点执行 |

当前仓库没有独立接入 BSA 或 Spiral-STC 启动文件。相关博客可以作为学习资料，不应写成现有启动入口。

## 2. 输入和输出

覆盖规划节点订阅：

```text
/map
/move_base_simple/goal
```

发布：

```text
/mr_traditional_planner/coverage_path
```

并作为 动作 client 连接：

```text
/move_base
```

RViz 的 `2D Nav Goal` 只触发覆盖任务，点击位置不作为覆盖终点。

## 3. 启动方式

通过 导航 覆盖模式：

```bash
roslaunch mr_navigation navigation_sim.launch \
  planning_mode:=coverage \
  coverage_planner:=stc \
  local_planner:=dwa
```

切换 BCD：

```bash
roslaunch mr_navigation navigation_sim.launch \
  planning_mode:=coverage \
  coverage_planner:=bcd \
  local_planner:=dwa
```

通过 planner_sim 同时做算法调试：

```bash
roslaunch mr_traditional_planner planner_sim.launch \
  algorithm:=stc \
  impl:=cpp \
  planning_mode:=coverage \
  coverage_planner:=stc
```

## 4. BCD 当前行为

BCD 当前实现要点：

- 从 `/map` 读取 OccupancyGrid；
- 以机器人当前位置作为覆盖起点；
- 按扫描行/自由段生成覆盖顺序；
- 在段之间使用安全连接路径；
- 发布覆盖路径；
- 将 路径点 依次发送给 `/move_base`。

常用调试参数包括 `sweep_spacing` 和 `goal_timeout`。

## 5. STC 当前行为

STC 当前实现要点：

- 从 `/map` 读取 OccupancyGrid；
- 构建粗栅格覆盖结构；
- 生成树状或螺旋式遍历顺序；
- 对路径做连接和压缩；
- 发布覆盖路径；
- 将 路径点 依次发送给 `/move_base`。

常用调试参数包括 `tree_spacing` 和 `goal_timeout`。

## 6. 成功标准

- `/map` 有数据；
- `map -> base_footprint` TF 可用；
- 点击 RViz `2D Nav Goal` 后 `/mr_traditional_planner/coverage_path` 有数据；
- `/move_base/status` 中可以看到 动作状态变化；
- `/cmd_vel` 有输出；
- Gazebo 中机器人按 路径点 移动。

## 7. 已知限制

- 覆盖路径不等于最终实际轨迹，DWA 会根据局部代价地图 执行每个路径点。
- 膨胀后的狭窄区域可能导致部分自由空间被跳过或连接失败。
- 当前没有生成覆盖率统计、重复覆盖率统计或实验图表。
- 当前文档不把博客中的 BSA/Spiral-STC 描述为仓库现有功能。

## 8. 学习参考

覆盖规划博客已经放在本文开头，更多来源见 [references.md](references.md)。

## 9. 下一步阅读

普通规划框架看 [planner_framework.md](planner_framework.md)，参数整理看 [configuration_reference.md](configuration_reference.md)。
