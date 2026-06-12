<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# 参考资料

适合读者：想追溯模型来源、第三方算法来源和学习资料的用户。

## 1. ROS 与 Gazebo

- ROS Noetic: https://wiki.ros.org/noetic
- Gazebo classic: https://classic.gazebosim.org/
- navigation stack: https://wiki.ros.org/navigation
- move_base: https://wiki.ros.org/move_base
- map_server: https://wiki.ros.org/map_server
- amcl: https://wiki.ros.org/amcl
- gmapping: https://wiki.ros.org/gmapping
- hector_mapping: https://wiki.ros.org/hector_mapping

## 2. 机器人模型来源

WPB Home 相关来源：

- https://github.com/6-robot/wpb_home.git
- https://github.com/6-robot/wpr_simulation.git

本仓库保留官方 URDF/mesh，并在 `mr_description/urdf/wpb_home/simulation/` 中增加 Gazebo simulation-only 适配层。

## 3. 传统规划算法来源

`mr_traditional_planner` 中部分算法参考或改写自 PythonRobotics：

- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/DStar
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/DStarLite
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/ThetaStar
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/RRTStar
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/RRT
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/DynamicWindowApproach
- https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/CubicSpline

仓库内第三方说明：

```text
src/mr_traditional_planner/THIRD_PARTY_NOTICES.md
```

## 4. 覆盖路径规划学习资料

下面两篇覆盖路径规划学习解析由作者本人撰写：

- [【全覆盖路径规划】回溯螺旋算法 Backtracking Spiral Algorithm (BSA)：基于优先级状态机的底层逻辑深入解析](https://blog.csdn.net/weixin_66211313/article/details/159582434)
- [【全覆盖路径规划】螺旋生成树覆盖算法（Spiral-STC）：基于双层栅格与宏观拓扑的在线路径规划解析](https://blog.csdn.net/weixin_66211313/article/details/159733957)

说明：当前仓库接入的覆盖算法为 BCD 和 STC；上面两篇博客作为学习参考，不等同于当前仓库已有 launch 入口。

## 5. 下一步阅读

回到 [index.md](index.md) 选择专题文档。

---

<a id="english"></a>

## English

This page lists traceable references for ROS/Gazebo, robot models, planner implementations, and coverage-planning learning material.

The two coverage-planning blog posts linked in the Chinese section were written by the repository author. They are learning references for BSA and Spiral-STC. The implemented coverage planners in this repository are BCD and STC.

Third-party notices for planner code adapted from PythonRobotics are kept in `src/mr_traditional_planner/THIRD_PARTY_NOTICES.md`.
