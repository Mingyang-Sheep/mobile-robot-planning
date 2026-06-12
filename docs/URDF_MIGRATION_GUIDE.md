<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# URDF Migration 参考入口

这份文件保留为兼容入口。新的 URDF / 机器人移植说明已经按专题整理到下面几份文档：

| 需要的信息 | 请阅读 |
|---|---|
| 从外部 URDF 接入 Gazebo / SLAM / Navigation | [import_robot_urdf_to_navigation.md](import_robot_urdf_to_navigation.md) |
| 当前支持的机器人和模型切换 | [robot_models.md](robot_models.md) |
| Gazebo world、spawn 和插件 | [gazebo_simulation.md](gazebo_simulation.md) |
| costmap、footprint、DWA、AMCL 参数 | [configuration_reference.md](configuration_reference.md) |

新文档命名统一使用小写 snake_case。这个大写文件名继续保留，是为了兼容已有引用。

---

<a id="english"></a>

# URDF Migration Entry

This file is kept as a compatibility entry. The maintained URDF / robot migration guidance has been organized into the topic pages below:

| Information needed | Read |
|---|---|
| Import an external URDF into Gazebo / SLAM / Navigation | [import_robot_urdf_to_navigation.md](import_robot_urdf_to_navigation.md) |
| Current robot support and model switching | [robot_models.md](robot_models.md) |
| Gazebo worlds, spawning, and plugins | [gazebo_simulation.md](gazebo_simulation.md) |
| costmap, footprint, DWA, and AMCL parameters | [configuration_reference.md](configuration_reference.md) |

New documentation filenames use lowercase snake_case. This uppercase filename remains for compatibility with existing references.
