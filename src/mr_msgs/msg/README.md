<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>

# mr_msgs/msg

`mr_msgs/msg` 保存本仓库预留或已使用的 benchmark 级 ROS message 定义。当前具体消息为 `PlannerResult.msg`。

## 当前消息

| 文件 | 作用 |
|---|---|
| `PlannerResult.msg` | 记录单次规划运行的算法名、路径长度、规划耗时、waypoint 数量和是否成功 |

`PlannerResult.msg` 字段：

```text
Header header
string algorithm
float64 path_length
float64 planning_time
int32 num_waypoints
bool success
```

如果后续增加 service 或 action，请优先在仓库级文档中说明使用场景，再在本目录补充接口字段。

---

<a id="english"></a>

# mr_msgs/msg

`mr_msgs/msg` stores benchmark-level ROS message definitions reserved or used by this workspace. The current concrete message is `PlannerResult.msg`.

## Current Message

| File | Purpose |
|---|---|
| `PlannerResult.msg` | Records the algorithm identifier, path length, planning time, waypoint count, and success flag for one planning run |

`PlannerResult.msg` fields:

```text
Header header
string algorithm
float64 path_length
float64 planning_time
int32 num_waypoints
bool success
```

When future services or actions are added, document their use cases in the repository-level docs first, then keep the field-level interface notes here.
