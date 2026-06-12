<div align="right">

[Chinese](README_zh.md)

</div>

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
