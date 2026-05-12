# Benchmark result for a single planning run.
Header header

# Algorithm identifier, e.g. "astar", "bcd".
string algorithm

# Total path length in meters.
float64 path_length

# Wall-clock planning time in seconds.
float64 planning_time

# Number of waypoints in the published path.
int32 num_waypoints

# True if the planner found a feasible path.
bool success
