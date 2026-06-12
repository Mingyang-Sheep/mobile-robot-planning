<div align="right">

[Chinese](zh/optimal_path_planners.md)

</div>

# Point-to-Point Path Planners

This page documents point-to-point planners and their current support matrix.

Current global planner keys include `astar`, `dijkstra`, `dstar`, `dstar_lite`, `theta_star`, and `rrt_star`. They can be used through the C++ `GlobalPlannerAdapter` for `move_base` and through C++/Python debug nodes for visualization. `cubic_spline` is a path smoother, and DWA is the local planning/control layer.

Known limitations are explicitly listed in the Chinese section. In particular, D* is a static-style adaptation here, while D* Lite still needs stronger dynamic-obstacle evaluation before being described as a complete dynamic replanning system.
