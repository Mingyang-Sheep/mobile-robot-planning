<div align="right">

[Chinese](zh/coverage_path_planning.md)

</div>

# Coverage Path Planning

This page documents the current coverage-planning chain.

Integrated coverage planners:

- `bcd`
- `stc`

They subscribe to `/map` and `/move_base_simple/goal`, publish `/mr_traditional_planner/coverage_path`, and execute waypoints through the `/move_base` action interface. In coverage mode, RViz `2D Nav Goal` is mainly a trigger; the clicked point is not the final coverage endpoint.

The author-written BSA and Spiral-STC blog posts are linked near the top of this page as learning notes. They are not current launch entries in this repository.
