#!/usr/bin/env python3
"""Validate planner switching truthfulness without starting Gazebo.

The check uses roslaunch --dump-params so it can run quickly in CI or before a
manual RViz run. It verifies that algorithm names route to the expected
move_base role and that the adapter contains the failure-path safeguards needed
to avoid stale RViz paths.
"""

import os
import subprocess
import sys


GLOBAL_ALGORITHMS = {
    "astar": "grid_astar_costmap",
    "dijkstra": "grid_dijkstra_costmap",
    "dstar": "static_dstar_like_not_fully_dynamic",
    "dstar_lite": "stateful_dstar_lite_incremental_costmap",
    "theta_star": "theta_star_grid_line_of_sight_costmap",
    "rrt_star": "rrt_star_sampling_costmap",
}

NON_GLOBAL_ALGORITHMS = {
    "cubic_spline": "path_smoother_debug_only",
    "dwa": "local_planner_debug_only",
    "bcd": "coverage_planner_debug_only",
    "stc": "coverage_planner_debug_only",
}


def repo_root():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def dump_params(algorithm):
    cmd = [
        "roslaunch",
        "--dump-params",
        "mr_traditional_planner",
        "planner_sim.launch",
        "algorithm:=%s" % algorithm,
        "impl:=cpp",
        "use_rviz:=false",
        "gui:=false",
        "headless:=true",
    ]
    return subprocess.check_output(cmd, cwd=repo_root(), text=True)


def parse_params(text):
    params = {}
    for line in text.splitlines():
        if not line.startswith("/") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        params[key.strip()] = value.strip()
    return params


def expect(condition, failures, message):
    if not condition:
        failures.append(message)


def validate_algorithm(algorithm, params):
    failures = []
    base_global = params.get("/move_base/base_global_planner")
    global_planner = params.get("/move_base/global_planner")
    base_local = params.get("/move_base/base_local_planner")
    debug_algorithm = params.get("/planner_plugin_node/algorithm")
    debug_path_topic = params.get("/planner_plugin_node/path_topic")

    expect(
        base_local == "dwa_local_planner/DWAPlannerROS",
        failures,
        "base_local_planner is not DWAPlannerROS",
    )

    if algorithm in GLOBAL_ALGORITHMS:
        expect(
            base_global == "mr_traditional_planner/GlobalPlannerAdapter",
            failures,
            "global algorithm did not select GlobalPlannerAdapter",
        )
        expect(global_planner == algorithm, failures, "global_planner does not match algorithm")
        expect(debug_algorithm == algorithm, failures, "debug planner algorithm mismatch")
        expect(
            debug_path_topic == "/mr_traditional_planner/debug_optimal_path",
            failures,
            "global debug planner should publish debug_optimal_path",
        )
    else:
        expect(
            base_global == "mr_traditional_planner/GlobalPlannerAdapter",
            failures,
            "non-global debug run should still use adapter for default navigation",
        )
        expect(
            global_planner == "astar",
            failures,
            "non-global algorithm should not masquerade as move_base global planner",
        )
        if algorithm == "dwa":
            expect(debug_algorithm is None, failures, "DWA should use move_base local_plan by default")
            expect(debug_path_topic is None, failures, "DWA should not publish ordinary debug path by default")
        else:
            expect(debug_algorithm == algorithm, failures, "debug planner algorithm mismatch")
            expected_topic = (
                "/mr_traditional_planner/debug_coverage_path"
                if algorithm in {"bcd", "stc"}
                else "/mr_traditional_planner/debug_optimal_path"
            )
            expect(debug_path_topic == expected_topic, failures, "debug path topic mismatch")

    return failures


def static_source_checks():
    failures = []
    root = repo_root()
    adapter_path = os.path.join(
        root, "src", "mr_traditional_planner", "src", "nav_core", "global_planner_adapter.cpp"
    )
    plugin_node_path = os.path.join(
        root, "src", "mr_traditional_planner", "src", "planner_plugin_node.cpp"
    )
    python_node_path = os.path.join(
        root, "src", "mr_traditional_planner", "scripts", "python_planner_node.py"
    )

    with open(adapter_path, "r", encoding="utf-8") as handle:
        adapter = handle.read()
    with open(plugin_node_path, "r", encoding="utf-8") as handle:
        plugin_node = handle.read()
    with open(python_node_path, "r", encoding="utf-8") as handle:
        python_node = handle.read()

    required_adapter_tokens = [
        "publishEmptyVisualPath",
        '" success="',
        '(success ? "true" : "false")',
        "fallback=disabled",
        "request_id",
        "active algorithm:",
        "static_dstar_like_not_fully_dynamic",
        "stateful_dstar_lite_incremental_costmap",
    ]
    for token in required_adapter_tokens:
        expect(token in adapter, failures, "adapter missing token: %s" % token)

    expect(
        '"/mr_traditional_planner/executed_global_path"' in adapter,
        failures,
        "adapter does not own executed_global_path",
    )
    expect(
        "/mr_traditional_planner/executed_global_path" not in plugin_node,
        failures,
        "debug C++ entry should not publish executed_global_path",
    )
    expect(
        "/mr_traditional_planner/executed_global_path" not in python_node,
        failures,
        "debug Python entry should not publish executed_global_path",
    )
    expect("cleared debug path topic" in plugin_node, failures, "C++ debug startup clear missing")
    expect("Cleared debug path topic" in python_node, failures, "Python debug startup clear missing")
    expect("[DebugPlanner]" in plugin_node, failures, "C++ debug path publication log missing")
    expect("[DebugPlanner]" in python_node, failures, "Python debug path publication log missing")
    expect("goalCallback" in plugin_node, failures, "C++ debug goal clear missing")
    expect("goal_callback" in python_node, failures, "Python debug goal clear missing")
    return failures


def main():
    rows = []
    failed = False
    algorithms = list(GLOBAL_ALGORITHMS) + list(NON_GLOBAL_ALGORITHMS)

    for algorithm in algorithms:
        params = parse_params(dump_params(algorithm))
        failures = validate_algorithm(algorithm, params)
        role = "global" if algorithm in GLOBAL_ALGORITHMS else "debug-only"
        implementation = GLOBAL_ALGORITHMS.get(algorithm, NON_GLOBAL_ALGORITHMS.get(algorithm))
        status = "PASS" if not failures else "FAIL: " + "; ".join(failures)
        if failures:
            failed = True
        rows.append((algorithm, role, implementation, params.get("/move_base/global_planner"), status))

    static_failures = static_source_checks()
    if static_failures:
        failed = True

    print("| Algorithm | Role | Real implementation | move_base global | Status |")
    print("| --- | --- | --- | --- | --- |")
    for row in rows:
        print("| %s | %s | %s | %s | %s |" % row)

    print("\nStatic safeguards:")
    if static_failures:
        for failure in static_failures:
            print("- FAIL: %s" % failure)
    else:
        print("- PASS: failure clears executed path, request_id logging exists, fallback is disabled")
        print("- PASS: debug entry points clear debug path on startup/new goal and never publish executed_global_path")

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
