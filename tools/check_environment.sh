#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '[PASS] %s\n' "$*"
}

warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf '[WARN] %s\n' "$*"
}

fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '[FAIL] %s\n' "$*"
}

check_file() {
  local path="$1"
  if [[ -e "${path}" ]]; then
    pass "file: ${path}"
  else
    fail "missing file: ${path}"
  fi
}

check_command() {
  local cmd="$1"
  if command -v "${cmd}" >/dev/null 2>&1; then
    pass "command: ${cmd} -> $(command -v "${cmd}")"
  else
    fail "missing command: ${cmd}"
  fi
}

check_ros_package() {
  local pkg="$1"
  if command -v rospack >/dev/null 2>&1 && rospack find "${pkg}" >/dev/null 2>&1; then
    pass "ros package: ${pkg}"
  else
    fail "missing ros package: ${pkg}"
  fi
}

check_python_module() {
  local module="$1"
  local required="${2:-required}"
  if python3 -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('${module}') else 1)" >/dev/null 2>&1; then
    pass "python module: ${module}"
  elif [[ "${required}" == "optional" ]]; then
    warn "optional python module missing: ${module}"
  else
    fail "missing python module: ${module}"
  fi
}

print_section() {
  printf '\n== %s ==\n' "$1"
}

load_ros_environment() {
  if [[ -e /opt/ros/noetic/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
  fi

  if [[ -e "${REPO_ROOT}/devel/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/devel/setup.bash"
  else
    export ROS_PACKAGE_PATH="${REPO_ROOT}/src:${ROS_PACKAGE_PATH:-}"
    warn "未找到 devel/setup.bash；已临时把 src 加入 ROS_PACKAGE_PATH。请运行 catkin_make。"
  fi
}

check_os() {
  print_section "System"
  if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    if [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == "20.04" ]]; then
      pass "Ubuntu 20.04: ${PRETTY_NAME}"
    else
      warn "当前系统不是 Ubuntu 20.04: ${PRETTY_NAME:-unknown}"
    fi
  else
    warn "无法读取 /etc/os-release"
  fi

  if [[ "${ROS_DISTRO:-}" == "noetic" ]]; then
    pass "ROS_DISTRO=noetic"
  elif [[ -z "${ROS_DISTRO:-}" ]]; then
    warn "ROS_DISTRO 未设置；请 source /opt/ros/noetic/setup.bash"
  else
    fail "ROS_DISTRO=${ROS_DISTRO}，期望 noetic"
  fi
}

check_commands() {
  print_section "Commands"
  local commands=(
    roslaunch
    rospack
    roscore
    rostopic
    rosnode
    rosparam
    catkin_make
    gazebo
    gzserver
    python3
  )

  local cmd
  for cmd in "${commands[@]}"; do
    check_command "${cmd}"
  done

  if command -v gazebo >/dev/null 2>&1; then
    gazebo --version 2>/dev/null || true
  fi
}

check_ros_packages() {
  print_section "ROS Packages"
  local packages=(
    xacro
    urdf
    robot_state_publisher
    joint_state_publisher
    joint_state_publisher_gui
    gazebo_ros
    gazebo_plugins
    map_server
    amcl
    move_base
    navfn
    costmap_2d
    nav_core
    base_local_planner
    dwa_local_planner
    pluginlib
    actionlib
    move_base_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    std_msgs
    std_srvs
    tf
    tf2_ros
    gmapping
    hector_mapping
    rviz
    rqt_tf_tree
    teleop_twist_keyboard
    mr_description
    mr_gazebo
    mr_maps
    mr_slam
    mr_navigation
    mr_traditional_planner
    mr_learning
    mr_msgs
  )

  local pkg
  for pkg in "${packages[@]}"; do
    check_ros_package "${pkg}"
  done
}

check_python() {
  print_section "Python"
  check_python_module numpy required
  check_python_module torch optional
}

check_repository_files() {
  print_section "Repository Files"
  local files=(
    "${REPO_ROOT}/src/mr_navigation/launch/navigation_sim.launch"
    "${REPO_ROOT}/src/mr_traditional_planner/launch/planner_sim.launch"
    "${REPO_ROOT}/src/mr_navigation/rviz/navigation.rviz"
    "${REPO_ROOT}/src/mr_maps/maps/turtlebot3_world.yaml"
    "${REPO_ROOT}/src/mr_maps/maps/turtlebot3_world.pgm"
    "${REPO_ROOT}/src/mr_maps/maps/maze_2.yaml"
    "${REPO_ROOT}/src/mr_maps/maps/maze_2.pgm"
    "${REPO_ROOT}/src/mr_gazebo/worlds/turtlebot3_world.world"
    "${REPO_ROOT}/src/mr_gazebo/worlds/maze/maze_2.world"
    "${REPO_ROOT}/src/mr_navigation/config/robot_models.yaml"
    "${REPO_ROOT}/src/mr_traditional_planner/planner_plugins.xml"
  )

  local file
  for file in "${files[@]}"; do
    check_file "${file}"
  done
}

check_plugins() {
  print_section "Planner Plugins"
  if command -v rospack >/dev/null 2>&1 && rospack plugins --attrib=plugin mr_traditional_planner >/tmp/mr_planner_plugins.txt 2>/tmp/mr_planner_plugins.err; then
    if [[ -s /tmp/mr_planner_plugins.txt ]]; then
      pass "pluginlib export for mr_traditional_planner"
      sed -n '1,20p' /tmp/mr_planner_plugins.txt
    else
      fail "rospack plugins returned no mr_traditional_planner entries"
    fi
  else
    fail "unable to query mr_traditional_planner plugin exports"
    sed -n '1,20p' /tmp/mr_planner_plugins.err 2>/dev/null || true
  fi
}

print_summary() {
  print_section "Summary"
  printf 'PASS: %d\nWARN: %d\nFAIL: %d\n' "${PASS_COUNT}" "${WARN_COUNT}" "${FAIL_COUNT}"

  if [[ "${FAIL_COUNT}" -gt 0 ]]; then
    printf '\n请先修复 FAIL 项，再运行 catkin_make 或 Quick Start。\n'
    exit 1
  fi

  printf '\n环境检查未发现阻塞项。\n'
}

main() {
  cd "${REPO_ROOT}"
  load_ros_environment
  check_os
  check_commands
  check_ros_packages
  check_python
  check_repository_files
  check_plugins
  print_summary
}

main "$@"
