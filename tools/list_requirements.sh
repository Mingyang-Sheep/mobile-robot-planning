#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${REPO_ROOT}"
SRC_ROOT="${WORKSPACE_ROOT}/src"
GAZEBO_MODELS_DIR="${SRC_ROOT}/mr_gazebo/models"
GAZEBO_WORLDS_DIR="${SRC_ROOT}/mr_gazebo/worlds"

SYSTEM_PACKAGES=(
  build-essential
  cmake
  git
  python3
  python3-pip
  python3-rosdep
  python3-catkin-pkg
  python3-empy
)

ROS_PACKAGES=(
  ros-noetic-desktop-full
  ros-noetic-xacro
  ros-noetic-urdf
  ros-noetic-robot-state-publisher
  ros-noetic-gazebo-ros
  ros-noetic-gazebo-plugins
  ros-noetic-map-server
  ros-noetic-amcl
  ros-noetic-move-base
  ros-noetic-navfn
  ros-noetic-dwa-local-planner
  ros-noetic-actionlib
  ros-noetic-actionlib-msgs
  ros-noetic-geometry-msgs
  ros-noetic-move-base-msgs
  ros-noetic-nav-msgs
  ros-noetic-roscpp
  ros-noetic-rospy
  ros-noetic-tf
  ros-noetic-rviz
)

GAZEBO_PLUGINS=(
  libgazebo_ros_diff_drive.so
  libgazebo_ros_imu.so
  libgazebo_ros_laser.so
)

GAZEBO_BUILTIN_MODELS=(
  sun
  ground_plane
)

LOCAL_GAZEBO_MODELS=(
  turtlebot3_square
  turtlebot3_plaza
  turtlebot3_world
)

print_section() {
  local title="$1"
  printf '\n== %s ==\n' "${title}"
}

print_list() {
  local item
  for item in "$@"; do
    printf '  - %s\n' "${item}"
  done
}

command_status() {
  local cmd="$1"
  if command -v "${cmd}" >/dev/null 2>&1; then
    printf '  [OK] %s -> %s\n' "${cmd}" "$(command -v "${cmd}")"
  else
    printf '  [MISSING] %s\n' "${cmd}"
  fi
}

ros_package_status() {
  local pkg="$1"
  if command -v rospack >/dev/null 2>&1 && rospack find "${pkg}" >/dev/null 2>&1; then
    printf '  [OK] %s\n' "${pkg}"
  else
    printf '  [MISSING] %s\n' "${pkg}"
  fi
}

file_status() {
  local path="$1"
  if [[ -e "${path}" ]]; then
    printf '  [OK] %s\n' "${path}"
  else
    printf '  [MISSING] %s\n' "${path}"
  fi
}

plugin_status() {
  local plugin="$1"
  if [[ -e "/opt/ros/noetic/lib/${plugin}" ]]; then
    printf '  [OK] %s -> /opt/ros/noetic/lib/%s\n' "${plugin}" "${plugin}"
  elif ldconfig -p 2>/dev/null | grep -Fq "${plugin}"; then
    printf '  [OK] %s\n' "${plugin}"
  else
    printf '  [MISSING] %s\n' "${plugin}"
  fi
}

prepare_ros_home() {
  if [[ -z "${ROS_HOME:-}" ]]; then
    export ROS_HOME="/tmp/ros_home"
  fi

  mkdir -p "${ROS_HOME}" >/dev/null 2>&1 || true
}

print_header() {
  cat <<EOF
mobile-robot-benchmark 环境准备清单
仓库根目录: ${WORKSPACE_ROOT}

这个脚本只负责罗列和快速检查，不会自动安装任何依赖。
默认目标环境:
  - Ubuntu 20.04
  - ROS 1 Noetic
  - Gazebo 11
EOF
}

print_install_examples() {
  print_section "建议安装命令"
  cat <<'EOF'
  1. 初始化 rosdep
     sudo rosdep init
     rosdep update

  2. 安装系统工具
     sudo apt-get update
     sudo apt-get install -y build-essential cmake git python3 python3-pip python3-rosdep python3-catkin-pkg python3-empy

  3. 安装 ROS / Gazebo 依赖
     sudo apt-get install -y \
       ros-noetic-desktop-full \
       ros-noetic-xacro \
       ros-noetic-urdf \
       ros-noetic-robot-state-publisher \
       ros-noetic-gazebo-ros \
       ros-noetic-gazebo-plugins \
       ros-noetic-map-server \
       ros-noetic-amcl \
       ros-noetic-move-base \
       ros-noetic-navfn \
       ros-noetic-dwa-local-planner \
       ros-noetic-actionlib \
       ros-noetic-actionlib-msgs \
       ros-noetic-geometry-msgs \
       ros-noetic-move-base-msgs \
       ros-noetic-nav-msgs \
       ros-noetic-roscpp \
       ros-noetic-rospy \
       ros-noetic-tf \
       ros-noetic-rviz

  4. 构建工作区
     source /opt/ros/noetic/setup.bash
     cd /home/lmy/mobile_robot_benchmark
     catkin_make
     source devel/setup.bash
EOF
}

print_environment_exports() {
  print_section "运行前建议环境变量"
  cat <<EOF
  source /opt/ros/noetic/setup.bash
  source ${WORKSPACE_ROOT}/devel/setup.bash

  export GAZEBO_MODEL_PATH=${GAZEBO_MODELS_DIR}:\${GAZEBO_MODEL_PATH:-}

  说明:
    - 本仓库的 launch 文件已经在 mr_gazebo 层自动注入 GAZEBO_MODEL_PATH
    - 如果你手动运行 gazebo / gzserver，建议显式导出上面的 GAZEBO_MODEL_PATH
    - 当前仓库没有额外要求 GAZEBO_PLUGIN_PATH
EOF
}

print_repo_requirements() {
  print_section "系统层依赖"
  print_list "${SYSTEM_PACKAGES[@]}"

  print_section "ROS / Gazebo 层依赖"
  print_list "${ROS_PACKAGES[@]}"

  print_section "Gazebo 必需插件"
  print_list "${GAZEBO_PLUGINS[@]}"

  print_section "Gazebo 内置模型"
  print_list "${GAZEBO_BUILTIN_MODELS[@]}"

  print_section "仓库内自带的 Gazebo 模型"
  print_list "${LOCAL_GAZEBO_MODELS[@]}"

  print_section "当前仓库实际调用到的核心 ROS 包"
  print_list \
    gazebo_ros \
    gazebo_plugins \
    robot_state_publisher \
    xacro \
    urdf \
    map_server \
    amcl \
    move_base \
    navfn \
    dwa_local_planner \
    rviz \
    actionlib \
    actionlib_msgs \
    geometry_msgs \
    move_base_msgs \
    nav_msgs \
    roscpp \
    rospy \
    tf
}

run_quick_checks() {
  print_section "基础命令检查"
  command_status bash
  command_status python3
  command_status git
  command_status cmake
  command_status roscore
  command_status roslaunch
  command_status rospack
  command_status catkin_make
  command_status gazebo
  command_status gzserver
  command_status gzclient
  command_status rviz

  print_section "ROS 包检查"
  ros_package_status gazebo_ros
  ros_package_status gazebo_plugins
  ros_package_status robot_state_publisher
  ros_package_status xacro
  ros_package_status urdf
  ros_package_status map_server
  ros_package_status amcl
  ros_package_status move_base
  ros_package_status navfn
  ros_package_status dwa_local_planner
  ros_package_status rviz
  ros_package_status actionlib
  ros_package_status actionlib_msgs
  ros_package_status geometry_msgs
  ros_package_status move_base_msgs
  ros_package_status nav_msgs
  ros_package_status roscpp
  ros_package_status rospy
  ros_package_status tf

  print_section "Gazebo 插件库检查"
  plugin_status libgazebo_ros_diff_drive.so
  plugin_status libgazebo_ros_imu.so
  plugin_status libgazebo_ros_laser.so

  print_section "仓库关键目录检查"
  file_status "${SRC_ROOT}/mr_description"
  file_status "${SRC_ROOT}/mr_gazebo"
  file_status "${SRC_ROOT}/mr_maps"
  file_status "${SRC_ROOT}/mr_navigation"
  file_status "${SRC_ROOT}/mr_traditional_planner"
  file_status "${GAZEBO_MODELS_DIR}"
  file_status "${GAZEBO_WORLDS_DIR}"

  print_section "仓库关键文件检查"
  file_status "${SRC_ROOT}/mr_gazebo/launch/spawn_robot.launch"
  file_status "${SRC_ROOT}/mr_gazebo/launch/spawn_navigation_world.launch"
  file_status "${SRC_ROOT}/mr_navigation/launch/navigation.launch"
  file_status "${SRC_ROOT}/mr_navigation/launch/navigation_sim.launch"
  file_status "${SRC_ROOT}/mr_traditional_planner/launch/planner.launch"
  file_status "${SRC_ROOT}/mr_traditional_planner/launch/planner_sim.launch"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_burger.urdf.xacro"
  file_status "${SRC_ROOT}/mr_gazebo/worlds/turtlebot3_world.world"
  file_status "${SRC_ROOT}/mr_gazebo/worlds/stage_1.world"
  file_status "${SRC_ROOT}/mr_maps/maps/turtlebot3_world.yaml"
}

main() {
  prepare_ros_home
  print_header
  print_repo_requirements
  print_environment_exports
  print_install_examples
  run_quick_checks
}

main "$@"
