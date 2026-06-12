#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SYSTEM_PACKAGES=(
  build-essential
  cmake
  git
  gnome-terminal
  python3
  python3-pip
  python3-numpy
  python3-rosdep
  python3-catkin-pkg
  python3-empy
)

ROS_PACKAGES=(
  ros-noetic-xacro
  ros-noetic-urdf
  ros-noetic-joint-state-publisher
  ros-noetic-joint-state-publisher-gui
  ros-noetic-robot-state-publisher
  ros-noetic-gazebo-ros
  ros-noetic-gazebo-plugins
  ros-noetic-gazebo-ros-control
  ros-noetic-map-server
  ros-noetic-amcl
  ros-noetic-move-base
  ros-noetic-navfn
  ros-noetic-costmap-2d
  ros-noetic-nav-core
  ros-noetic-base-local-planner
  ros-noetic-dwa-local-planner
  ros-noetic-pluginlib
  ros-noetic-actionlib
  ros-noetic-actionlib-msgs
  ros-noetic-gazebo-msgs
  ros-noetic-geometry-msgs
  ros-noetic-message-generation
  ros-noetic-message-runtime
  ros-noetic-move-base-msgs
  ros-noetic-nav-msgs
  ros-noetic-sensor-msgs
  ros-noetic-std-msgs
  ros-noetic-std-srvs
  ros-noetic-tf
  ros-noetic-tf2
  ros-noetic-tf2-ros
  ros-noetic-gmapping
  ros-noetic-hector-mapping
  ros-noetic-hector-slam
  ros-noetic-teleop-twist-keyboard
  ros-noetic-rviz
  ros-noetic-rqt-tf-tree
)

log() {
  printf '[install] %s\n' "$*"
}

die() {
  printf '[install][ERROR] %s\n' "$*" >&2
  exit 1
}

check_ubuntu() {
  if [[ ! -r /etc/os-release ]]; then
    die "无法读取 /etc/os-release，无法确认系统版本。"
  fi

  # shellcheck disable=SC1091
  source /etc/os-release

  if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "20.04" ]]; then
    if [[ "${ALLOW_UNSUPPORTED_OS:-0}" == "1" ]]; then
      log "当前系统为 ${PRETTY_NAME:-unknown}，不是 Ubuntu 20.04；ALLOW_UNSUPPORTED_OS=1，继续执行。"
    else
      die "当前系统为 ${PRETTY_NAME:-unknown}，本脚本目标环境是 Ubuntu 20.04。若确认要继续，请设置 ALLOW_UNSUPPORTED_OS=1。"
    fi
  else
    log "系统版本检查通过: ${PRETTY_NAME}。"
  fi
}

check_ros_hint() {
  if [[ -n "${ROS_DISTRO:-}" && "${ROS_DISTRO}" != "noetic" ]]; then
    die "当前 ROS_DISTRO=${ROS_DISTRO}，本仓库目标是 ROS Noetic。请切换环境后重试。"
  fi

  if [[ -e /opt/ros/noetic/setup.bash ]]; then
    log "检测到 /opt/ros/noetic/setup.bash。"
  else
    log "未检测到 /opt/ros/noetic/setup.bash；apt 将尝试安装所需 ros-noetic-* 包。"
  fi
}

check_root() {
  if [[ "${EUID}" -ne 0 ]]; then
    die "安装 apt 依赖需要 root 权限。请运行: sudo bash tools/install_dependencies.sh"
  fi
}

install_apt_packages() {
  log "更新 apt 索引。"
  apt-get update

  log "安装系统工具。"
  apt-get install -y "${SYSTEM_PACKAGES[@]}"

  log "安装 ROS / Gazebo 依赖。"
  apt-get install -y "${ROS_PACKAGES[@]}"
}

install_learning_python_optional() {
  if [[ "${INSTALL_LEARNING_PIP:-0}" != "1" ]]; then
    log "跳过 Learning Python pip 依赖。需要时可运行: sudo INSTALL_LEARNING_PIP=1 bash tools/install_dependencies.sh"
    return
  fi

  log "安装 Learning Python 依赖到 root 用户环境。若你使用 conda/venv，建议手动在对应环境中安装。"
  python3 -m pip install --upgrade pip
  python3 -m pip install -r "${SCRIPT_DIR}/requirements-learning.txt"
}

print_next_steps() {
  cat <<EOF

[install] 依赖安装流程结束。

下一步:
  cd ${REPO_ROOT}
  source /opt/ros/noetic/setup.bash
  catkin_make
  source devel/setup.bash
  bash tools/check_environment.sh

Quick Start:
  roslaunch mr_navigation navigation_sim.launch
EOF
}

main() {
  check_ubuntu
  check_ros_hint
  check_root
  install_apt_packages
  install_learning_python_optional
  print_next_steps
}

main "$@"
