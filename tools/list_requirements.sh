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
  ros-noetic-roscpp
  ros-noetic-rospy
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

PYTHON_MODULES=(
  numpy
  torch
)

GAZEBO_PLUGINS=(
  libgazebo_ros_diff_drive.so
  libgazebo_ros_imu.so
  libgazebo_ros_imu_sensor.so
  libgazebo_ros_joint_state_publisher.so
  libgazebo_ros_laser.so
  libgazebo_ros_depth_camera.so
  libgazebo_ros_openni_kinect.so
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

python_module_status() {
  local module="$1"
  if python3 -c "import importlib.util, sys; sys.exit(0 if importlib.util.find_spec('${module}') else 1)" >/dev/null 2>&1; then
    printf '  [OK] %s\n' "${module}"
  else
    printf '  [MISSING] %s\n' "${module}"
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
     sudo apt-get install -y build-essential cmake git gnome-terminal python3 python3-pip python3-numpy python3-rosdep python3-catkin-pkg python3-empy

  3. 安装 Python 训练依赖
     python3 -m pip install --user -r tools/requirements-learning.txt

  4. 安装 ROS / Gazebo 依赖
     sudo apt-get install -y \
       ros-noetic-xacro \
       ros-noetic-urdf \
       ros-noetic-joint-state-publisher \
       ros-noetic-joint-state-publisher-gui \
       ros-noetic-robot-state-publisher \
       ros-noetic-gazebo-ros \
       ros-noetic-gazebo-plugins \
       ros-noetic-gazebo-ros-control \
       ros-noetic-map-server \
       ros-noetic-amcl \
       ros-noetic-move-base \
       ros-noetic-navfn \
       ros-noetic-costmap-2d \
       ros-noetic-nav-core \
       ros-noetic-base-local-planner \
       ros-noetic-dwa-local-planner \
       ros-noetic-pluginlib \
       ros-noetic-actionlib \
       ros-noetic-actionlib-msgs \
       ros-noetic-gazebo-msgs \
       ros-noetic-geometry-msgs \
       ros-noetic-message-generation \
       ros-noetic-message-runtime \
       ros-noetic-move-base-msgs \
       ros-noetic-nav-msgs \
       ros-noetic-roscpp \
       ros-noetic-rospy \
       ros-noetic-sensor-msgs \
       ros-noetic-std-msgs \
       ros-noetic-std-srvs \
       ros-noetic-tf \
       ros-noetic-tf2 \
       ros-noetic-tf2-ros \
       ros-noetic-gmapping \
       ros-noetic-hector-mapping \
       ros-noetic-hector-slam \
       ros-noetic-teleop-twist-keyboard \
       ros-noetic-rviz \
       ros-noetic-rqt-tf-tree

  5. 用 rosdep 复核 package.xml 依赖
     # 将 <workspace_root> 替换为当前仓库根目录
     source /opt/ros/noetic/setup.bash
     cd <workspace_root>
     rosdep install --from-paths src --ignore-src -r -y

  6. 构建工作区
     source /opt/ros/noetic/setup.bash
     cd <workspace_root>
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

  print_section "Python 模块依赖"
  print_list "${PYTHON_MODULES[@]}"

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
    joint_state_publisher \
    joint_state_publisher_gui \
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
    gazebo_msgs \
    geometry_msgs \
    message_generation \
    message_runtime \
    move_base_msgs \
    nav_msgs \
    pluginlib \
    roscpp \
    rospy \
    sensor_msgs \
    std_msgs \
    std_srvs \
    gmapping \
    hector_mapping \
    teleop_twist_keyboard \
    tf
}

run_quick_checks() {
  print_section "基础命令检查"
  command_status bash
  command_status python3
  command_status pip3
  command_status git
  command_status gnome-terminal
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
  ros_package_status joint_state_publisher
  ros_package_status joint_state_publisher_gui
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
  ros_package_status gazebo_msgs
  ros_package_status geometry_msgs
  ros_package_status message_generation
  ros_package_status message_runtime
  ros_package_status move_base_msgs
  ros_package_status nav_msgs
  ros_package_status pluginlib
  ros_package_status roscpp
  ros_package_status rospy
  ros_package_status sensor_msgs
  ros_package_status std_msgs
  ros_package_status std_srvs
  ros_package_status gmapping
  ros_package_status hector_mapping
  ros_package_status teleop_twist_keyboard
  ros_package_status tf

  print_section "Python 模块检查"
  python_module_status numpy
  python_module_status torch

  print_section "Gazebo 插件库检查"
  plugin_status libgazebo_ros_diff_drive.so
  plugin_status libgazebo_ros_imu.so
  plugin_status libgazebo_ros_imu_sensor.so
  plugin_status libgazebo_ros_joint_state_publisher.so
  plugin_status libgazebo_ros_laser.so
  plugin_status libgazebo_ros_depth_camera.so
  plugin_status libgazebo_ros_openni_kinect.so

  print_section "仓库关键目录检查"
  file_status "${SRC_ROOT}/mr_description"
  file_status "${SRC_ROOT}/mr_gazebo"
  file_status "${SRC_ROOT}/mr_maps"
  file_status "${SRC_ROOT}/mr_navigation"
  file_status "${SRC_ROOT}/mr_slam"
  file_status "${SRC_ROOT}/mr_traditional_planner"
  file_status "${GAZEBO_MODELS_DIR}"
  file_status "${GAZEBO_WORLDS_DIR}"

  print_section "仓库关键文件检查"
  file_status "${SRC_ROOT}/mr_gazebo/launch/spawn_robot.launch"
  file_status "${SRC_ROOT}/mr_gazebo/launch/spawn_navigation_world.launch"
  file_status "${SRC_ROOT}/mr_gazebo/worlds/empty.world"
  file_status "${SRC_ROOT}/mr_navigation/launch/navigation.launch"
  file_status "${SRC_ROOT}/mr_navigation/launch/navigation_sim.launch"
  file_status "${SRC_ROOT}/mr_navigation/launch/simulation.launch"
  file_status "${SRC_ROOT}/mr_navigation/launch/slam_sim.launch"
  file_status "${SRC_ROOT}/mr_navigation/launch/teleop_keyboard.launch"
  file_status "${SRC_ROOT}/mr_slam/launch/slam.launch"
  file_status "${SRC_ROOT}/mr_slam/launch/gmapping.launch"
  file_status "${SRC_ROOT}/mr_slam/launch/hector.launch"
  file_status "${SRC_ROOT}/mr_slam/config/gmapping_params.yaml"
  file_status "${SRC_ROOT}/mr_slam/config/hector.yaml"
  file_status "${SRC_ROOT}/mr_slam/rviz/slam.rviz"
  file_status "${SRC_ROOT}/mr_traditional_planner/launch/planner.launch"
  file_status "${SRC_ROOT}/mr_traditional_planner/launch/planner_sim.launch"
  file_status "${SRC_ROOT}/mr_description/launch/load_robot_description.launch"
  file_status "${SRC_ROOT}/mr_description/launch/wpb_home_description.launch"
  file_status "${SRC_ROOT}/mr_description/urdf/common_properties.urdf"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_burger.urdf"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_burger.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_waffle.urdf"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_waffle.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_waffle_pi.urdf"
  file_status "${SRC_ROOT}/mr_description/urdf/turtlebot3_waffle_pi.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/project/turtlebot3_burger.simulation.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/project/turtlebot3_waffle.simulation.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/project/turtlebot3_waffle_pi.simulation.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/meshes/bases/burger_base.stl"
  file_status "${SRC_ROOT}/mr_description/meshes/bases/waffle_base.stl"
  file_status "${SRC_ROOT}/mr_description/meshes/bases/waffle_pi_base.stl"
  file_status "${SRC_ROOT}/mr_description/rviz/model.rviz"
  file_status "${SRC_ROOT}/mr_description/urdf/wpb_home/wpb_home.urdf"
  file_status "${SRC_ROOT}/mr_description/urdf/wpb_home/wpb_home_mani.urdf"
  file_status "${SRC_ROOT}/mr_description/urdf/wpb_home/simulation/wpb_home_sim.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/wpb_home/simulation/wpb_home_mani_sim.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/wpb_home/simulation/wpb_home_mani_model.urdf.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/wpb_home/simulation/wpb_home_gazebo_plugins.xacro"
  file_status "${SRC_ROOT}/mr_description/urdf/generated"
  file_status "${SRC_ROOT}/mr_description/meshes/wpb_home/wpb_home.dae"
  file_status "${SRC_ROOT}/mr_description/meshes/wpb_home/wpb_home_mani.dae"
  file_status "${SRC_ROOT}/mr_description/meshes/wpb_home/forearm.dae"
  file_status "${SRC_ROOT}/mr_description/meshes/wpb_home/finger.dae"
  file_status "${SRC_ROOT}/mr_description/rviz/wpb_home/urdf.rviz"
  file_status "${SRC_ROOT}/mr_description/rviz/wpb_home/sensor.rviz"
  file_status "${SRC_ROOT}/mr_description/config/wpb_home/wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/costmap_common_params_burger.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/costmap_common_params_waffle.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/costmap_common_params_waffle_pi.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/dwa_local_planner_params_burger.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/dwa_local_planner_params_waffle.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/dwa_local_planner_params_waffle_pi.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/costmap_common_params_wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/dwa_local_planner_params_wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/global_costmap_params_wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/local_costmap_params_wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/move_base_params_wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/amcl_params_wpb_home.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/costmap_common_params_wpb_home_mani.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/dwa_local_planner_params_wpb_home_mani.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/global_costmap_params_wpb_home_mani.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/local_costmap_params_wpb_home_mani.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/move_base_params_wpb_home_mani.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/amcl_params_wpb_home_mani.yaml"
  file_status "${SRC_ROOT}/mr_navigation/config/robot_models.yaml"
  file_status "${SRC_ROOT}/mr_gazebo/worlds/turtlebot3_world.world"
  file_status "${SRC_ROOT}/mr_gazebo/worlds/stage_1.world"
  file_status "${SRC_ROOT}/mr_gazebo/worlds/maze/maze_1.world"
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
