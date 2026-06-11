cmake_minimum_required(VERSION 3.0.2)
project(mr_traditional_planner)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  move_base_msgs
  nav_msgs
  nav_core
  pluginlib
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mr_traditional_planner_plugins
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    move_base_msgs
    nav_msgs
    nav_core
    pluginlib
    roscpp
    rospy
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mr_traditional_planner_plugins
  src/optimal/astar_planner_node.cpp
  src/optimal/dijkstra_planner_node.cpp
  src/optimal/dstar_planner_node.cpp
  src/optimal/dstar_lite_planner_node.cpp
  src/optimal/theta_star_planner_node.cpp
  src/optimal/rrt_star_planner_node.cpp
  src/optimal/dwa_planner_node.cpp
  src/optimal/cubic_spline_planner_node.cpp
  src/coverage/bcd_planner_node.cpp
  src/coverage/stc_planner_node.cpp
  src/nav_core/global_planner_adapter.cpp
)
target_link_libraries(mr_traditional_planner_plugins
  ${catkin_LIBRARIES}
)

add_executable(planner_plugin_node
  src/planner_plugin_node.cpp
)
target_link_libraries(planner_plugin_node
  ${catkin_LIBRARIES}
)

add_dependencies(mr_traditional_planner_plugins ${catkin_EXPORTED_TARGETS})
add_dependencies(planner_plugin_node ${catkin_EXPORTED_TARGETS})

# Python 节点和包安装。
catkin_install_python(PROGRAMS
  scripts/python_planner_node.py
  scripts/planner_compatibility_validator.py
  scripts/planner_truth_matrix_check.py
  scripts/optimal/astar_planner_py.py
  scripts/optimal/dijkstra_planner_py.py
  scripts/optimal/dstar_planner_py.py
  scripts/optimal/dstar_lite_planner_py.py
  scripts/optimal/theta_star_planner_py.py
  scripts/optimal/rrt_star_planner_py.py
  scripts/optimal/dwa_planner_py.py
  scripts/optimal/cubic_spline_planner_py.py
  scripts/coverage/bcd_planner_py.py
  scripts/coverage/stc_planner_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  mr_traditional_planner_plugins
  planner_plugin_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES planner_plugins.xml
  THIRD_PARTY_NOTICES.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include launch scripts/utils
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

# Install Python packages so python_planner_node.py can import them at runtime.
install(DIRECTORY scripts/optimal scripts/coverage scripts/utils
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)
