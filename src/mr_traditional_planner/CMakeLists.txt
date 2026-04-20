cmake_minimum_required(VERSION 3.0.2)
project(mr_traditional_planner)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  nav_msgs
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    geometry_msgs
    move_base_msgs
    nav_msgs
    roscpp
    rospy
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(astar_planner_cpp
  src/optimal/astar_planner_node.cpp
)
target_link_libraries(astar_planner_cpp
  ${catkin_LIBRARIES}
)

add_executable(dijkstra_planner_cpp
  src/optimal/dijkstra_planner_node.cpp
)
target_link_libraries(dijkstra_planner_cpp
  ${catkin_LIBRARIES}
)

add_executable(bcd_planner_cpp
  src/coverage/bcd_planner_node.cpp
)
target_link_libraries(bcd_planner_cpp
  ${catkin_LIBRARIES}
)

add_executable(stc_planner_cpp
  src/coverage/stc_planner_node.cpp
)
target_link_libraries(stc_planner_cpp
  ${catkin_LIBRARIES}
)

add_dependencies(astar_planner_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(dijkstra_planner_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(bcd_planner_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(stc_planner_cpp ${catkin_EXPORTED_TARGETS})

# 预先接入 Python 节点安装流程，后续补算法实现时不需要再改构建骨架。
catkin_install_python(PROGRAMS
  scripts/optimal/astar_planner_py.py
  scripts/optimal/dijkstra_planner_py.py
  scripts/coverage/bcd_planner_py.py
  scripts/coverage/stc_planner_py.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
  astar_planner_cpp
  dijkstra_planner_cpp
  bcd_planner_cpp
  stc_planner_cpp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include launch scripts/utils
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
