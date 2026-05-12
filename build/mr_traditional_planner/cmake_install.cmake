# Install script for directory: /home/lmy/mobile_robot_benchmark/src/mr_traditional_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lmy/mobile_robot_benchmark/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/mr_traditional_planner.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mr_traditional_planner/cmake" TYPE FILE FILES
    "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/mr_traditional_plannerConfig.cmake"
    "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/mr_traditional_plannerConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mr_traditional_planner" TYPE FILE FILES "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/python_planner_node.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/astar_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/dijkstra_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/dstar_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/dstar_lite_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/theta_star_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/rrt_star_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/dwa_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/cubic_spline_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/bcd_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE PROGRAM FILES "/home/lmy/mobile_robot_benchmark/build/mr_traditional_planner/catkin_generated/installspace/stc_planner_py.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/lmy/mobile_robot_benchmark/devel/lib/libmr_traditional_planner_plugins.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmr_traditional_planner_plugins.so")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE EXECUTABLE FILES "/home/lmy/mobile_robot_benchmark/devel/lib/mr_traditional_planner/planner_plugin_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node"
         OLD_RPATH "/opt/ros/noetic/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner/planner_plugin_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mr_traditional_planner" TYPE FILE FILES
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/planner_plugins.xml"
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/THIRD_PARTY_NOTICES.md"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mr_traditional_planner" TYPE DIRECTORY FILES
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/include"
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/launch"
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/utils"
    USE_SOURCE_PERMISSIONS)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mr_traditional_planner" TYPE DIRECTORY FILES
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/optimal"
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/coverage"
    "/home/lmy/mobile_robot_benchmark/src/mr_traditional_planner/scripts/utils"
    USE_SOURCE_PERMISSIONS FILES_MATCHING REGEX "/[^/]*\\.py$")
endif()

