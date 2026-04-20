#!/bin/bash
echo "================================================="
echo "  TurtleBot3 螺旋全覆盖导航仿真 - 环境依赖安装脚本 "
echo "================================================="

# 1. 更新软件源
sudo apt-get update

# 2. 安装 ROS 导航基础包 (Noetic版本)
echo ">>> 正在安装 ROS Navigation Stack..."
sudo apt-get install -y ros-noetic-navigation

# 3. 安装 TEB 局部规划器 (项目的核心灵魂)
echo ">>> 正在安装 TEB Local Planner..."
sudo apt-get install -y ros-noetic-teb-local-planner

# 4. 安装 TurtleBot3 官方依赖与仿真环境
echo ">>> 正在安装 TurtleBot3 相关包..."
sudo apt-get install -y ros-noetic-dynamixel-sdk
sudo apt-get install -y ros-noetic-turtlebot3-msgs
sudo apt-get install -y ros-noetic-turtlebot3
sudo apt-get install -y ros-noetic-turtlebot3-simulations

# 5. 提示用户编译
echo "================================================="
echo "  依赖安装完成！"
echo "  请在工作空间根目录下运行以下命令进行编译："
echo "  catkin_make"
echo "  source devel/setup.bash"
echo "================================================="