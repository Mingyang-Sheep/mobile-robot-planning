<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# 安装与环境准备

适合读者：第一次在 Ubuntu 20.04 + ROS Noetic 环境中使用本仓库的用户。

本文只描述当前仓库实际需要的依赖。安装脚本不会自动修改 shell 启动文件，也不会自动写入用户系统配置。

## 1. 系统要求

| 项目 | 要求 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| Gazebo | Gazebo 11 |
| 构建工具 | catkin / catkin_make |
| Python | Python 3 |

仓库面向 ROS 1 Noetic，不是 ROS 2 工程。

## 2. 获取依赖清单

先进入仓库：

```bash
cd ~/mobile_robot_benchmark
```

查看当前环境：

```bash
bash tools/check_environment.sh
```

安装系统依赖：

```bash
sudo bash tools/install_dependencies.sh
```

`install_dependencies.sh` 会检查 Ubuntu 和 ROS 环境，并安装仓库中 package.xml、launch 和源码实际使用到的 ROS 包。脚本支持重复执行，已安装的 apt 包不会因此报错。

## 3. 编译仓库

每个新终端都需要先加载 ROS：

```bash
source /opt/ros/noetic/setup.bash
```

然后编译：

```bash
cd ~/mobile_robot_benchmark
catkin_make
```

编译成功后加载本工作区：

```bash
source devel/setup.bash
```

如果打开了多个终端，每个终端都要执行：

```bash
cd ~/mobile_robot_benchmark
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

## 4. 为什么需要 source

ROS 使用环境变量查找 package、消息类型、插件和动态库。常见变量包括：

| 变量 | 作用 |
|---|---|
| `ROS_PACKAGE_PATH` | 让 `roslaunch` 和 `rospack` 找到 `src/mr_*` 包 |
| `CMAKE_PREFIX_PATH` | 让 catkin 找到已编译的工作区 |
| `PYTHONPATH` | 让 Python 节点找到工作区生成的 Python 包 |
| `LD_LIBRARY_PATH` | 让 pluginlib 和 ROS 节点找到 C++ 动态库 |

没有 `source devel/setup.bash` 时，常见现象是 `Resource not found`、插件加载失败、Python 节点找不到包。

## 5. 主要依赖

当前仓库使用的核心 ROS 依赖包括：

- `gazebo_ros`、`gazebo_plugins`
- `xacro`、`urdf`、`robot_state_publisher`
- `joint_state_publisher`、`joint_state_publisher_gui`
- `map_server`
- `amcl`
- `move_base`
- `navfn`
- `dwa_local_planner`
- `costmap_2d`
- `nav_core`
- `base_local_planner`
- `pluginlib`
- `tf`、`tf2_ros`
- `slam_gmapping` 对应的 ROS package `gmapping`
- `hector_mapping`
- `rviz`
- `rqt_tf_tree`
- `teleop_twist_keyboard`

Learning 模块使用 Python `numpy` 和 `torch`。`numpy` 可通过 apt 安装；`torch` 的版本通常受 CPU/GPU、CUDA 和 Python 环境影响，安装脚本默认只检查并提示，不强制安装 PyTorch。

## 6. 快速环境检查

编译并 source 后执行：

```bash
bash tools/check_environment.sh
```

重点看这些检查项：

- `ROS_DISTRO=noetic`
- `roslaunch`、`rospack`、`catkin_make` 存在
- Gazebo 版本可读取
- `rospack find mr_navigation` 成功
- `rospack find mr_traditional_planner` 成功
- 关键 ROS 依赖包可找到
- `numpy` 可导入
- `torch` 如果不存在，会被标记为 Learning 可选依赖缺失

## 7. 下一步阅读

安装完成后继续阅读 [quick_start.md](quick_start.md)。如果环境检查失败，先看 [troubleshooting.md](troubleshooting.md)。

---

<a id="english"></a>

## English

This page describes the environment needed by the current repository: Ubuntu 20.04, ROS Noetic, Gazebo 11, catkin, and Python 3.

Recommended flow:

- Source ROS with `source /opt/ros/noetic/setup.bash`.
- Run `bash tools/check_environment.sh` to inspect the local setup.
- Install ROS dependencies with `sudo bash tools/install_dependencies.sh`.
- Build with `catkin_make` from the workspace root.
- Source `devel/setup.bash` in every terminal that runs ROS commands.

Learning features need `numpy` and optionally `torch`. PyTorch is not forced by the install script because CPU/GPU and CUDA choices vary by machine.
