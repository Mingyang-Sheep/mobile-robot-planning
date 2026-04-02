#!/bin/bash

# ==============================================================================
# Turtlebot3 强化学习专属环境 (ROS Noetic + Conda + TensorFlow) 一键部署脚本
# ==============================================================================

echo "🚀 开始部署 Turtlebot3 强化学习专属 Conda 环境..."

# 1. 确保 Conda 命令在脚本中可用 (核心黑魔法)
# 因为 bash 脚本默认不加载 conda 环境，需要手动挂载 hook
eval "$(conda shell.bash hook)"

# 2. 检查是否已经存在同名环境，如果存在则提醒
ENV_NAME="turtle_rl"
if conda info --envs | grep -q "^$ENV_NAME "; then
    echo "⚠️ 发现已存在名为 $ENV_NAME 的环境！"
    read -p "是否删除旧环境并重新安装？(y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        conda env remove -n $ENV_NAME -y
    else
        echo "🛑 部署中止。"
        exit 1
    fi
fi

# 3. 创建纯净的 Python 3.8 虚拟环境 (兼容 ROS Noetic)
echo "📦 正在创建 Python 3.8 虚拟环境..."
conda create -n $ENV_NAME python=3.8 -y

# 4. 激活环境
echo "🔄 激活 $ENV_NAME 环境..."
conda activate $ENV_NAME

# 5. 安装核心深度学习框架 (TensorFlow CPU 版 + Keras)
echo "🧠 正在植入 AI 大脑 (TensorFlow & Keras)..."
pip install tensorflow-cpu keras

# 6. 安装 ROS-Conda 跨界通信依赖包
echo "🔗 正在打通 ROS 与 Conda 的通信链路..."
pip install rospkg pyyaml catkin_pkg defusedxml

# 7. 安装精确版本的 ROS 编译模板工具 (极其重要的避坑点！)
echo "🔨 正在安装兼容 ROS Noetic 的编译工具 (empy 3.3.4)..."
pip install empy==3.3.4

echo "=============================================================================="
echo "🎉 部署完成！"
echo "👉 请在终端输入以下命令激活您的专属环境："
echo "   conda activate $ENV_NAME"
echo "=============================================================================="
