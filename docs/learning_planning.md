<div align="right">

[中文](#中文) | [English](#english)

</div>

<a id="中文"></a>
# Learning 规划模块

适合读者：想了解 `mr_learning` 当前真实完成度，或准备继续扩展强化学习环境的开发者。

当前 `mr_learning` 应视为实验性 DQN Demo，而不是成熟的强化学习规划平台。它已经提供 stage 1 Gazebo 环境封装、DQN agent、训练入口和模型保存逻辑，但环境规模、任务复杂度、评估脚本和与 Navigation 主链路的集成仍需要继续开发。

## 1. 当前文件结构

```text
src/mr_learning/
  launch/
    learning_env.launch
    dqn_train.launch
  scripts/
    dqn_train.py
  src/mr_learning/
    agents/dqn_agent.py
    agents/replay_buffer.py
    environments/base_env.py
    environments/stage_1_env.py
    goal_manager.py
  save_model/stage_1/
```

## 2. 当前算法

当前训练入口使用 DQN：

- 网络：`state_size -> 64 -> 64 -> action_size`
- 损失：MSE
- 优化器：Adam
- replay buffer：经验回放
- target network：按步数周期更新
- epsilon-greedy：随训练衰减
- device：`cuda` 可用时使用 CUDA，否则使用 CPU

源码位置：

```text
src/mr_learning/src/mr_learning/agents/dqn_agent.py
```

## 3. 当前环境

当前 `dqn_train.py` 只把 stage `"1"` 映射到 `Stage1Env`：

```text
_ENV_MAP = {"1": Stage1Env}
```

启动文件：

```bash
roslaunch mr_learning dqn_train.launch stage:=1
```

环境入口：

```text
src/mr_learning/src/mr_learning/environments/stage_1_env.py
```

Gazebo world 默认来自：

```text
src/mr_gazebo/worlds/stage_1.world
```

## 4. 状态、动作和奖励

当前状态维度：

```text
364 = 360 维激光 + heading + goal distance + 2 个占位值
```

动作空间：

```text
5 个离散动作
```

`BaseEnv` 中动作会映射为角速度，线速度默认保持较小正值。

奖励逻辑在 `Stage1Env` 中定义：

- 碰撞：负奖励；
- 到达目标：正奖励；
- 普通过程：结合朝向误差和距离变化给奖励。

碰撞检测基于 LaserScan 最小距离阈值。

## 5. ROS / Gazebo 对接

当前环境使用：

| 接口 | 用途 |
|---|---|
| `/cmd_vel` | 发布动作速度 |
| `/odom` | 读取机器人位姿 |
| `/scan` 或 `scan` | 读取激光，当前 `_read_scan()` 中实际等待的是 `scan` |
| Gazebo services | 重置仿真、生成/删除目标模型 |

目标模型通过 `GoalManager` 使用 Gazebo service 生成和删除。

## 6. 模型保存

训练脚本会把模型保存到：

```text
src/mr_learning/save_model/stage_1/
```

保存内容包括 `.pth` 权重和 `.json` 元数据。仓库中已经存在若干 stage 1 保存文件，说明该目录当前不是空占位。

## 7. CPU 虚拟机适用范围

CPU 虚拟机可以用于：

- 检查 launch 是否能展开；
- 跑短时 episode；
- 调试 topic、reward 和 reset 流程；
- 验证模型保存路径。

CPU 虚拟机不适合承诺：

- 大规模训练速度；
- 长时间稳定训练；
- 与真实机器人同等的策略效果；
- 多场景泛化能力。

训练时建议先关闭 Gazebo GUI：

```bash
roslaunch mr_learning dqn_train.launch stage:=1 gui:=false
```

## 8. 当前没有完成的内容

当前代码中不能描述为已完成的内容：

- 多 stage 环境训练入口；
- PPO、Q-learning 等其他算法入口；
- 成熟的训练/测试分离脚本；
- 标准化评估指标和结果图表；
- 与 Navigation 的 move_base 规划器插件化集成；
- 复杂动态障碍环境；
- 自动课程学习或多机器人训练。

## 9. 扩展方向

后续开发者可以继续扩展：

- 为 `stage_2`、`stage_3`、`stage_4` 增加环境类和 `_ENV_MAP` 映射；
- 明确 scan topic 参数，避免 `BaseEnv._read_scan()` 固定等待 `scan`；
- 增加独立测试脚本；
- 增加 reward/episode 日志可视化；
- 增加模型评估和对比指标；
- 将学习策略与 Navigation 或 planner framework 做清晰接口定义。

## 10. 下一步阅读

Learning 依赖安装看 [installation.md](installation.md)，Gazebo 环境看 [gazebo_simulation.md](gazebo_simulation.md)。

---

<a id="english"></a>

## English

This page documents the current `mr_learning` scope. The module should be treated as an experimental stage 1 DQN demo, not a mature reinforcement-learning planning platform.

The current training entry uses DQN with replay buffer, target network updates, epsilon-greedy exploration, and saved `.pth`/`.json` checkpoints. The environment reads laser and odometry data, publishes `/cmd_vel`, and uses Gazebo services for reset and goal handling.

Missing pieces include multi-stage training entries, mature train/test separation, standard evaluation metrics, complex dynamic obstacle scenarios, and integration as a Navigation planner plugin.
