<div align="right">

[Chinese](zh/learning_planning.md)

</div>

# Learning Planning Module

This page documents the current `mr_learning` scope. The module should be treated as an experimental stage 1 DQN demo, not a mature reinforcement-learning planning platform.

The current training entry uses DQN with replay buffer, target network updates, epsilon-greedy exploration, and saved `.pth`/`.json` checkpoints. The environment reads laser and odometry data, publishes `/cmd_vel`, and uses Gazebo services for reset and goal handling.

Missing pieces include multi-stage training entries, mature train/test separation, standard evaluation metrics, complex dynamic obstacle scenarios, and integration as a Navigation planner plugin.
