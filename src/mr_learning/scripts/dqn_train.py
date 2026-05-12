#!/usr/bin/env python3
"""DQN training node for mobile robot navigation."""

import os
import sys
import time

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

from mr_learning.agents.dqn_agent import DQNAgent
from mr_learning.environments.stage_1_env import Stage1Env
from mr_learning.goal_manager import GoalManager

# Map stage number → environment class
_ENV_MAP = {
    "1": Stage1Env,
}

# Default obstacle positions for each stage (matching world files)
_OBSTACLE_MAP = {
    "1": [],  # stage_1 has no obstacles
}


def _find_goal_sdf():
    """Locate the goal_box SDF relative to mr_gazebo models."""
    try:
        import rospkg
        rp = rospkg.RosPack()
        gazebo_path = rp.get_path("mr_gazebo")
        sdf = os.path.join(gazebo_path, "models", "turtlebot3_square", "goal_box", "model.sdf")
        if os.path.isfile(sdf):
            return sdf
    except Exception:
        pass
    # Fallback: search from this file's location
    base = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        candidate = os.path.join(base, "src", "mr_gazebo", "models", "turtlebot3_square", "goal_box", "model.sdf")
        if os.path.isfile(candidate):
            return candidate
        base = os.path.dirname(base)
    raise FileNotFoundError("Cannot find goal_box/model.sdf — set ~goal_sdf_path param manually.")


def main():
    rospy.init_node("dqn_train")

    stage = rospy.get_param("~stage", "1")
    episodes = rospy.get_param("~episodes", 3000)
    max_steps = rospy.get_param("~max_steps", 6000)
    load_model = rospy.get_param("~load_model", False)

    env_cls = _ENV_MAP.get(stage)
    if env_cls is None:
        rospy.logfatal("Unknown stage '%s'. Available: %s", stage, list(_ENV_MAP.keys()))
        sys.exit(1)

    # Goal manager
    goal_sdf = rospy.get_param("~goal_sdf_path", _find_goal_sdf())
    obstacles = _OBSTACLE_MAP.get(stage, [])
    goal_mgr = GoalManager(goal_sdf_path=goal_sdf, obstacle_positions=obstacles)

    # Environment
    env = env_cls(goal_manager=goal_mgr, max_steps_per_episode=max_steps)

    # Agent
    state_size = 364  # 360 laser + heading + dist + 2 padding
    action_size = 5
    agent = DQNAgent(state_size=state_size, action_size=action_size)

    # Model save path
    pkg_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    save_dir = os.path.join(pkg_path, "save_model", "stage_" + stage)
    os.makedirs(save_dir, exist_ok=True)
    save_prefix = os.path.join(save_dir, "stage_" + stage + "_")

    if load_model:
        agent.load(save_prefix + "latest")
        rospy.loginfo("Loaded model from %slatest", save_prefix)

    # Publishers
    pub_result = rospy.Publisher("result", Float32MultiArray, queue_size=5)
    pub_action = rospy.Publisher("get_action", Float32MultiArray, queue_size=5)

    result_msg = Float32MultiArray()
    action_msg = Float32MultiArray()

    start_time = time.time()

    for ep in range(agent._episode_count + 1, episodes + 1):
        state = env.reset()
        score = 0.0

        for step in range(max_steps):
            if rospy.is_shutdown():
                env.close()
                return

            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            agent.append_memory(state, action, reward, next_state, done)

            loss = agent.train_model()

            score += reward
            state = next_state

            action_msg.data = [float(action), score, reward]
            pub_action.publish(action_msg)

            if done:
                break

        # End of episode
        agent.decay_epsilon()
        agent._episode_count = ep

        result_msg.data = [score, float(np.max(agent.q_value))]
        pub_result.publish(result_msg)

        elapsed = time.time() - start_time
        h, remainder = divmod(int(elapsed), 3600)
        m, s = divmod(remainder, 60)
        rospy.loginfo(
            "Ep: %d  score: %.2f  memory: %d  epsilon: %.3f  time: %d:%02d:%02d",
            ep, score, len(agent.memory), agent.epsilon, h, m, s,
        )

        # Save model periodically
        if ep % 10 == 0:
            agent.save(save_prefix + str(ep))
            agent.save(save_prefix + "latest")

    env.close()
    rospy.loginfo("Training complete after %d episodes.", episodes)


if __name__ == "__main__":
    main()
