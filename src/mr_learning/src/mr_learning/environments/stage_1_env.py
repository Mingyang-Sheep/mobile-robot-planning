"""Stage 1 environment: 4x4m square, 4 static obstacles, simplest configuration."""

import math
import numpy as np

from mr_learning.environments.base_env import BaseEnv

PI = math.pi


class Stage1Env(BaseEnv):
    """Stage 1: static obstacles, 360-beam laser, 5 discrete actions.

    State vector (364 dims):
        [0:360)  — laser scan (clipped to 3.5m)
        [360]    — heading (angle to goal)
        [361]    — current goal distance
        [362]    — 0.0 (obstacle min range placeholder)
        [363]    — 0.0 (obstacle angle placeholder)

    Reward:
        collision  → -200
        goal reach → +200
        otherwise  → yaw_reward[action] * 5 * distance_rate
    """

    def __init__(self, goal_manager, **kwargs):
        defaults = dict(
            action_size=5,
            max_linear_vel=0.15,
            max_angular_vel=1.5,
            collision_dist=0.13,
            goal_reached_dist=0.2,
            laser_clip=3.5,
        )
        defaults.update(kwargs)
        super().__init__(goal_manager=goal_manager, **defaults)
        self._start_goal_dist = self.get_goal_distance()

    def _compute_state(self, scan_range, collision):
        heading = self.heading
        current_dist = self.get_goal_distance()
        return scan_range + [heading, current_dist, 0.0, 0.0]

    def _compute_reward(self, state, collision, action):
        heading = state[-4]
        current_dist = state[-3]

        # Yaw reward: how well the chosen action aligns with the goal direction
        yaw_reward = []
        for i in range(self.action_size):
            angle = -PI / 4.0 + heading + (PI / 8.0 * i) + PI / 2.0
            tr = 1.0 - 4.0 * math.fabs(0.5 - math.modf(0.25 + 0.5 * angle % (2.0 * PI) / PI)[0])
            yaw_reward.append(tr)

        distance_rate = 2.0 ** (current_dist / self._start_goal_dist)
        reward = round(yaw_reward[action] * 5.0, 2) * distance_rate

        if collision:
            reward = -200.0

        if self.goal_reached:
            reward = 200.0
            self._start_goal_dist = self.get_goal_distance()

        return reward

    def reset(self):
        state = super().reset()
        self._start_goal_dist = self.get_goal_distance()
        return state
