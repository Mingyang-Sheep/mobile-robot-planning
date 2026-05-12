"""Experience replay buffer for off-policy RL agents."""

import random
from collections import deque


class ReplayBuffer:
    """Fixed-size FIFO replay buffer.

    Args:
        capacity: Maximum number of transitions to store.
    """

    def __init__(self, capacity=1_000_000):
        self._buffer = deque(maxlen=capacity)

    def append(self, state, action, reward, next_state, done):
        self._buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        return random.sample(self._buffer, batch_size)

    def __len__(self):
        return len(self._buffer)
