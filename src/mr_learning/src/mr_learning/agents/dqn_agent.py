"""DQN agent with target network and experience replay (PyTorch)."""

import json
import os
import random

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from mr_learning.agents.replay_buffer import ReplayBuffer


class QNetwork(nn.Module):
    def __init__(self, state_size, action_size):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_size, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(64, action_size),
        )

    def forward(self, x):
        return self.net(x)


class DQNAgent:
    """Deep Q-Network agent.

    Args:
        state_size: Dimension of the state vector.
        action_size: Number of discrete actions.
        gamma: Discount factor.
        lr: Learning rate.
        epsilon: Initial exploration rate.
        epsilon_decay: Multiplicative decay per episode.
        epsilon_min: Minimum exploration rate.
        batch_size: Mini-batch size for training.
        train_start: Minimum memory size before training begins.
        target_update_period: Steps between target network updates.
        buffer_capacity: Replay buffer capacity.
    """

    def __init__(
        self,
        state_size,
        action_size,
        gamma=0.99,
        lr=0.00025,
        epsilon=1.0,
        epsilon_decay=0.99,
        epsilon_min=0.05,
        batch_size=64,
        train_start=64,
        target_update_period=2000,
        buffer_capacity=1_000_000,
    ):
        self.state_size = state_size
        self.action_size = action_size
        self.gamma = gamma
        self.epsilon = epsilon
        self.epsilon_decay = epsilon_decay
        self.epsilon_min = epsilon_min
        self.batch_size = batch_size
        self.train_start = train_start
        self.target_update_period = target_update_period

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.model = QNetwork(state_size, action_size).to(self.device)
        self.target_model = QNetwork(state_size, action_size).to(self.device)
        self.target_model.load_state_dict(self.model.state_dict())
        self.target_model.eval()

        self.optimizer = optim.Adam(self.model.parameters(), lr=lr)
        self.loss_fn = nn.MSELoss()

        self.memory = ReplayBuffer(buffer_capacity)
        self.q_value = np.zeros(action_size)
        self._step_count = 0
        self._episode_count = 0

    def get_action(self, state):
        if np.random.rand() <= self.epsilon:
            self.q_value = np.zeros(self.action_size)
            return random.randrange(self.action_size)

        with torch.no_grad():
            s = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            q = self.model(s).cpu().numpy()[0]
        self.q_value = q
        return int(np.argmax(q))

    def append_memory(self, state, action, reward, next_state, done):
        self.memory.append(state, action, reward, next_state, done)

    def train_model(self):
        """Sample a mini-batch and perform one gradient step.

        Returns:
            loss value (float) or None if not enough data.
        """
        if len(self.memory) < self.train_start:
            return None

        batch = self.memory.sample(self.batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)

        states_t = torch.FloatTensor(np.array(states)).to(self.device)
        actions_t = torch.LongTensor(actions).to(self.device)
        rewards_t = torch.FloatTensor(rewards).to(self.device)
        next_states_t = torch.FloatTensor(np.array(next_states)).to(self.device)
        dones_t = torch.FloatTensor(dones).to(self.device)

        # Current Q values
        q_values = self.model(states_t)
        q_taken = q_values.gather(1, actions_t.unsqueeze(1)).squeeze(1)

        # Target Q values
        with torch.no_grad():
            next_q = self.target_model(next_states_t).max(dim=1)[0]
            target = rewards_t + (1.0 - dones_t) * self.gamma * next_q

        loss = self.loss_fn(q_taken, target)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()

        self._step_count += 1
        if self._step_count % self.target_update_period == 0:
            self.update_target()

        return loss.item()

    def update_target(self):
        self.target_model.load_state_dict(self.model.state_dict())

    def decay_epsilon(self):
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

    def save(self, path):
        """Save model weights and training params."""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save(self.model.state_dict(), path + ".pth")
        params = {"epsilon": self.epsilon, "episode": self._episode_count}
        with open(path + ".json", "w") as f:
            json.dump(params, f)

    def load(self, path):
        """Load model weights and training params."""
        pth = path + ".pth"
        if os.path.isfile(pth):
            self.model.load_state_dict(torch.load(pth, map_location=self.device))
            self.update_target()
        json_path = path + ".json"
        if os.path.isfile(json_path):
            with open(json_path) as f:
                params = json.load(f)
            self.epsilon = params.get("epsilon", self.epsilon)
            self._episode_count = params.get("episode", 0)
