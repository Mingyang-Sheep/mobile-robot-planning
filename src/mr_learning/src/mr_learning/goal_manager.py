"""Goal spawn/delete manager for Gazebo-based learning environments."""

import math
import os
import random

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose


class GoalManager:
    """Manages goal_box model in Gazebo: spawn, delete, random position generation.

    Args:
        goal_sdf_path: Absolute path to goal_box/model.sdf.
        obstacle_positions: List of (x, y) tuples for obstacle centers.
        goal_range: (min, max) for random goal coordinate generation.
        min_obstacle_dist: Minimum distance from obstacles and origin.
        min_goal_spacing: Minimum distance between consecutive goals.
        model_name: Gazebo model name for the goal marker.
    """

    def __init__(
        self,
        goal_sdf_path,
        obstacle_positions=None,
        goal_range=(-1.2, 1.2),
        min_obstacle_dist=0.4,
        min_goal_spacing=1.0,
        model_name="goal",
    ):
        with open(goal_sdf_path, "r") as f:
            self._sdf = f.read()

        self._obstacles = obstacle_positions or []
        self._goal_range = goal_range
        self._min_obstacle_dist = min_obstacle_dist
        self._min_goal_spacing = min_goal_spacing
        self._model_name = model_name

        self._goal_pose = Pose()
        self._goal_pose.position.x = 0.6
        self._goal_pose.position.y = 0.0

        self._last_x = self._goal_pose.position.x
        self._last_y = self._goal_pose.position.y
        self._goal_exists = False

        self._sub_model_states = rospy.Subscriber(
            "gazebo/model_states", ModelStates, self._model_states_cb
        )

    def _model_states_cb(self, msg):
        self._goal_exists = self._model_name in msg.name

    def _spawn(self):
        while not rospy.is_shutdown():
            if not self._goal_exists:
                try:
                    rospy.wait_for_service("gazebo/spawn_sdf_model", timeout=5.0)
                    spawner = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
                    spawner(self._model_name, self._sdf, "", self._goal_pose, "world")
                    rospy.loginfo(
                        "Goal spawned at (%.2f, %.2f)",
                        self._goal_pose.position.x,
                        self._goal_pose.position.y,
                    )
                    return
                except rospy.ServiceException as e:
                    rospy.logwarn("Failed to spawn goal: %s", e)
            rospy.sleep(0.1)

    def _delete(self):
        while not rospy.is_shutdown():
            if self._goal_exists:
                try:
                    rospy.wait_for_service("gazebo/delete_model", timeout=5.0)
                    deleter = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
                    deleter(self._model_name)
                    return
                except rospy.ServiceException as e:
                    rospy.logwarn("Failed to delete goal: %s", e)
            rospy.sleep(0.1)

    def respawn(self, delete_first=False):
        """Delete existing goal (if requested), generate new valid position, spawn.

        Returns:
            (goal_x, goal_y) tuple.
        """
        if delete_first:
            self._delete()

        x, y = self._generate_position()
        self._goal_pose.position.x = x
        self._goal_pose.position.y = y

        rospy.sleep(0.3)
        self._spawn()

        self._last_x = x
        self._last_y = y
        return x, y

    def _generate_position(self):
        """Generate a random goal position that avoids obstacles, origin, and last goal."""
        lo, hi = self._goal_range
        steps = int(round((hi - lo) * 10)) + 1
        candidates = [lo + i * 0.1 for i in range(steps)]

        while not rospy.is_shutdown():
            x = random.choice(candidates)
            y = random.choice(candidates)

            # Reject if too close to origin
            if abs(x) < self._min_obstacle_dist and abs(y) < self._min_obstacle_dist:
                continue

            # Reject if too close to any obstacle
            too_close = False
            for ox, oy in self._obstacles:
                if abs(x - ox) <= self._min_obstacle_dist and abs(y - oy) <= self._min_obstacle_dist:
                    too_close = True
                    break
            if too_close:
                continue

            # Reject if too close to previous goal
            if math.hypot(x - self._last_x, y - self._last_y) < self._min_goal_spacing:
                continue

            return x, y

        return 0.6, 0.0  # fallback
