"""Base Gazebo environment for learning-based navigation.

Robot-agnostic: all robot-specific parameters are read from ROS params.
"""

import math
import numpy as np

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion

from mr_learning.goal_manager import GoalManager


class BaseEnv:
    """Gym-like environment wrapping Gazebo simulation.

    Args:
        action_size: Number of discrete actions.
        goal_manager: GoalManager instance for goal spawning.
        scan_topic: LaserScan topic name.
        cmd_vel_topic: Twist command topic.
        odom_topic: Odometry topic.
        max_linear_vel: Fixed linear velocity (m/s).
        max_angular_vel: Maximum angular velocity (rad/s).
        collision_dist: Minimum laser range before collision (m).
        goal_reached_dist: Distance threshold for goal arrival (m).
        max_steps_per_episode: Step limit per episode.
        laser_clip: Max laser range for normalization (m).
    """

    def __init__(
        self,
        action_size,
        goal_manager,
        scan_topic="/scan",
        cmd_vel_topic="/cmd_vel",
        odom_topic="/odom",
        max_linear_vel=0.15,
        max_angular_vel=1.5,
        collision_dist=0.13,
        goal_reached_dist=0.2,
        max_steps_per_episode=6000,
        laser_clip=3.5,
    ):
        self.action_size = action_size
        self.goal_manager = goal_manager
        self.max_linear_vel = max_linear_vel
        self.max_angular_vel = max_angular_vel
        self.collision_dist = collision_dist
        self.goal_reached_dist = goal_reached_dist
        self.max_steps = max_steps_per_episode
        self.laser_clip = laser_clip

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.heading = 0.0
        self.position = Point()
        self.goal_reached = False
        self._init_goal = True

        self._pub_cmd = rospy.Publisher(cmd_vel_topic, Twist, queue_size=5)
        self._sub_odom = rospy.Subscriber(odom_topic, Odometry, self._odom_cb)
        self._reset_proxy = rospy.ServiceProxy("gazebo/reset_simulation", Empty)
        self._unpause_proxy = rospy.ServiceProxy("gazebo/unpause_physics", Empty)
        self._pause_proxy = rospy.ServiceProxy("gazebo/pause_physics", Empty)

    def _odom_cb(self, msg):
        self.position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        goal_angle = math.atan2(self.goal_y - self.position.y, self.goal_x - self.position.x)
        heading = goal_angle - yaw
        if heading > math.pi:
            heading -= 2.0 * math.pi
        elif heading < -math.pi:
            heading += 2.0 * math.pi
        self.heading = round(heading, 2)

    def get_goal_distance(self):
        return round(
            math.hypot(self.goal_x - self.position.x, self.goal_y - self.position.y), 2
        )

    def get_position(self):
        return [self.position.x, self.position.y]

    def get_goal(self):
        return [self.goal_x, self.goal_y]

    def _read_scan(self):
        """Block until a LaserScan message is received."""
        data = None
        while data is None and not rospy.is_shutdown():
            try:
                data = rospy.wait_for_message("scan", LaserScan, timeout=5.0)
            except rospy.ROSException:
                pass
        return data

    def _process_scan(self, scan_msg):
        """Convert LaserScan to clipped list and compute collision flag.

        Returns:
            (scan_list, collision): scan_list is list of floats (Inf→laser_clip, NaN→0),
                                    collision is True if any valid range < collision_dist.
        """
        scan_range = []
        for r in scan_msg.ranges:
            if r == float("Inf"):
                scan_range.append(self.laser_clip)
            elif np.isnan(r):
                scan_range.append(0.0)
            else:
                scan_range.append(r)

        collision = any(0 < r < self.collision_dist for r in scan_range)
        return scan_range, collision

    def _compute_state(self, scan_range, collision):
        """Override in subclass to build the state vector."""
        raise NotImplementedError

    def _compute_reward(self, state, collision, action):
        """Override in subclass to compute the reward."""
        raise NotImplementedError

    def reset(self):
        """Reset simulation, respawn goal, return initial state."""
        rospy.wait_for_service("gazebo/reset_simulation")
        try:
            self._reset_proxy()
        except rospy.ServiceException as e:
            rospy.logwarn("gazebo/reset_simulation failed: %s", e)

        scan_msg = self._read_scan()
        if scan_msg is None:
            return np.zeros(1)

        if self._init_goal:
            self.goal_x, self.goal_y = self.goal_manager.respawn()
            self._init_goal = False

        self.goal_reached = False
        scan_range, collision = self._process_scan(scan_msg)
        state = self._compute_state(scan_range, collision)
        return np.asarray(state)

    def step(self, action):
        """Execute one step: publish cmd_vel, read sensors, compute state/reward.

        Returns:
            (state, reward, done)
        """
        # Convert discrete action to angular velocity
        ang_vel = ((self.action_size - 1) / 2.0 - action) * self.max_angular_vel * 0.5
        cmd = Twist()
        cmd.linear.x = self.max_linear_vel
        cmd.angular.z = ang_vel
        self._pub_cmd.publish(cmd)

        scan_msg = self._read_scan()
        if scan_msg is None:
            return np.zeros(1), 0.0, True

        scan_range, collision = self._process_scan(scan_msg)
        state = self._compute_state(scan_range, collision)

        # Check goal reached
        if self.get_goal_distance() < self.goal_reached_dist:
            self.goal_reached = True

        reward = self._compute_reward(state, collision, action)
        done = collision or self.goal_reached

        if collision:
            self._pub_cmd.publish(Twist())  # stop robot

        if self.goal_reached:
            self._pub_cmd.publish(Twist())  # stop robot
            self.goal_x, self.goal_y = self.goal_manager.respawn(delete_first=True)
            self.goal_reached = False

        return np.asarray(state), reward, done

    def close(self):
        self._pub_cmd.publish(Twist())
