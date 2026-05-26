#!/usr/bin/env python3
"""Publish odom and TF for a Gazebo model using /gazebo/model_states."""

import math

import rospy
import tf
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry


class GazeboModelOdom:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "go2w_gazebo")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.odom_topic = rospy.get_param("~odom_topic", "odom")
        self.publish_tf = rospy.get_param("~publish_tf", True)
        self.update_rate = float(rospy.get_param("~update_rate", 30.0))
        self._last_publish = rospy.Time(0)

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb, queue_size=1)

    def _model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            rospy.logwarn_throttle(5.0, "Gazebo model '%s' not found in /gazebo/model_states", self.model_name)
            return

        now = rospy.Time.now()
        if self.update_rate > 0.0 and (now - self._last_publish).to_sec() < 1.0 / self.update_rate:
            return
        self._last_publish = now

        pose = msg.pose[index]
        twist = msg.twist[index]
        _, _, yaw = tf.transformations.euler_from_quaternion(
            (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        )
        planar_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        body_vx = cos_yaw * twist.linear.x + sin_yaw * twist.linear.y
        body_vy = -sin_yaw * twist.linear.x + cos_yaw * twist.linear.y

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = pose.position.x
        odom.pose.pose.position.y = pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = planar_quat[0]
        odom.pose.pose.orientation.y = planar_quat[1]
        odom.pose.pose.orientation.z = planar_quat[2]
        odom.pose.pose.orientation.w = planar_quat[3]
        odom.twist.twist.linear.x = body_vx
        odom.twist.twist.linear.y = body_vy
        odom.twist.twist.angular.z = twist.angular.z

        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[14] = 0.05
        odom.pose.covariance[21] = 0.05
        odom.pose.covariance[28] = 0.05
        odom.pose.covariance[35] = math.radians(2.0) ** 2
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = math.radians(2.0) ** 2

        self.odom_pub.publish(odom)

        if self.publish_tf:
            position = pose.position
            self.tf_broadcaster.sendTransform(
                (position.x, position.y, 0.0),
                tuple(planar_quat),
                now,
                self.base_frame,
                self.odom_frame,
            )


if __name__ == "__main__":
    rospy.init_node("gazebo_model_odom")
    GazeboModelOdom()
    rospy.spin()
