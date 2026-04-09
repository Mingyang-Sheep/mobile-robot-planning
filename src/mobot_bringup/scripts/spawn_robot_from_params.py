#!/usr/bin/env python3

import math
import sys
import time

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def wait_for_param(param_name, timeout_sec):
    """等待参数出现，避免与 set_init_pose.py 并发启动时发生竞争。"""
    deadline = time.time() + timeout_sec
    while not rospy.is_shutdown():
        if rospy.has_param(param_name):
            return rospy.get_param(param_name)
        if time.time() > deadline:
            raise rospy.ROSException("等待参数 {} 超时".format(param_name))
        time.sleep(0.1)


def main():
    rospy.init_node("spawn_robot_from_params", anonymous=False)

    robot_name = rospy.get_param("~robot", "waffle_pi")
    spawn_z = float(rospy.get_param("~z", 0.0))
    timeout_sec = float(rospy.get_param("~timeout", 30.0))
    reference_frame = rospy.get_param("~reference_frame", "world")

    # 这里读取 set_init_pose.py 写入的全局参数。
    # 这样 launch 只需要传 map 名称，就能自动得到对应初始位姿。
    initial_x = float(wait_for_param("/initial_pose_x", timeout_sec))
    initial_y = float(wait_for_param("/initial_pose_y", timeout_sec))
    initial_yaw = float(wait_for_param("/initial_pose_yaw", timeout_sec))

    robot_description = wait_for_param("/robot_description", timeout_sec)

    rospy.loginfo("等待 Gazebo /gazebo/spawn_urdf_model 服务...")
    rospy.wait_for_service("/gazebo/spawn_urdf_model", timeout=timeout_sec)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)

    pose = Pose()
    pose.position.x = initial_x
    pose.position.y = initial_y
    pose.position.z = spawn_z

    half_yaw = initial_yaw / 2.0
    pose.orientation.z = math.sin(half_yaw)
    pose.orientation.w = math.cos(half_yaw)

    # 直接调用 Gazebo 服务生成机器人，避免 ROS1 XML Launch 无法把运行时参数重新注入命令行的问题。
    response = spawn_model(robot_name, robot_description, "", pose, reference_frame)
    if not response.success:
        rospy.logerr("Gazebo 生成机器人失败: %s", response.status_message)
        return 1

    rospy.loginfo(
        "已生成机器人 %s，位姿为 x=%.3f, y=%.3f, z=%.3f, yaw=%.3f",
        robot_name,
        initial_x,
        initial_y,
        spawn_z,
        initial_yaw,
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except rospy.ROSException as exc:
        rospy.logerr(str(exc))
        sys.exit(1)
