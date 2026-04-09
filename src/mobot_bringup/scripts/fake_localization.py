#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("fake_localization", anonymous=False)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = "map"
    transform.child_frame_id = "odom"

    # 这里明确对齐 TurtleBot3 官方建图/定位逻辑：
    # map 原点与 odom 原点完全重合，均视为机器人起始建图点的 (0, 0)。
    # Gazebo 中机器人仍然可以在任意物理坐标生成，但 map -> odom 不再叠加二次偏移。
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.0
    transform.transform.rotation.x = 0.0
    transform.transform.rotation.y = 0.0
    transform.transform.rotation.z = 0.0
    transform.transform.rotation.w = 1.0

    broadcaster.sendTransform(transform)
    rospy.loginfo("已发布静态 TF map -> odom: x=0.000, y=0.000, yaw=0.000")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
