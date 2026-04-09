#!/usr/bin/env python3

import sys

import rospy


def main():
    """根据 map 名称读取嵌套 YAML，并展开为简单的全局参数。"""
    rospy.init_node("set_init_pose", anonymous=False)

    map_name = rospy.get_param("~map_name", "").strip()
    if not map_name:
        rospy.logerr("未提供 ~map_name，无法设置初始位姿。")
        return 1

    pose_param_name = "/initial_poses/{}".format(map_name)
    if not rospy.has_param(pose_param_name):
        rospy.logerr("参数服务器中不存在 %s，请检查 initial_poses.yaml 是否包含该场景。", pose_param_name)
        return 1

    pose = rospy.get_param(pose_param_name)
    for key in ("x", "y", "a"):
        if key not in pose:
            rospy.logerr("%s 缺少字段 %s，无法生成完整的初始位姿。", pose_param_name, key)
            return 1

    # 这里故意写成简单的全局参数，供后续脚本直接读取。
    # 这样做的原因是 ROS1 XML Launch 无法直接从嵌套 YAML key 中取值后再传给 spawn_model。
    rospy.set_param("/initial_pose_x", float(pose["x"]))
    rospy.set_param("/initial_pose_y", float(pose["y"]))
    rospy.set_param("/initial_pose_yaw", float(pose["a"]))

    rospy.loginfo(
        "已为场景 %s 设置初始位姿: x=%.3f, y=%.3f, yaw=%.3f",
        map_name,
        float(pose["x"]),
        float(pose["y"]),
        float(pose["a"]),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
