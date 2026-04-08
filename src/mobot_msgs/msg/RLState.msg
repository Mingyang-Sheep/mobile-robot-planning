std_msgs/Header header

# 雷达/激光离散距离（按环境生成，长度在不同环境中可能不同）
float32[] radar_ranges

# 目标相对位姿（机器人坐标系下）
float32 target_rel_x
float32 target_rel_y
float32 target_rel_yaw

# 当前速度（线速度/角速度）
geometry_msgs/Twist current_velocity

