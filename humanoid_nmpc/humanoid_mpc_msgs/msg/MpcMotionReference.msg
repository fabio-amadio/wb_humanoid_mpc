std_msgs/Header header

string[] joint_names
float64[] joint_pos
float64[] joint_vel

geometry_msgs/Pose root_pose_w
geometry_msgs/Twist root_twist_w

# JointRefAnchorRpMotionCommand layout:
# [joint_pos, joint_vel, base_linear_velocity_x_body, base_linear_velocity_y_body, base_yaw_rate_body, base_height_world, base_roll, base_pitch]
float32[] motion_cmd
