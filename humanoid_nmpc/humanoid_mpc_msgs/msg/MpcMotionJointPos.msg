std_msgs/Header header

# Compact motion command without joint velocities:
# [joint_pos, base_linear_velocity_x_body, base_linear_velocity_y_body,
#  base_yaw_rate_body, base_height_world, base_roll, base_pitch]
float32[] motion_cmd
