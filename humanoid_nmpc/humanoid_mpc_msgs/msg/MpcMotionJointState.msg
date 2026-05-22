std_msgs/Header header

# Compact motion command with joint velocities:
# [joint_pos, joint_vel, base_linear_velocity_x_body, base_linear_velocity_y_body,
#  base_yaw_rate_body, base_height_world, base_roll, base_pitch]
float32[] motion_cmd
