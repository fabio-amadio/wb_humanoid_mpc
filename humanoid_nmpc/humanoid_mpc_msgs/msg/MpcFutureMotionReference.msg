std_msgs/Header header

# Step offsets in units of dt. The first entry may be 0 for the current step.
int32[] steps

# Step duration used to convert steps into seconds.
float32 dt

# Step-major compact JointRefAnchorRpMotionCommand layout:
# [step0, step1, ...], each step encoded as
# [joint_pos, joint_vel, base_linear_velocity_x_body, base_linear_velocity_y_body,
#  base_yaw_rate_body, base_height_world, base_roll, base_pitch]
float32[] motion_cmd
