# Joint position command with optional velocity limit
string joint_name        # Name of joint to command (e.g., "x_rail_joint", "z_rail_joint")
float64 target_position  # Target position in radians or meters (depends on joint type)
float64 max_velocity     # Maximum velocity limit (rad/s or m/s). 0.0 = use controller default
