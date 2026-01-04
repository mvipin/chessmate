# Joint command message for SCARA arm control
std_msgs/Header header

# Joint names (should match URDF)
string[] joint_names

# Target joint positions (radians for revolute, meters for prismatic)
float64[] positions

# Target joint velocities (rad/s or m/s)
float64[] velocities

# Target joint accelerations (rad/s² or m/s²)
float64[] accelerations

# Joint effort/torque limits
float64[] efforts

# Execution parameters
float32 execution_time
bool wait_for_completion

# Safety parameters
bool enable_collision_checking
float32 velocity_scaling
float32 acceleration_scaling

# Command type
uint8 POSITION_CONTROL = 0
uint8 VELOCITY_CONTROL = 1
uint8 EFFORT_CONTROL = 2
uint8 control_mode
