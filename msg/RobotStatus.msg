# RobotStatus.msg - System health monitoring
string state           # "idle", "moving", "gripping", "error", "calibrating"
geometry_msgs/Pose current_pose
sensor_msgs/JointState joint_states
string[] active_warnings
string[] error_messages
float32 battery_voltage  # For future wireless operation (0.0 if N/A)
bool gripper_has_piece   # True if gripper is holding a piece
int64 timestamp          # Status timestamp
