# Robot animation command message
# Timestamp of the command
builtin_interfaces/Time timestamp

# Animation types
uint8 RESET_POSE = 0
uint8 DOZE_OFF = 1
uint8 WAKE_UP = 2
uint8 THINK_HARD = 3

# Robot action types
uint8 HOME_Z = 10
uint8 HOME_ALL = 11
uint8 EXECUTE_MOVE = 12

uint8 animation_type

# Animation/action data (e.g., move string for EXECUTE_MOVE)
string data
