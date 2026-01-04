# Arduino communication command message
# Timestamp of the command
builtin_interfaces/Time timestamp

# Command types for Arduino communication
uint8 CMD_INIT = 0
uint8 CMD_OCCUPANCY = 1
uint8 CMD_LEGAL_MOVES = 2
uint8 CMD_HINT = 3
uint8 CMD_CHECK = 4
uint8 CMD_START_MOVE = 5
uint8 CMD_COMPUTER_MOVE = 6
uint8 CMD_OVERRIDE_MOVE = 7
uint8 CMD_RESET = 8
uint8 CMD_CHECKMATE = 9
uint8 CMD_HEARTBEAT = 10
uint8 CMD_STATUS = 11
uint8 CMD_CALIBRATE = 12
uint8 CMD_LED_ON = 13
uint8 CMD_LED_OFF = 14
uint8 CMD_HINT_ENABLE = 15
uint8 CMD_HINT_DISABLE = 16

# Robot animation commands (sent directly to robot controller)
uint8 CMD_ROBOT_WAKE_UP = 20
uint8 CMD_ROBOT_DOZE_OFF = 21
uint8 CMD_ROBOT_THINK_HARD = 22
uint8 CMD_ROBOT_RESET_POSE = 23
uint8 CMD_ROBOT_HOME_Z = 24
uint8 CMD_ROBOT_HOME_ALL = 25
uint8 CMD_ROBOT_EXECUTE_MOVE = 26

# Game flow indication commands (replaces single-character indications)
uint8 CMD_GAME_COMPUTER_TURN = 30
uint8 CMD_GAME_HUMAN_TURN = 31
uint8 CMD_GAME_MOVE_INVALID = 32
uint8 CMD_GAME_CHECKMATE = 33

uint8 command_type

# Command payload data
string data

# Target Arduino (0=chessboard controller, 1=robot controller)
uint8 target_arduino
