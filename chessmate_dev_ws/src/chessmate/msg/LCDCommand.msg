# LCD display command message
# Timestamp of the command
builtin_interfaces/Time timestamp

# Command types
uint8 CMD_CLEAR = 0
uint8 CMD_DISPLAY_TEXT = 1
uint8 CMD_DISPLAY_MENU = 2
uint8 CMD_DISPLAY_GAME_STATUS = 3
uint8 CMD_SET_BRIGHTNESS = 4

uint8 command_type

# Text content to display (for text/status commands)
string text

# Menu options (for menu display)
string[] menu_options
int32 selected_index
int32 menu_level

# Display parameters
uint8 font_size  # 0=small, 1=large
uint8 x_position
uint8 y_position
bool invert_display
uint8 brightness  # 0-255
