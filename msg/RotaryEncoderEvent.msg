# Rotary encoder input event message
# Timestamp of the event
builtin_interfaces/Time timestamp

# Type of rotary encoder event
uint8 EVENT_TYPE_ROTATION = 0
uint8 EVENT_TYPE_BUTTON_PRESS = 1
uint8 EVENT_TYPE_BUTTON_RELEASE = 2

uint8 event_type

# For rotation events: direction (-1 for counter-clockwise, +1 for clockwise)
# For button events: 0 (not applicable)
int8 direction

# Button state (true if pressed, false if released)
bool button_pressed

# Raw encoder position (cumulative)
int32 encoder_position
