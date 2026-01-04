# GripperCommand.msg - End effector control
string action        # "open", "close", "grip_piece", "release_piece"
float32 force_limit  # Maximum grip force (Newtons, 0.0 = no limit)
float32 timeout      # Command timeout (seconds)
bool gentle_mode     # True for delicate piece handling
