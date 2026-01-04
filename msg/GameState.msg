# GameState.msg
# Represents the current state of a chess game

# Game status constants
uint8 STATUS_WAITING = 0
uint8 STATUS_PLAYING = 1
uint8 STATUS_PAUSED = 2
uint8 STATUS_FINISHED = 3
uint8 STATUS_ERROR = 4

# Game result constants
uint8 RESULT_NONE = 0
uint8 RESULT_WHITE_WINS = 1
uint8 RESULT_BLACK_WINS = 2
uint8 RESULT_DRAW = 3
uint8 RESULT_ABORTED = 4

# Current game status
uint8 status

# Game result (only valid when status is FINISHED)
uint8 result

# Current player to move (true = white, false = black)
bool white_to_move

# Move number (increments after black's move)
uint32 move_number

# Half-move clock (for 50-move rule)
uint32 halfmove_clock

# FEN (Forsyth-Edwards Notation) string representing current position
string fen

# Last move played (empty if no moves yet)
ChessMove last_move

# Game start time
builtin_interfaces/Time game_start_time

# Time remaining for each player (in seconds)
float64 white_time_remaining
float64 black_time_remaining

# Whether the game is using time control
bool time_control_enabled

# Additional game information
string white_player
string black_player
string event_name
string opening_name
