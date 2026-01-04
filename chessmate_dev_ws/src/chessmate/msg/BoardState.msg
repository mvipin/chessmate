# BoardState.msg - Complete chess board representation
ChessPiece[64] squares    # Array representing 8x8 board (a1=0, b1=1, ..., h8=63)
string active_color       # "white" or "black"
string castling_rights    # "KQkq" format (K=white king, Q=white queen, k=black king, q=black queen)
string en_passant_target  # Square name or "-" if no en passant possible
int32 halfmove_clock     # Moves since last capture/pawn move (for 50-move rule)
int32 fullmove_number    # Game move counter
string fen_string        # Full FEN representation for validation
int64 timestamp          # Board state timestamp
