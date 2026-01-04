# ChessPiece.msg - Individual piece representation
string color      # "white", "black", or "empty"
string type       # "pawn", "rook", "knight", "bishop", "queen", "king", or "none"
bool has_moved    # For castling/en passant logic
int32 square_id   # 0-63 representing a1-h8 (optional, for board representation)
