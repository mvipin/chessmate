# ChessMove.msg - Standardized chess move representation
string from_square     # e.g., "e2"
string to_square       # e.g., "e4"  
string piece_type      # "pawn", "rook", "knight", "bishop", "queen", "king"
string move_type       # "normal", "castle", "en_passant", "promotion"
string promotion_piece # For pawn promotion (optional, empty if not promotion)
bool is_capture        # True if move captures opponent piece
float32 confidence     # AI confidence in move (0.0-1.0)
int64 timestamp        # Move timestamp (nanoseconds)
