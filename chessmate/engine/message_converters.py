#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Message Converters
Utilities for converting between ROS2 messages and chess formats
"""

import chess
from typing import Optional, Dict, List
from chessmate.msg import ChessMove, BoardState, ChessPiece


class MessageConverter:
    """
    Converts between ROS2 messages and standard chess formats
    """
    
    # Piece type mappings
    PIECE_TYPE_MAP = {
        chess.PAWN: "pawn",
        chess.ROOK: "rook", 
        chess.KNIGHT: "knight",
        chess.BISHOP: "bishop",
        chess.QUEEN: "queen",
        chess.KING: "king"
    }
    
    REVERSE_PIECE_MAP = {v: k for k, v in PIECE_TYPE_MAP.items()}
    
    def uci_to_chess_move(self, uci_move: str, board_state: BoardState) -> ChessMove:
        """
        Convert UCI move string to ChessMove message
        
        Args:
            uci_move: Move in UCI format (e.g., "e2e4", "e7e8q")
            board_state: Current board state for context
            
        Returns:
            ChessMove message
        """
        move = ChessMove()
        
        # Parse UCI move
        move.from_square = uci_move[:2]
        move.to_square = uci_move[2:4]
        
        # Handle promotion
        if len(uci_move) == 5:
            promotion_char = uci_move[4].lower()
            promotion_map = {'q': 'queen', 'r': 'rook', 'b': 'bishop', 'n': 'knight'}
            move.promotion_piece = promotion_map.get(promotion_char, '')
            move.move_type = "promotion"
        else:
            move.promotion_piece = ""
            move.move_type = "normal"
        
        # Determine piece type from board state
        from_index = self.square_to_index(move.from_square)
        if 0 <= from_index < 64 and len(board_state.squares) == 64:
            piece = board_state.squares[from_index]
            move.piece_type = piece.piece_type
        else:
            # Fallback - try to determine from move pattern
            move.piece_type = self.infer_piece_type(uci_move)
        
        # Check for special moves
        move.move_type = self.determine_move_type(uci_move, board_state)
        
        # Check if capture (simplified - would need full board analysis for accuracy)
        to_index = self.square_to_index(move.to_square)
        if 0 <= to_index < 64 and len(board_state.squares) == 64:
            target_piece = board_state.squares[to_index]
            move.is_capture = target_piece.piece_type != "empty"
        else:
            move.is_capture = False
        
        return move
    
    def chess_move_to_uci(self, chess_move: ChessMove) -> str:
        """
        Convert ChessMove message to UCI string
        
        Args:
            chess_move: ChessMove message
            
        Returns:
            UCI move string
        """
        uci = chess_move.from_square + chess_move.to_square
        
        # Add promotion piece if applicable
        if chess_move.move_type == "promotion" and chess_move.promotion_piece:
            promotion_map = {'queen': 'q', 'rook': 'r', 'bishop': 'b', 'knight': 'n'}
            promotion_char = promotion_map.get(chess_move.promotion_piece.lower(), 'q')
            uci += promotion_char
        
        return uci
    
    def board_state_to_fen(self, board_state: BoardState) -> str:
        """
        Convert BoardState message to FEN string
        
        Args:
            board_state: BoardState message
            
        Returns:
            FEN string
        """
        # If FEN is already provided, use it
        if board_state.fen_string:
            return board_state.fen_string
        
        # Otherwise construct from board array
        fen_parts = []
        
        # 1. Piece placement
        board_fen = ""
        for rank in range(7, -1, -1):  # 8th rank to 1st rank
            empty_count = 0
            rank_str = ""
            
            for file in range(8):  # a-h files
                index = rank * 8 + file
                if index < len(board_state.squares):
                    piece = board_state.squares[index]
                    
                    if piece.piece_type == "empty":
                        empty_count += 1
                    else:
                        if empty_count > 0:
                            rank_str += str(empty_count)
                            empty_count = 0
                        
                        # Convert piece to FEN notation
                        piece_char = self.piece_to_fen_char(piece)
                        rank_str += piece_char
                else:
                    empty_count += 1
            
            if empty_count > 0:
                rank_str += str(empty_count)
            
            board_fen += rank_str
            if rank > 0:
                board_fen += "/"
        
        fen_parts.append(board_fen)
        
        # 2. Active color
        fen_parts.append(board_state.active_color[0])  # "w" or "b"
        
        # 3. Castling rights
        fen_parts.append(board_state.castling_rights or "-")
        
        # 4. En passant target
        fen_parts.append(board_state.en_passant_target or "-")
        
        # 5. Halfmove clock
        fen_parts.append(str(board_state.halfmove_clock))
        
        # 6. Fullmove number
        fen_parts.append(str(board_state.fullmove_number))
        
        return " ".join(fen_parts)
    
    def fen_to_board_state(self, fen_string: str) -> BoardState:
        """
        Convert FEN string to BoardState message

        Args:
            fen_string: FEN notation string

        Returns:
            BoardState message
        """
        board_state = BoardState()
        board_state.fen_string = fen_string
        board_state.timestamp = 0  # Will be set by caller if needed

        try:
            # Parse FEN
            board = chess.Board(fen_string)

            # Initialize squares array
            board_state.squares = []

            # Fill board squares
            for square in chess.SQUARES:
                piece = board.piece_at(square)
                chess_piece = ChessPiece()

                if piece:
                    chess_piece.piece_type = self.PIECE_TYPE_MAP.get(piece.piece_type, "empty")
                    chess_piece.color = "white" if piece.color == chess.WHITE else "black"
                else:
                    chess_piece.piece_type = "empty"
                    chess_piece.color = ""

                board_state.squares.append(chess_piece)

            # Set other fields
            board_state.active_color = "white" if board.turn == chess.WHITE else "black"
            board_state.castling_rights = self.castling_rights_to_string(board)
            board_state.en_passant_target = chess.square_name(board.ep_square) if board.ep_square else "-"
            board_state.halfmove_clock = board.halfmove_clock
            board_state.fullmove_number = board.fullmove_number

        except Exception as e:
            print(f"Error parsing FEN: {e}")
            # Return empty board state on error
            board_state.squares = []
            for _ in range(64):
                piece = ChessPiece()
                piece.piece_type = "empty"
                piece.color = ""
                board_state.squares.append(piece)

        return board_state
    
    def square_to_index(self, square_name: str) -> int:
        """Convert square name (e.g., 'e4') to array index (0-63)"""
        if len(square_name) != 2:
            return -1
        
        file = ord(square_name[0].lower()) - ord('a')  # 0-7
        rank = int(square_name[1]) - 1  # 0-7
        
        if 0 <= file <= 7 and 0 <= rank <= 7:
            return rank * 8 + file
        return -1
    
    def index_to_square(self, index: int) -> str:
        """Convert array index (0-63) to square name"""
        if not 0 <= index <= 63:
            return ""
        
        file = chr(ord('a') + (index % 8))
        rank = str((index // 8) + 1)
        return file + rank
    
    def piece_to_fen_char(self, piece: ChessPiece) -> str:
        """Convert ChessPiece to FEN character"""
        if piece.piece_type == "empty":
            return ""
        
        char_map = {
            "pawn": "p", "rook": "r", "knight": "n",
            "bishop": "b", "queen": "q", "king": "k"
        }
        
        char = char_map.get(piece.piece_type.lower(), "")
        if piece.color == "white":
            char = char.upper()
        
        return char
    
    def castling_rights_to_string(self, board: chess.Board) -> str:
        """Convert castling rights to string format"""
        rights = ""
        if board.has_kingside_castling_rights(chess.WHITE):
            rights += "K"
        if board.has_queenside_castling_rights(chess.WHITE):
            rights += "Q"
        if board.has_kingside_castling_rights(chess.BLACK):
            rights += "k"
        if board.has_queenside_castling_rights(chess.BLACK):
            rights += "q"
        
        return rights if rights else "-"
    
    def infer_piece_type(self, uci_move: str) -> str:
        """Infer piece type from move pattern (fallback method)"""
        # This is a simplified inference - in practice, you'd need board state
        from_square = uci_move[:2]
        to_square = uci_move[2:4]
        
        # Check for pawn moves
        from_file = from_square[0]
        to_file = to_square[0]
        from_rank = int(from_square[1])
        to_rank = int(to_square[1])
        
        # Pawn moves forward or diagonally one square (or two from starting position)
        if from_file == to_file and abs(to_rank - from_rank) in [1, 2]:
            return "pawn"
        elif abs(ord(to_file) - ord(from_file)) == 1 and abs(to_rank - from_rank) == 1:
            return "pawn"  # Diagonal capture
        
        # Default to pawn if can't determine
        return "pawn"
    
    def determine_move_type(self, uci_move: str, board_state: BoardState) -> str:
        """Determine special move types"""
        # Check for promotion
        if len(uci_move) == 5:
            return "promotion"
        
        # Check for castling (simplified)
        if uci_move in ["e1g1", "e1c1", "e8g8", "e8c8"]:
            return "castle"
        
        # Check for en passant (would need more sophisticated logic)
        # For now, default to normal
        return "normal"
