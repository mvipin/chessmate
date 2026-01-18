#!/usr/bin/env python3
"""
Unit Tests for Chess Logic

Tests the chess game logic including move parsing, validation, and game state.
These are pure unit tests with no hardware or ROS2 dependencies.

Test Coverage:
    - UCI move parsing
    - Algebraic notation conversion
    - Square coordinate mapping
    - Special move detection (castling, en passant, promotion)
"""

import pytest

# Mark all tests in this module as unit tests
pytestmark = pytest.mark.unit


class TestSquareCoordinates:
    """Test chess square coordinate conversions."""

    def algebraic_to_indices(self, square: str) -> tuple:
        """
        Convert algebraic notation to board indices.
        
        Args:
            square: Square in algebraic notation (e.g., 'e4')
            
        Returns:
            Tuple of (file_index, rank_index) where both are 0-7
        """
        if len(square) != 2:
            raise ValueError(f"Invalid square notation: {square}")
        
        file_char = square[0].lower()
        rank_char = square[1]
        
        if file_char < 'a' or file_char > 'h':
            raise ValueError(f"Invalid file: {file_char}")
        if rank_char < '1' or rank_char > '8':
            raise ValueError(f"Invalid rank: {rank_char}")
        
        file_idx = ord(file_char) - ord('a')  # 0-7
        rank_idx = int(rank_char) - 1          # 0-7
        
        return (file_idx, rank_idx)

    def indices_to_algebraic(self, file_idx: int, rank_idx: int) -> str:
        """
        Convert board indices to algebraic notation.
        
        Args:
            file_idx: File index (0-7, where 0='a')
            rank_idx: Rank index (0-7, where 0='1')
            
        Returns:
            Square in algebraic notation
        """
        if not (0 <= file_idx <= 7 and 0 <= rank_idx <= 7):
            raise ValueError(f"Invalid indices: ({file_idx}, {rank_idx})")
        
        file_char = chr(ord('a') + file_idx)
        rank_char = str(rank_idx + 1)
        
        return file_char + rank_char

    # =========================================================================
    # Algebraic to Indices Tests
    # =========================================================================

    def test_corner_squares(self, corner_squares):
        """Test conversion of corner squares."""
        expected = {
            'a1': (0, 0),
            'a8': (0, 7),
            'h1': (7, 0),
            'h8': (7, 7),
        }
        for square in corner_squares:
            result = self.algebraic_to_indices(square)
            assert result == expected[square], f"{square} -> {result} != {expected[square]}"

    def test_center_squares(self, center_squares):
        """Test conversion of center squares."""
        expected = {
            'd4': (3, 3),
            'd5': (3, 4),
            'e4': (4, 3),
            'e5': (4, 4),
        }
        for square in center_squares:
            result = self.algebraic_to_indices(square)
            assert result == expected[square], f"{square} -> {result} != {expected[square]}"

    def test_all_squares_roundtrip(self, all_chess_squares):
        """Test that all squares can be converted and back."""
        for square in all_chess_squares:
            file_idx, rank_idx = self.algebraic_to_indices(square)
            result = self.indices_to_algebraic(file_idx, rank_idx)
            assert result == square, f"Roundtrip failed: {square} -> ({file_idx}, {rank_idx}) -> {result}"

    def test_invalid_file_raises(self):
        """Test that invalid file raises ValueError."""
        with pytest.raises(ValueError, match="Invalid file"):
            self.algebraic_to_indices('i4')  # 'i' is not a valid file

    def test_invalid_rank_raises(self):
        """Test that invalid rank raises ValueError."""
        with pytest.raises(ValueError, match="Invalid rank"):
            self.algebraic_to_indices('e9')  # '9' is not a valid rank

    def test_invalid_notation_raises(self):
        """Test that malformed notation raises ValueError."""
        with pytest.raises(ValueError, match="Invalid square"):
            self.algebraic_to_indices('e')  # Too short
        with pytest.raises(ValueError, match="Invalid square"):
            self.algebraic_to_indices('e44')  # Too long


class TestUCIMoveParser:
    """Test UCI move format parsing."""

    def parse_uci_move(self, uci_move: str) -> dict:
        """
        Parse a UCI format move string.
        
        Args:
            uci_move: Move in UCI format (e.g., 'e2e4', 'e7e8q')
            
        Returns:
            Dictionary with from_square, to_square, and optional promotion
        """
        if len(uci_move) < 4 or len(uci_move) > 5:
            raise ValueError(f"Invalid UCI move format: {uci_move}")
        
        from_square = uci_move[0:2]
        to_square = uci_move[2:4]
        promotion = uci_move[4] if len(uci_move) == 5 else None
        
        # Validate squares
        for sq in [from_square, to_square]:
            if sq[0] < 'a' or sq[0] > 'h' or sq[1] < '1' or sq[1] > '8':
                raise ValueError(f"Invalid square in move: {sq}")
        
        # Validate promotion piece
        if promotion and promotion not in 'qrbn':
            raise ValueError(f"Invalid promotion piece: {promotion}")
        
        return {
            'from_square': from_square,
            'to_square': to_square,
            'promotion': promotion,
        }

    def test_normal_pawn_move(self):
        """Test parsing a normal pawn move."""
        result = self.parse_uci_move('e2e4')
        assert result['from_square'] == 'e2'
        assert result['to_square'] == 'e4'
        assert result['promotion'] is None

    def test_knight_move(self):
        """Test parsing a knight move."""
        result = self.parse_uci_move('g1f3')
        assert result['from_square'] == 'g1'
        assert result['to_square'] == 'f3'
        assert result['promotion'] is None

    def test_promotion_to_queen(self):
        """Test parsing pawn promotion to queen."""
        result = self.parse_uci_move('e7e8q')
        assert result['from_square'] == 'e7'
        assert result['to_square'] == 'e8'
        assert result['promotion'] == 'q'

    def test_promotion_to_knight(self):
        """Test parsing pawn promotion to knight (underpromotion)."""
        result = self.parse_uci_move('a7a8n')
        assert result['from_square'] == 'a7'
        assert result['to_square'] == 'a8'
        assert result['promotion'] == 'n'

    def test_all_promotion_pieces(self):
        """Test all valid promotion pieces."""
        for piece in 'qrbn':
            result = self.parse_uci_move(f'e7e8{piece}')
            assert result['promotion'] == piece

    def test_invalid_promotion_piece_raises(self):
        """Test that invalid promotion piece raises error."""
        with pytest.raises(ValueError, match="Invalid promotion"):
            self.parse_uci_move('e7e8k')  # Can't promote to king

    def test_invalid_move_format_raises(self):
        """Test that invalid move format raises error."""
        with pytest.raises(ValueError, match="Invalid UCI"):
            self.parse_uci_move('e2')  # Too short
        with pytest.raises(ValueError, match="Invalid UCI"):
            self.parse_uci_move('e2e4qq')  # Too long


class TestSpecialMoves:
    """Test detection and handling of special chess moves."""

    def is_castling(self, uci_move: str) -> str:
        """
        Detect if a move is castling.

        Args:
            uci_move: Move in UCI format

        Returns:
            'kingside', 'queenside', or None
        """
        castling_moves = {
            'e1g1': 'kingside',   # White kingside
            'e1c1': 'queenside',  # White queenside
            'e8g8': 'kingside',   # Black kingside
            'e8c8': 'queenside',  # Black queenside
        }
        return castling_moves.get(uci_move[:4])

    def is_en_passant(self, uci_move: str, en_passant_square: str) -> bool:
        """
        Detect if a move is en passant capture.

        Args:
            uci_move: Move in UCI format
            en_passant_square: Current en passant target square or None

        Returns:
            True if move is en passant
        """
        if not en_passant_square:
            return False

        to_square = uci_move[2:4]
        return to_square == en_passant_square

    def test_white_kingside_castling(self):
        """Test detection of white kingside castling."""
        assert self.is_castling('e1g1') == 'kingside'

    def test_white_queenside_castling(self):
        """Test detection of white queenside castling."""
        assert self.is_castling('e1c1') == 'queenside'

    def test_black_kingside_castling(self):
        """Test detection of black kingside castling."""
        assert self.is_castling('e8g8') == 'kingside'

    def test_black_queenside_castling(self):
        """Test detection of black queenside castling."""
        assert self.is_castling('e8c8') == 'queenside'

    def test_normal_king_move_not_castling(self):
        """Test that normal king moves are not detected as castling."""
        assert self.is_castling('e1e2') is None
        assert self.is_castling('e1f1') is None

    def test_en_passant_detection(self):
        """Test en passant move detection."""
        # White pawn on e5 captures black pawn that just moved d7-d5
        assert self.is_en_passant('e5d6', 'd6') is True

    def test_normal_capture_not_en_passant(self):
        """Test that normal captures are not detected as en passant."""
        assert self.is_en_passant('e5d6', None) is False
        assert self.is_en_passant('e5d6', 'c6') is False  # Wrong square


class TestMoveSequence:
    """Test move sequence handling for robot execution."""

    def get_move_sequence(self, uci_move: str, is_capture: bool = False,
                          castling: str = None, en_passant: bool = False) -> list:
        """
        Generate sequence of robot moves for a chess move.

        Args:
            uci_move: Move in UCI format
            is_capture: Whether this move captures a piece
            castling: 'kingside', 'queenside', or None
            en_passant: Whether this is an en passant capture

        Returns:
            List of (action, square) tuples
        """
        from_sq = uci_move[0:2]
        to_sq = uci_move[2:4]
        sequence = []

        if castling:
            # King move
            sequence.append(('pick', from_sq))
            sequence.append(('place', to_sq))
            # Rook move
            if castling == 'kingside':
                rook_from = 'h1' if from_sq[1] == '1' else 'h8'
                rook_to = 'f1' if from_sq[1] == '1' else 'f8'
            else:
                rook_from = 'a1' if from_sq[1] == '1' else 'a8'
                rook_to = 'd1' if from_sq[1] == '1' else 'd8'
            sequence.append(('pick', rook_from))
            sequence.append(('place', rook_to))
        elif en_passant:
            # Capture the pawn on the adjacent square
            captured_sq = to_sq[0] + from_sq[1]  # Same rank as moving pawn
            sequence.append(('pick', captured_sq))
            sequence.append(('place', 'graveyard'))
            sequence.append(('pick', from_sq))
            sequence.append(('place', to_sq))
        elif is_capture:
            sequence.append(('pick', to_sq))
            sequence.append(('place', 'graveyard'))
            sequence.append(('pick', from_sq))
            sequence.append(('place', to_sq))
        else:
            sequence.append(('pick', from_sq))
            sequence.append(('place', to_sq))

        return sequence

    def test_simple_move_sequence(self):
        """Test sequence for a simple non-capture move."""
        seq = self.get_move_sequence('e2e4')
        assert seq == [('pick', 'e2'), ('place', 'e4')]

    def test_capture_sequence(self):
        """Test sequence for a capture move."""
        seq = self.get_move_sequence('e4d5', is_capture=True)
        assert seq == [
            ('pick', 'd5'),
            ('place', 'graveyard'),
            ('pick', 'e4'),
            ('place', 'd5'),
        ]

    def test_kingside_castling_sequence(self):
        """Test sequence for kingside castling."""
        seq = self.get_move_sequence('e1g1', castling='kingside')
        assert seq == [
            ('pick', 'e1'),
            ('place', 'g1'),
            ('pick', 'h1'),
            ('place', 'f1'),
        ]

    def test_queenside_castling_sequence(self):
        """Test sequence for queenside castling."""
        seq = self.get_move_sequence('e1c1', castling='queenside')
        assert seq == [
            ('pick', 'e1'),
            ('place', 'c1'),
            ('pick', 'a1'),
            ('place', 'd1'),
        ]

    def test_en_passant_sequence(self):
        """Test sequence for en passant capture."""
        seq = self.get_move_sequence('e5d6', en_passant=True)
        assert seq == [
            ('pick', 'd5'),  # Captured pawn is on d5, not d6
            ('place', 'graveyard'),
            ('pick', 'e5'),
            ('place', 'd6'),
        ]

