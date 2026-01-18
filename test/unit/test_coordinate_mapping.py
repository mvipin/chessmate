#!/usr/bin/env python3
"""
Unit Tests for Coordinate Mapping

Tests the mapping between chess squares and robot coordinates.
These are pure unit tests with no hardware or ROS2 dependencies.

Test Coverage:
    - Chess square to robot XY coordinate conversion
    - Robot coordinate to chess square conversion
    - Board orientation handling
    - Workspace validation
"""

import pytest
import math

# Mark all tests in this module as unit tests
pytestmark = pytest.mark.unit


class TestChessToRobotMapping:
    """Test mapping from chess squares to robot coordinates."""

    # Board configuration optimized for SCARA workspace
    # With L1=0.202m and L2=0.190m, max reach is 0.392m, min reach is 0.012m
    # Board must fit within reachable annulus
    # Using smaller squares and centered position to ensure all 64 squares reachable
    SQUARE_SIZE = 0.035  # 35mm squares (compact board for testing)
    BOARD_CENTER_X = 0.250  # Board center X from robot base
    BOARD_CENTER_Y = 0.000  # Board center Y (centered)
    
    def square_to_robot_coords(self, square: str) -> tuple:
        """
        Convert chess square to robot XY coordinates.
        
        The board is oriented with white on the robot's side:
        - a1 is at bottom-left (negative Y)
        - h8 is at top-right (positive Y)
        
        Args:
            square: Chess square in algebraic notation (e.g., 'e4')
            
        Returns:
            Tuple of (x, y) in meters
        """
        file_char = square[0].lower()
        rank_char = square[1]
        
        # Convert to 0-7 indices
        file_idx = ord(file_char) - ord('a')  # 0-7 (a-h)
        rank_idx = int(rank_char) - 1          # 0-7 (1-8)
        
        # Calculate offset from board center
        # Files a-h map to Y axis (a=-3.5 squares, h=+3.5 squares)
        # Ranks 1-8 map to X axis (1=closest to robot, 8=farthest)
        y_offset = (file_idx - 3.5) * self.SQUARE_SIZE
        x_offset = (rank_idx - 3.5) * self.SQUARE_SIZE
        
        x = self.BOARD_CENTER_X + x_offset
        y = self.BOARD_CENTER_Y + y_offset
        
        return (x, y)

    def robot_coords_to_square(self, x: float, y: float) -> str:
        """
        Convert robot XY coordinates to nearest chess square.
        
        Args:
            x: X coordinate in meters
            y: Y coordinate in meters
            
        Returns:
            Chess square in algebraic notation, or None if outside board
        """
        # Calculate offset from board center
        x_offset = x - self.BOARD_CENTER_X
        y_offset = y - self.BOARD_CENTER_Y
        
        # Convert to square indices
        rank_idx = round(x_offset / self.SQUARE_SIZE + 3.5)
        file_idx = round(y_offset / self.SQUARE_SIZE + 3.5)
        
        # Validate indices
        if not (0 <= file_idx <= 7 and 0 <= rank_idx <= 7):
            return None
        
        file_char = chr(ord('a') + file_idx)
        rank_char = str(rank_idx + 1)
        
        return file_char + rank_char

    # =========================================================================
    # Square to Robot Coordinate Tests
    # =========================================================================

    def test_board_center_d4_d5(self):
        """Test that d4/d5 are near board center."""
        x_d4, y_d4 = self.square_to_robot_coords('d4')
        x_d5, y_d5 = self.square_to_robot_coords('d5')
        
        # d4 and d5 should be close to board center Y
        assert abs(y_d4) < self.SQUARE_SIZE, f"d4 Y={y_d4} not near center"
        assert abs(y_d5) < self.SQUARE_SIZE, f"d5 Y={y_d5} not near center"

    def test_corner_squares_positions(self, corner_squares):
        """Test that corner squares are at expected positions."""
        coords = {sq: self.square_to_robot_coords(sq) for sq in corner_squares}
        
        # a1 should be at minimum X and minimum Y
        # h8 should be at maximum X and maximum Y
        assert coords['a1'][0] < coords['a8'][0], "a1 should be closer to robot than a8"
        assert coords['a1'][1] < coords['h1'][1], "a1 should be left of h1"

    def test_adjacent_squares_spacing(self):
        """Test that adjacent squares are exactly SQUARE_SIZE apart."""
        x_e4, y_e4 = self.square_to_robot_coords('e4')
        x_e5, y_e5 = self.square_to_robot_coords('e5')
        x_f4, y_f4 = self.square_to_robot_coords('f4')
        
        # e4 to e5 should differ only in X by SQUARE_SIZE
        assert abs(x_e5 - x_e4 - self.SQUARE_SIZE) < 1e-6
        assert abs(y_e5 - y_e4) < 1e-6
        
        # e4 to f4 should differ only in Y by SQUARE_SIZE
        assert abs(x_f4 - x_e4) < 1e-6
        assert abs(y_f4 - y_e4 - self.SQUARE_SIZE) < 1e-6

    def test_all_squares_in_workspace(self, all_chess_squares, scara_config_meters):
        """Test that all chess squares are within robot workspace."""
        l1 = scara_config_meters['l1']
        l2 = scara_config_meters['l2']
        max_reach = l1 + l2
        min_reach = abs(l1 - l2)

        # Small tolerance for floating point precision
        tolerance = 0.002  # 2mm tolerance

        for square in all_chess_squares:
            x, y = self.square_to_robot_coords(square)
            distance = math.sqrt(x**2 + y**2)

            assert distance <= max_reach + tolerance, \
                f"Square {square} at ({x:.3f}, {y:.3f}) beyond max reach {max_reach:.3f}"
            assert distance >= min_reach - tolerance, \
                f"Square {square} at ({x:.3f}, {y:.3f}) inside min reach {min_reach:.3f}"

    # =========================================================================
    # Robot Coordinate to Square Tests
    # =========================================================================

    def test_roundtrip_all_squares(self, all_chess_squares):
        """Test that all squares survive coordinate roundtrip."""
        for square in all_chess_squares:
            x, y = self.square_to_robot_coords(square)
            result = self.robot_coords_to_square(x, y)
            assert result == square, f"Roundtrip failed: {square} -> ({x}, {y}) -> {result}"

    def test_coords_outside_board_returns_none(self):
        """Test that coordinates outside board return None."""
        # Way outside the board
        assert self.robot_coords_to_square(0.0, 0.0) is None
        assert self.robot_coords_to_square(1.0, 0.0) is None

    def test_coords_near_square_center(self):
        """Test that coordinates near square center map correctly."""
        x, y = self.square_to_robot_coords('e4')
        
        # Small offset should still map to e4
        assert self.robot_coords_to_square(x + 0.01, y + 0.01) == 'e4'
        assert self.robot_coords_to_square(x - 0.01, y - 0.01) == 'e4'

