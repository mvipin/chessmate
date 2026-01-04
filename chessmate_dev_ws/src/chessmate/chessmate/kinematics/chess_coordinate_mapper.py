#!/usr/bin/env python3
"""
Chess Board Coordinate Mapper for ChessMate
Converts chess square notation to robot coordinate system
"""

import math
from typing import Tuple, Dict, List, Optional
from dataclasses import dataclass


@dataclass
class ChessBoardConfiguration:
    """Chess board configuration parameters"""
    # Board position relative to robot base (meters)
    # Updated with real F360 measurements: 137mm from robot base + 116.67mm to center
    board_center_x: float = 0.25367  # 253.67mm forward from robot base (137mm + 116.67mm center offset)
    board_center_y: float = 0.000    # Centered on robot Y-axis
    board_height: float = 0.010      # Board surface height (1cm thickness)

    # Board dimensions - Updated with real F360 measurements
    square_size: float = 0.029166    # 29.166mm squares (233.33mm / 8 squares)
    board_size: float = 0.23333      # Total board size (233.33mm from F360)

    # Piece handling heights
    piece_pickup_height: float = 0.025   # Height to pick up pieces (2.5cm)
    piece_transport_height: float = 0.080 # Height for safe transport (8cm)
    piece_place_height: float = 0.025    # Height to place pieces (2.5cm)
    
    # Captured piece storage area
    capture_area_x: float = 0.150        # X position for captured pieces
    capture_area_y_white: float = -0.250 # Y position for white captured pieces
    capture_area_y_black: float = 0.250  # Y position for black captured pieces


class ChessBoardMapper:
    """
    Chess Board Coordinate Mapper
    
    Converts between chess notation (e.g., 'e4') and robot Cartesian coordinates.
    Handles standard chess board layout with a1 at bottom-left, h8 at top-right.
    
    Chess Board Layout:
    8 | a8 b8 c8 d8 e8 f8 g8 h8
    7 | a7 b7 c7 d7 e7 f7 g7 h7
    6 | a6 b6 c6 d6 e6 f6 g6 h6
    5 | a5 b5 c5 d5 e5 f5 g5 h5
    4 | a4 b4 c4 d4 e4 f4 g4 h4
    3 | a3 b3 c3 d3 e3 f3 g3 h3
    2 | a2 b2 c2 d2 e2 f2 g2 h2
    1 | a1 b1 c1 d1 e1 f1 g1 h1
      +------------------------
        a  b  c  d  e  f  g  h
    """
    
    def __init__(self, config: Optional[ChessBoardConfiguration] = None):
        """Initialize chess board mapper with configuration"""
        self.config = config or ChessBoardConfiguration()
        
        # Create lookup tables for fast conversion
        self._create_lookup_tables()
    
    def square_to_cartesian(self, square_name: str, height: Optional[float] = None) -> Tuple[float, float, float]:
        """
        Convert chess square notation to Cartesian coordinates
        
        Args:
            square_name: Chess square (e.g., 'e4', 'a1', 'h8')
            height: Z coordinate (defaults to board height)
            
        Returns:
            Tuple of (x, y, z) coordinates in robot frame
            
        Raises:
            ValueError: If square name is invalid
        """
        if not self.is_valid_square(square_name):
            raise ValueError(f"Invalid chess square: {square_name}")
        
        square_name = square_name.lower()
        file_char = square_name[0]  # a-h
        rank_char = square_name[1]  # 1-8
        
        # Convert file (a-h) to column index (0-7)
        file_index = ord(file_char) - ord('a')
        
        # Convert rank (1-8) to row index (0-7)
        rank_index = int(rank_char) - 1
        
        # Calculate position relative to board center
        # Files (a-h) map to Y coordinates (left to right from robot perspective)
        # Ranks (1-8) map to X coordinates (near to far from robot perspective)
        x_offset = (rank_index - 3.5) * self.config.square_size
        y_offset = (file_index - 3.5) * self.config.square_size
        
        # Convert to robot base coordinates
        x = self.config.board_center_x + x_offset
        y = self.config.board_center_y + y_offset
        z = height if height is not None else self.config.board_height
        
        return (x, y, z)
    
    def cartesian_to_square(self, x: float, y: float) -> Optional[str]:
        """
        Convert Cartesian coordinates to chess square notation
        
        Args:
            x, y: Cartesian coordinates
            
        Returns:
            Chess square name or None if outside board
        """
        # Calculate offsets from board center
        x_offset = x - self.config.board_center_x
        y_offset = y - self.config.board_center_y
        
        # Convert to square indices
        rank_index = round(x_offset / self.config.square_size + 3.5)
        file_index = round(y_offset / self.config.square_size + 3.5)
        
        # Check if within board bounds
        if not (0 <= rank_index <= 7 and 0 <= file_index <= 7):
            return None
        
        # Convert to chess notation
        file_char = chr(ord('a') + file_index)
        rank_char = str(rank_index + 1)
        
        return file_char + rank_char
    
    def get_pickup_position(self, square_name: str) -> Tuple[float, float, float]:
        """Get position for picking up a piece from a square"""
        x, y, _ = self.square_to_cartesian(square_name)
        return (x, y, self.config.piece_pickup_height)
    
    def get_place_position(self, square_name: str) -> Tuple[float, float, float]:
        """Get position for placing a piece on a square"""
        x, y, _ = self.square_to_cartesian(square_name)
        return (x, y, self.config.piece_place_height)
    
    def get_transport_position(self, square_name: str) -> Tuple[float, float, float]:
        """Get safe transport position above a square"""
        x, y, _ = self.square_to_cartesian(square_name)
        return (x, y, self.config.piece_transport_height)
    
    def get_capture_storage_position(self, piece_color: str, capture_index: int = 0) -> Tuple[float, float, float]:
        """
        Get position for storing captured pieces
        
        Args:
            piece_color: 'white' or 'black'
            capture_index: Index for multiple captured pieces
            
        Returns:
            Storage position coordinates
        """
        x = self.config.capture_area_x
        
        if piece_color.lower() == 'white':
            y = self.config.capture_area_y_white + (capture_index * self.config.square_size * 0.5)
        else:
            y = self.config.capture_area_y_black + (capture_index * self.config.square_size * 0.5)
        
        z = self.config.piece_place_height
        
        return (x, y, z)
    
    def get_all_squares(self) -> List[str]:
        """Get list of all chess squares"""
        squares = []
        for rank in range(1, 9):
            for file_char in 'abcdefgh':
                squares.append(f"{file_char}{rank}")
        return squares
    
    def get_square_neighbors(self, square_name: str) -> List[str]:
        """Get list of neighboring squares (for collision avoidance)"""
        if not self.is_valid_square(square_name):
            return []
        
        file_char = square_name[0]
        rank_num = int(square_name[1])
        
        neighbors = []
        
        # Check all 8 directions
        for df in [-1, 0, 1]:
            for dr in [-1, 0, 1]:
                if df == 0 and dr == 0:
                    continue  # Skip the square itself
                
                new_file = chr(ord(file_char) + df)
                new_rank = rank_num + dr
                
                if 'a' <= new_file <= 'h' and 1 <= new_rank <= 8:
                    neighbors.append(f"{new_file}{new_rank}")
        
        return neighbors
    
    def calculate_move_distance(self, from_square: str, to_square: str) -> float:
        """Calculate Euclidean distance between two squares"""
        from_pos = self.square_to_cartesian(from_square)
        to_pos = self.square_to_cartesian(to_square)
        
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        
        return math.sqrt(dx*dx + dy*dy)
    
    def is_valid_square(self, square_name: str) -> bool:
        """Check if square name is valid chess notation"""
        if not isinstance(square_name, str) or len(square_name) != 2:
            return False
        
        file_char = square_name[0].lower()
        rank_char = square_name[1]
        
        return ('a' <= file_char <= 'h') and ('1' <= rank_char <= '8')
    
    def _create_lookup_tables(self):
        """Create lookup tables for fast coordinate conversion"""
        self.square_to_coords = {}
        self.coords_to_square = {}
        
        for square in self.get_all_squares():
            coords = self.square_to_cartesian(square)
            self.square_to_coords[square] = coords
            # Round coordinates for reverse lookup
            rounded_coords = (round(coords[0], 3), round(coords[1], 3))
            self.coords_to_square[rounded_coords] = square
