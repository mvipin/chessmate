#!/usr/bin/env python3
"""
ChessMate Test Configuration

Shared pytest fixtures and configuration for all test types.
"""

import pytest
import sys
import os

# Add parent package to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


@pytest.fixture
def scara_config():
    """Default SCARA robot configuration for testing."""
    return {
        'link1_length': 228.0,  # mm
        'link2_length': 136.5,  # mm
        'z_stroke': 50.0,       # mm
        'base_offset_x': 0.0,
        'base_offset_y': 0.0,
    }


@pytest.fixture
def chessboard_config():
    """Default chessboard configuration for testing."""
    return {
        'square_size': 57.0,      # mm
        'board_origin_x': 180.0,  # mm from robot base
        'board_origin_y': -228.0, # mm from robot base
        'piece_height': 45.0,     # mm
        'lift_height': 20.0,      # mm above piece
    }


@pytest.fixture
def mock_stockfish_path():
    """Path to Stockfish engine for testing."""
    paths = [
        '/usr/games/stockfish',
        '/usr/bin/stockfish',
        '/usr/local/bin/stockfish',
    ]
    for path in paths:
        if os.path.exists(path):
            return path
    pytest.skip("Stockfish not found")


@pytest.fixture
def sample_chess_moves():
    """Sample chess moves for testing."""
    return [
        ('e2', 'e4'),  # King's pawn opening
        ('e7', 'e5'),  # Response
        ('g1', 'f3'),  # Knight development
        ('b8', 'c6'),  # Knight response
        ('f1', 'c4'),  # Bishop development (Italian Game)
    ]


@pytest.fixture
def sample_fen_positions():
    """Sample FEN positions for testing."""
    return {
        'starting': 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1',
        'italian_game': 'r1bqkbnr/pppp1ppp/2n5/4p3/2B1P3/5N2/PPPP1PPP/RNBQK2R b KQkq - 3 3',
        'scholars_mate': 'r1bqkb1r/pppp1Qpp/2n2n2/4p3/2B1P3/8/PPPP1PPP/RNB1K1NR b KQkq - 0 4',
        'endgame': '8/8/4k3/8/8/4K3/4P3/8 w - - 0 1',
    }

