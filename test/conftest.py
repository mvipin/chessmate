#!/usr/bin/env python3
"""
ChessMate Test Configuration

Shared pytest fixtures and configuration for all test types.

Test Categories (markers):
    - unit: Pure unit tests (no hardware, no ROS2)
    - integration: Tests requiring ROS2 but not hardware
    - hardware: Tests requiring physical hardware
    - slow: Long-running tests (>10 seconds)
    - ros2: Tests requiring ROS2 runtime
"""

import pytest
import sys
import os

# Add parent package to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


def pytest_configure(config):
    """Register custom markers for test categorization."""
    config.addinivalue_line(
        "markers", "unit: Pure unit tests (no hardware, no ROS2 required)"
    )
    config.addinivalue_line(
        "markers", "integration: Integration tests (may require ROS2)"
    )
    config.addinivalue_line(
        "markers", "hardware: Tests requiring physical hardware"
    )
    config.addinivalue_line(
        "markers", "requires_hardware: Tests that must have hardware connected"
    )
    config.addinivalue_line(
        "markers", "slow: Long-running tests (>10 seconds)"
    )
    config.addinivalue_line(
        "markers", "ros2: Tests requiring ROS2 runtime"
    )


# =============================================================================
# SCARA Robot Configuration Fixtures
# =============================================================================

@pytest.fixture
def scara_config():
    """Default SCARA robot configuration for testing (in mm)."""
    return {
        'link1_length': 228.0,  # mm
        'link2_length': 136.5,  # mm
        'z_stroke': 50.0,       # mm
        'base_offset_x': 0.0,
        'base_offset_y': 0.0,
    }


@pytest.fixture
def scara_config_meters():
    """SCARA robot configuration in meters (matches hardware config)."""
    return {
        'l1': 0.202,  # meters - First link length
        'l2': 0.190,  # meters - Second link length
        'z_min': 0.0,
        'z_max': 0.100,
        'theta1_min': -3.14159,
        'theta1_max': 3.14159,
        'theta2_min': -3.14159,
        'theta2_max': 3.14159,
    }


# =============================================================================
# Chessboard Configuration Fixtures
# =============================================================================

@pytest.fixture
def chessboard_config():
    """Default chessboard configuration for testing (in mm)."""
    return {
        'square_size': 57.0,      # mm
        'board_origin_x': 180.0,  # mm from robot base
        'board_origin_y': -228.0, # mm from robot base
        'piece_height': 45.0,     # mm
        'lift_height': 20.0,      # mm above piece
    }


@pytest.fixture
def chessboard_config_meters():
    """Chessboard configuration in meters (matches scara_config.yaml)."""
    return {
        'square_size': 0.050,      # meters (5cm squares)
        'board_center_x': 0.300,   # meters from robot base
        'board_center_y': 0.000,   # meters (centered)
        'board_height': 0.020,     # meters
        'board_size': 0.400,       # meters (8 * 0.050)
    }


# =============================================================================
# Chess Game Fixtures
# =============================================================================

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
def sample_uci_moves():
    """Sample UCI format moves for testing."""
    return [
        'e2e4',   # Normal pawn move
        'g1f3',   # Knight move
        'e1g1',   # Kingside castling
        'e1c1',   # Queenside castling
        'e7e8q',  # Pawn promotion to queen
        'd7d8n',  # Pawn promotion to knight
        'e5d6',   # En passant capture
    ]


@pytest.fixture
def sample_fen_positions():
    """Sample FEN positions for testing."""
    return {
        'starting': 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1',
        'italian_game': 'r1bqkbnr/pppp1ppp/2n5/4p3/2B1P3/5N2/PPPP1PPP/RNBQK2R b KQkq - 3 3',
        'scholars_mate': 'r1bqkb1r/pppp1Qpp/2n2n2/4p3/2B1P3/8/PPPP1PPP/RNB1K1NR b KQkq - 0 4',
        'endgame': '8/8/4k3/8/8/4K3/4P3/8 w - - 0 1',
        'promotion': '8/P7/8/8/8/8/8/4K2k w - - 0 1',
    }


# =============================================================================
# Test Data Fixtures
# =============================================================================

@pytest.fixture
def all_chess_squares():
    """All 64 chess squares in algebraic notation."""
    files = 'abcdefgh'
    ranks = '12345678'
    return [f + r for f in files for r in ranks]


@pytest.fixture
def corner_squares():
    """Corner squares of the chessboard."""
    return ['a1', 'a8', 'h1', 'h8']


@pytest.fixture
def center_squares():
    """Center squares of the chessboard."""
    return ['d4', 'd5', 'e4', 'e5']

