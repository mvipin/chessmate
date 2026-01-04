#!/usr/bin/env python3
"""
Unit tests for SCARA kinematics and chess coordinate mapping
"""

import pytest
import math
from chessmate.kinematics import SCARAKinematics, ChessBoardMapper


class TestSCARAKinematics:
    """Test cases for SCARA kinematics solver"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.scara = SCARAKinematics()
    
    def test_forward_kinematics_safe_position(self):
        """Test forward kinematics at a collision-safe position"""
        # Use θ2=30° which is safe (above 20° collision buffer)
        theta2_safe = math.radians(30)  # 30 degrees - safe configuration
        x, y, z = self.scara.forward_kinematics(0.0, theta2_safe, 0.05)

        # Calculate expected position using trigonometry
        l1, l2 = self.scara.config.l1, self.scara.config.l2
        expected_x = l1 + l2 * math.cos(theta2_safe)
        expected_y = l2 * math.sin(theta2_safe)
        expected_z = 0.05

        assert abs(x - expected_x) < 1e-6
        assert abs(y - expected_y) < 1e-6
        assert abs(z - expected_z) < 1e-6
    
    def test_forward_kinematics_90_degrees(self):
        """Test forward kinematics at 90 degrees with safe θ2"""
        theta1 = math.pi / 2  # 90 degrees
        theta2 = math.radians(45)  # 45 degrees - safe configuration
        z = 0.03

        x, y, z_out = self.scara.forward_kinematics(theta1, theta2, z)

        # Calculate expected position
        l1, l2 = self.scara.config.l1, self.scara.config.l2
        # After theta1 rotation, Link 1 points in +Y direction
        # Link 2 is at angle theta2 relative to Link 1
        link1_end_x = 0.0  # l1 * cos(90°) = 0
        link1_end_y = l1   # l1 * sin(90°) = l1

        # Link 2 position relative to Link 1 end
        link2_x = l2 * math.cos(theta1 + theta2)
        link2_y = l2 * math.sin(theta1 + theta2)

        expected_x = link1_end_x + link2_x
        expected_y = link1_end_y + link2_y

        assert abs(x - expected_x) < 1e-6
        assert abs(y - expected_y) < 1e-6
        assert abs(z_out - z) < 1e-6
    
    def test_inverse_kinematics_roundtrip(self):
        """Test that FK -> IK -> FK gives consistent results"""
        # Test multiple configurations (within workspace and collision-safe)
        test_configs = [
            (0.0, math.radians(30), 0.05),      # Safe: θ2=30° > 20° buffer
            (math.pi/4, math.radians(45), 0.03), # Safe: θ2=45° in valid range
            (-math.pi/6, math.radians(60), 0.08), # Safe: θ2=60° in valid range
            (math.pi/6, math.radians(75), 0.02)   # Safe: θ2=75° < 100° limit
        ]

        for theta1_orig, theta2_orig, z_orig in test_configs:
            # Forward kinematics
            x, y, z = self.scara.forward_kinematics(theta1_orig, theta2_orig, z_orig)

            # Check if position is reachable
            if not self.scara.is_reachable(x, y, z):
                continue  # Skip unreachable positions

            # Inverse kinematics
            theta1_calc, theta2_calc, z_calc = self.scara.inverse_kinematics(x, y, z)

            # Forward kinematics again
            x_check, y_check, z_check = self.scara.forward_kinematics(theta1_calc, theta2_calc, z_calc)

            # Check Cartesian position consistency (this is what matters)
            assert abs(x - x_check) < 1e-6, f"X mismatch: {x:.6f} vs {x_check:.6f}"
            assert abs(y - y_check) < 1e-6, f"Y mismatch: {y:.6f} vs {y_check:.6f}"
            assert abs(z - z_check) < 1e-6, f"Z mismatch: {z:.6f} vs {z_check:.6f}"
    
    def test_workspace_limits(self):
        """Test workspace boundary conditions"""
        # Test maximum reach
        max_reach = self.scara.workspace_radius_max
        assert self.scara.is_reachable(max_reach - 0.001, 0.0, 0.05)
        assert not self.scara.is_reachable(max_reach + 0.001, 0.0, 0.05)
        
        # Test minimum reach (if applicable)
        if self.scara.workspace_radius_min > 0.001:
            min_reach = self.scara.workspace_radius_min
            assert not self.scara.is_reachable(min_reach - 0.001, 0.0, 0.05)
            assert self.scara.is_reachable(min_reach + 0.001, 0.0, 0.05)
    
    def test_z_limits(self):
        """Test Z-axis limits"""
        # Valid Z position
        assert self.scara.is_reachable(0.2, 0.0, 0.05)
        
        # Invalid Z positions
        with pytest.raises(ValueError):
            self.scara.inverse_kinematics(0.2, 0.0, -0.01)  # Below minimum
        
        with pytest.raises(ValueError):
            self.scara.inverse_kinematics(0.2, 0.0, 0.15)   # Above maximum

    def test_collision_avoidance(self):
        """Test self-collision avoidance system"""
        # Test configurations that should be blocked
        with pytest.raises(ValueError, match="Self-collision"):
            self.scara.forward_kinematics(0.0, 0.0, 0.05)  # θ2=0° (alignment)

        with pytest.raises(ValueError, match="Self-collision"):
            self.scara.forward_kinematics(0.0, math.radians(15), 0.05)  # θ2=15° < 20° buffer

        with pytest.raises(ValueError, match="Self-collision"):
            self.scara.forward_kinematics(0.0, math.radians(105), 0.05)  # θ2=105° > 100° limit

        # Test configurations that should be allowed
        try:
            self.scara.forward_kinematics(0.0, math.radians(20), 0.05)  # θ2=20° (boundary)
            self.scara.forward_kinematics(0.0, math.radians(30), 0.05)  # θ2=30° (safe)
            self.scara.forward_kinematics(0.0, math.radians(90), 0.05)  # θ2=90° (safe)
            self.scara.forward_kinematics(0.0, math.radians(100), 0.05) # θ2=100° (boundary)
        except ValueError:
            pytest.fail("Safe configurations should not raise collision errors")

        # Test collision-free checking
        assert not self.scara.is_collision_free(0.0, 0.0, 0.05)  # Should fail
        assert not self.scara.is_collision_free(0.0, math.radians(15), 0.05)  # Should fail
        assert self.scara.is_collision_free(0.0, math.radians(30), 0.05)  # Should pass
        assert self.scara.is_collision_free(0.0, math.radians(90), 0.05)  # Should pass


class TestChessBoardMapper:
    """Test cases for chess board coordinate mapping"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.mapper = ChessBoardMapper()
    
    def test_square_validation(self):
        """Test chess square name validation"""
        # Valid squares
        assert self.mapper.is_valid_square("a1")
        assert self.mapper.is_valid_square("h8")
        assert self.mapper.is_valid_square("e4")
        assert self.mapper.is_valid_square("D5")  # Case insensitive
        
        # Invalid squares
        assert not self.mapper.is_valid_square("i1")  # Invalid file
        assert not self.mapper.is_valid_square("a9")  # Invalid rank
        assert not self.mapper.is_valid_square("a")   # Too short
        assert not self.mapper.is_valid_square("a11") # Too long
        assert not self.mapper.is_valid_square("")    # Empty
    
    def test_corner_squares(self):
        """Test coordinate mapping for corner squares"""
        # Test a1 (bottom-left)
        x, y, z = self.mapper.square_to_cartesian("a1")
        assert x < self.mapper.config.board_center_x  # Behind center
        assert y < self.mapper.config.board_center_y  # Left of center
        
        # Test h8 (top-right)
        x, y, z = self.mapper.square_to_cartesian("h8")
        assert x > self.mapper.config.board_center_x  # Ahead of center
        assert y > self.mapper.config.board_center_y  # Right of center
        
        # Test h1 (bottom-right)
        x, y, z = self.mapper.square_to_cartesian("h1")
        assert x < self.mapper.config.board_center_x  # Behind center
        assert y > self.mapper.config.board_center_y  # Right of center
        
        # Test a8 (top-left)
        x, y, z = self.mapper.square_to_cartesian("a8")
        assert x > self.mapper.config.board_center_x  # Ahead of center
        assert y < self.mapper.config.board_center_y  # Left of center
    
    def test_center_squares(self):
        """Test coordinate mapping for center squares"""
        # Test squares near board center
        center_squares = ["d4", "d5", "e4", "e5"]
        
        for square in center_squares:
            x, y, z = self.mapper.square_to_cartesian(square)
            
            # Should be close to board center
            assert abs(x - self.mapper.config.board_center_x) < self.mapper.config.square_size
            assert abs(y - self.mapper.config.board_center_y) < self.mapper.config.square_size
            assert abs(z - self.mapper.config.board_height) < 1e-6
    
    def test_coordinate_roundtrip(self):
        """Test that square -> coordinates -> square gives consistent results"""
        test_squares = ["a1", "a8", "h1", "h8", "e4", "d5", "c3", "f6"]
        
        for square_orig in test_squares:
            # Convert to coordinates
            x, y, z = self.mapper.square_to_cartesian(square_orig)
            
            # Convert back to square
            square_calc = self.mapper.cartesian_to_square(x, y)
            
            assert square_calc == square_orig.lower()
    
    def test_move_distance_calculation(self):
        """Test chess move distance calculations"""
        # Test horizontal move (same rank)
        dist = self.mapper.calculate_move_distance("a1", "h1")
        expected = 7 * self.mapper.config.square_size  # 7 squares
        assert abs(dist - expected) < 1e-6
        
        # Test vertical move (same file)
        dist = self.mapper.calculate_move_distance("a1", "a8")
        expected = 7 * self.mapper.config.square_size  # 7 squares
        assert abs(dist - expected) < 1e-6
        
        # Test diagonal move
        dist = self.mapper.calculate_move_distance("a1", "h8")
        expected = math.sqrt(2) * 7 * self.mapper.config.square_size  # Diagonal
        assert abs(dist - expected) < 1e-6
    
    def test_piece_handling_positions(self):
        """Test piece pickup, place, and transport positions"""
        square = "e4"
        
        # Test pickup position
        x, y, z = self.mapper.get_pickup_position(square)
        square_x, square_y, _ = self.mapper.square_to_cartesian(square)
        assert abs(x - square_x) < 1e-6
        assert abs(y - square_y) < 1e-6
        assert abs(z - self.mapper.config.piece_pickup_height) < 1e-6
        
        # Test transport position (should be higher)
        x, y, z = self.mapper.get_transport_position(square)
        assert z > self.mapper.config.piece_pickup_height
        
        # Test place position
        x, y, z = self.mapper.get_place_position(square)
        assert abs(z - self.mapper.config.piece_place_height) < 1e-6
    
    def test_capture_storage_positions(self):
        """Test captured piece storage positions"""
        # Test white piece storage
        x, y, z = self.mapper.get_capture_storage_position("white", 0)
        assert y < 0  # Should be on negative Y side
        
        # Test black piece storage
        x, y, z = self.mapper.get_capture_storage_position("black", 0)
        assert y > 0  # Should be on positive Y side
        
        # Test multiple captures (should be offset)
        x1, y1, z1 = self.mapper.get_capture_storage_position("white", 0)
        x2, y2, z2 = self.mapper.get_capture_storage_position("white", 1)
        assert abs(y2 - y1) > 0  # Should be offset


if __name__ == '__main__':
    pytest.main([__file__])
