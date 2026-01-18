#!/usr/bin/env python3
"""
Unit Tests for SCARA Kinematics

Tests the inverse kinematics calculations for the SCARA robot arm.
These are pure unit tests with no hardware or ROS2 dependencies.

Test Coverage:
    - Forward kinematics (joint angles -> end effector position)
    - Inverse kinematics (position -> joint angles)
    - Workspace boundary validation
    - Edge cases and error handling
"""

import pytest
import math
import numpy as np

# Mark all tests in this module as unit tests
pytestmark = pytest.mark.unit


class TestSCARAKinematics:
    """Test suite for SCARA robot kinematics calculations."""

    # Robot parameters (matching scara_config.yaml)
    L1 = 0.202  # First link length in meters
    L2 = 0.190  # Second link length in meters

    def forward_kinematics(self, theta1: float, theta2: float) -> tuple:
        """
        Calculate end effector position from joint angles.
        
        Args:
            theta1: First joint angle in radians
            theta2: Second joint angle in radians
            
        Returns:
            Tuple of (x, y) position in meters
        """
        x = self.L1 * math.cos(theta1) + self.L2 * math.cos(theta1 + theta2)
        y = self.L1 * math.sin(theta1) + self.L2 * math.sin(theta1 + theta2)
        return (x, y)

    def inverse_kinematics(self, x: float, y: float, elbow_up: bool = True) -> tuple:
        """
        Calculate joint angles from end effector position.
        
        Args:
            x: Target X position in meters
            y: Target Y position in meters
            elbow_up: If True, use elbow-up configuration
            
        Returns:
            Tuple of (theta1, theta2) in radians
            
        Raises:
            ValueError: If position is outside workspace
        """
        # Distance from origin to target
        d = math.sqrt(x**2 + y**2)
        
        # Check workspace limits
        max_reach = self.L1 + self.L2
        min_reach = abs(self.L1 - self.L2)
        
        if d > max_reach or d < min_reach:
            raise ValueError(f"Position ({x:.3f}, {y:.3f}) outside workspace. "
                           f"Distance {d:.3f}m not in [{min_reach:.3f}, {max_reach:.3f}]")
        
        # Calculate theta2 using law of cosines
        cos_theta2 = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Handle numerical errors
        
        if elbow_up:
            theta2 = -math.acos(cos_theta2)
        else:
            theta2 = math.acos(cos_theta2)
        
        # Calculate theta1
        k1 = self.L1 + self.L2 * math.cos(theta2)
        k2 = self.L2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)
        
        return (theta1, theta2)

    # =========================================================================
    # Forward Kinematics Tests
    # =========================================================================

    def test_forward_kinematics_home_position(self):
        """Test FK at home position (both joints at 0)."""
        x, y = self.forward_kinematics(0, 0)
        expected_x = self.L1 + self.L2
        expected_y = 0.0
        
        assert abs(x - expected_x) < 1e-6, f"X mismatch: {x} != {expected_x}"
        assert abs(y - expected_y) < 1e-6, f"Y mismatch: {y} != {expected_y}"

    def test_forward_kinematics_90_degrees(self):
        """Test FK with first joint at 90 degrees."""
        x, y = self.forward_kinematics(math.pi / 2, 0)
        expected_x = 0.0
        expected_y = self.L1 + self.L2
        
        assert abs(x - expected_x) < 1e-6, f"X mismatch: {x} != {expected_x}"
        assert abs(y - expected_y) < 1e-6, f"Y mismatch: {y} != {expected_y}"

    def test_forward_kinematics_folded(self):
        """Test FK with arm folded back (theta2 = 180 degrees)."""
        x, y = self.forward_kinematics(0, math.pi)
        expected_x = self.L1 - self.L2
        expected_y = 0.0
        
        assert abs(x - expected_x) < 1e-6, f"X mismatch: {x} != {expected_x}"
        assert abs(y - expected_y) < 1e-6, f"Y mismatch: {y} != {expected_y}"

    # =========================================================================
    # Inverse Kinematics Tests
    # =========================================================================

    def test_inverse_kinematics_max_reach(self):
        """Test IK at maximum reach (arm fully extended)."""
        target_x = self.L1 + self.L2 - 0.001  # Just inside max reach
        target_y = 0.0
        
        theta1, theta2 = self.inverse_kinematics(target_x, target_y)
        
        # Verify with forward kinematics
        x, y = self.forward_kinematics(theta1, theta2)
        assert abs(x - target_x) < 1e-4, f"X mismatch: {x} != {target_x}"
        assert abs(y - target_y) < 1e-4, f"Y mismatch: {y} != {target_y}"

    def test_inverse_kinematics_center_workspace(self):
        """Test IK at center of workspace."""
        target_x = 0.25
        target_y = 0.15
        
        theta1, theta2 = self.inverse_kinematics(target_x, target_y)
        
        # Verify with forward kinematics
        x, y = self.forward_kinematics(theta1, theta2)
        assert abs(x - target_x) < 1e-4, f"X mismatch: {x} != {target_x}"
        assert abs(y - target_y) < 1e-4, f"Y mismatch: {y} != {target_y}"

    def test_inverse_kinematics_negative_y(self):
        """Test IK with negative Y coordinate."""
        target_x = 0.30
        target_y = -0.10
        
        theta1, theta2 = self.inverse_kinematics(target_x, target_y)
        
        # Verify with forward kinematics
        x, y = self.forward_kinematics(theta1, theta2)
        assert abs(x - target_x) < 1e-4, f"X mismatch: {x} != {target_x}"
        assert abs(y - target_y) < 1e-4, f"Y mismatch: {y} != {target_y}"

    def test_inverse_kinematics_outside_workspace_raises(self):
        """Test that IK raises error for unreachable positions."""
        # Position beyond maximum reach
        with pytest.raises(ValueError, match="outside workspace"):
            self.inverse_kinematics(1.0, 0.0)  # 1 meter is way beyond reach

    def test_inverse_kinematics_inside_min_reach_raises(self):
        """Test that IK raises error for positions inside minimum reach."""
        # Position inside minimum reach (too close to base)
        with pytest.raises(ValueError, match="outside workspace"):
            self.inverse_kinematics(0.005, 0.0)  # 5mm from base

    def test_inverse_kinematics_elbow_configurations(self):
        """Test that elbow_up and elbow_down give different solutions."""
        target_x = 0.25
        target_y = 0.10
        
        theta1_up, theta2_up = self.inverse_kinematics(target_x, target_y, elbow_up=True)
        theta1_down, theta2_down = self.inverse_kinematics(target_x, target_y, elbow_up=False)
        
        # Both should reach the same position
        x_up, y_up = self.forward_kinematics(theta1_up, theta2_up)
        x_down, y_down = self.forward_kinematics(theta1_down, theta2_down)
        
        assert abs(x_up - target_x) < 1e-4
        assert abs(y_up - target_y) < 1e-4
        assert abs(x_down - target_x) < 1e-4
        assert abs(y_down - target_y) < 1e-4
        
        # But theta2 should have opposite signs
        assert theta2_up * theta2_down < 0, "Elbow configurations should differ"

