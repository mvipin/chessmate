#!/usr/bin/env python3
"""
ChessMate Animation Demo

This node creates smooth animated trajectories showing the SCARA robot
reaching various chess squares. It publishes joint states to demonstrate
realistic robot motion with the updated F360 dimensions.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time
import sys
import os

# Add the kinematics package to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../chessmate_kinematics'))

from chessmate.kinematics.scara_kinematics import SCARAKinematics, SCARAConfiguration
from chessmate.kinematics.chess_coordinate_mapper import ChessBoardMapper


class ChessAnimationDemo(Node):
    """
    Animated demonstration of SCARA robot reaching chess squares
    """
    
    def __init__(self):
        super().__init__('chess_animation_demo')
        
        # Initialize kinematics with collision avoidance enabled
        collision_config = SCARAConfiguration(
            collision_buffer_angle=math.radians(30),  # 30-degree buffer
            enable_collision_avoidance=True
        )
        self.scara = SCARAKinematics(collision_config)
        self.board_mapper = ChessBoardMapper()

        self.get_logger().info('🛡️ Collision avoidance enabled: 30° buffer around Link 1')
        
        # Joint state publisher
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Animation parameters
        self.animation_timer = self.create_timer(0.05, self.animation_callback)  # 20 Hz
        self.current_time = 0.0
        self.animation_phase = 0
        self.phase_start_time = 0.0

        # Demo sequence: chess squares to visit with Z-heights
        self.demo_sequence = [
            ('e4', 0.025),  # Center square - pickup height
            ('a1', 0.060),  # Bottom-left corner - high travel
            ('h1', 0.040),  # Bottom-right corner - medium travel
            ('h8', 0.070),  # Top-right corner - highest travel
            ('a8', 0.050),  # Top-left corner - medium-high travel
            ('d4', 0.030),  # Center-left - low travel
            ('e5', 0.035),  # Center-right - low-medium travel
            ('f6', 0.045),  # Mid-board - medium travel
            ('c3', 0.055),  # Lower-left quadrant - high travel
            ('g7', 0.065),  # Upper-right quadrant - very high travel
        ]

        # Current target and trajectory
        self.current_target_idx = 0
        self.trajectory_duration = 4.0  # 4 seconds per move (longer for Z-axis)
        self.pause_duration = 1.5       # 1.5 second pause at each position
        
        # Joint angle trajectories
        self.start_joints = [0.0, 0.0, 0.08]  # Starting position (higher for dramatic effect)
        self.target_joints = [0.0, 0.0, 0.08]
        self.current_joints = [0.0, 0.0, 0.08]

        # Z-axis movement parameters
        self.travel_height = 0.08  # Safe travel height between moves
        self.use_travel_height = True  # Enable multi-phase Z movement

        # Animation state
        self.is_moving = True
        self.move_start_time = 0.0
        
        self.get_logger().info('ChessMate Animation Demo started!')
        sequence_names = [f"{square}@{int(z*1000)}mm" for square, z in self.demo_sequence]
        self.get_logger().info(f'Demo sequence: {" -> ".join(sequence_names)}')

        # Calculate initial target
        self.calculate_next_target()
    
    def calculate_next_target(self):
        """Calculate joint angles for the next target square with custom Z-height"""
        if self.current_target_idx >= len(self.demo_sequence):
            self.current_target_idx = 0  # Loop back to start

        target_square, target_z_height = self.demo_sequence[self.current_target_idx]

        try:
            # Get chess square X,Y position but use custom Z-height
            x, y, _ = self.board_mapper.get_pickup_position(target_square)
            z = target_z_height  # Use custom Z-height for dramatic movement

            # Check if reachable and collision-free
            if self.scara.is_reachable(x, y, z):
                # Calculate inverse kinematics
                theta1, theta2, z_joint = self.scara.inverse_kinematics(x, y, z)

                # Double-check collision avoidance
                if self.scara.is_collision_free(theta1, theta2, z_joint):
                    # Update trajectory
                    self.start_joints = self.current_joints.copy()
                    self.target_joints = [theta1, theta2, z_joint]

                    self.get_logger().info(
                        f'✅ Moving to {target_square}@{z*1000:.0f}mm: ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f})mm '
                        f'-> joints [{math.degrees(theta1):.1f}°, {math.degrees(theta2):.1f}°, {z_joint*1000:.1f}mm]'
                    )
                else:
                    self.get_logger().warn(f'🛡️ Square {target_square}@{z*1000:.0f}mm would cause collision! Skipping...')
                    self.current_target_idx += 1
                    self.calculate_next_target()
                    return

            else:
                self.get_logger().warn(f'❌ Square {target_square}@{z*1000:.0f}mm is not reachable! Skipping...')
                self.current_target_idx += 1
                self.calculate_next_target()
                return

        except Exception as e:
            self.get_logger().error(f'Failed to calculate target for {target_square}@{target_z_height*1000:.0f}mm: {e}')
            self.current_target_idx += 1
            self.calculate_next_target()
            return

        # Reset timing
        self.move_start_time = self.current_time
        self.is_moving = True
    
    def interpolate_joints(self, t):
        """
        Smooth interpolation between start and target joint positions
        Uses a smooth S-curve (sigmoid-based) for natural motion
        """
        if t <= 0.0:
            return self.start_joints
        elif t >= 1.0:
            return self.target_joints
        
        # Smooth S-curve interpolation (ease-in-out)
        # Using a modified sigmoid function for smooth acceleration/deceleration
        smooth_t = 3 * t * t - 2 * t * t * t  # Cubic ease-in-out
        
        interpolated = []
        for i in range(3):
            start_val = self.start_joints[i]
            target_val = self.target_joints[i]
            interp_val = start_val + (target_val - start_val) * smooth_t
            interpolated.append(interp_val)
        
        return interpolated
    
    def animation_callback(self):
        """Main animation loop callback"""
        self.current_time += 0.05  # 50ms timestep
        
        if self.is_moving:
            # Calculate progress through current trajectory
            elapsed = self.current_time - self.move_start_time
            progress = elapsed / self.trajectory_duration
            
            if progress >= 1.0:
                # Movement complete, start pause
                self.current_joints = self.target_joints.copy()
                self.is_moving = False
                self.move_start_time = self.current_time
                
                target_square, target_z = self.demo_sequence[self.current_target_idx]
                self.get_logger().info(f'Reached {target_square}@{target_z*1000:.0f}mm! Pausing...')
                
            else:
                # Interpolate current joint positions
                self.current_joints = self.interpolate_joints(progress)
        
        else:
            # Pausing at current position
            elapsed = self.current_time - self.move_start_time
            
            if elapsed >= self.pause_duration:
                # Pause complete, move to next target
                self.current_target_idx += 1
                self.calculate_next_target()
        
        # Publish current joint state
        self.publish_joint_state()
    
    def publish_joint_state(self):
        """Publish current joint positions"""
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # SCARA joint names (must match URDF)
        joint_state.name = ['z_joint', 'base_joint', 'shoulder_joint']
        
        # Current joint positions
        joint_state.position = [
            self.current_joints[2],  # Z-axis (vertical)
            self.current_joints[0],  # Joint 1 (base rotation)
            self.current_joints[1],  # Joint 2 (elbow rotation)
        ]
        
        # Zero velocities and efforts for this demo
        joint_state.velocity = [0.0, 0.0, 0.0]
        joint_state.effort = [0.0, 0.0, 0.0]
        
        self.joint_pub.publish(joint_state)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        demo_node = ChessAnimationDemo()
        
        print("\n=== ChessMate Animation Demo ===")
        print("Watch RViz2 to see the SCARA robot smoothly move between chess squares!")
        print("The robot will visit key squares demonstrating reachability.")
        print("Press Ctrl+C to stop the demo.\n")
        
        rclpy.spin(demo_node)
        
    except KeyboardInterrupt:
        print("\nDemo stopped by user.")
    except Exception as e:
        print(f"Demo failed: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
