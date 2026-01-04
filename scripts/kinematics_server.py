#!/usr/bin/env python3
"""
ChessMate Kinematics Server
Provides ROS 2 services for SCARA kinematics and chess coordinate mapping
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from chessmate.srv import ForwardKinematics, InverseKinematics, ChessSquareKinematics

from chessmate.kinematics import SCARAKinematics, ChessBoardMapper


class ChessMateKinematicsServer(Node):
    """
    ROS 2 service server for ChessMate kinematics operations
    
    Provides services:
    - forward_kinematics: Joint angles -> Cartesian position
    - inverse_kinematics: Cartesian position -> Joint angles  
    - chess_square_kinematics: Chess square -> Robot coordinates
    """
    
    def __init__(self):
        super().__init__('chessmate_kinematics_server')
        
        # Initialize kinematics solver and chess board mapper
        self.scara = SCARAKinematics()
        self.board_mapper = ChessBoardMapper()
        
        # Create service servers
        self.fk_service = self.create_service(
            ForwardKinematics, 
            'forward_kinematics', 
            self.forward_kinematics_callback
        )
        
        self.ik_service = self.create_service(
            InverseKinematics, 
            'inverse_kinematics', 
            self.inverse_kinematics_callback
        )
        
        self.chess_ik_service = self.create_service(
            ChessSquareKinematics, 
            'chess_square_kinematics', 
            self.chess_square_callback
        )
        
        self.get_logger().info('ChessMate Kinematics Server ready')
        self.get_logger().info(f'SCARA Configuration: L1={self.scara.config.l1:.3f}m, L2={self.scara.config.l2:.3f}m')
        self.get_logger().info(f'Workspace: {self.scara.workspace_radius_min:.3f}m to {self.scara.workspace_radius_max:.3f}m')
    
    def forward_kinematics_callback(self, request, response):
        """Handle forward kinematics service requests"""
        try:
            # Extract joint angles from request
            theta1, theta2, z = request.joint_angles[:3]
            
            # Compute forward kinematics
            x, y, z_out = self.scara.forward_kinematics(theta1, theta2, z)
            
            # Create response pose
            response.end_effector_pose = Pose()
            response.end_effector_pose.position = Point(x=x, y=y, z=z_out)
            response.end_effector_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            response.success = True
            response.message = f"Forward kinematics computed successfully"
            
            self.get_logger().debug(f"FK: [{theta1:.3f}, {theta2:.3f}, {z:.3f}] -> ({x:.3f}, {y:.3f}, {z_out:.3f})")
            
        except Exception as e:
            response.success = False
            response.message = f"Forward kinematics failed: {str(e)}"
            self.get_logger().error(response.message)
        
        return response
    
    def inverse_kinematics_callback(self, request, response):
        """Handle inverse kinematics service requests"""
        try:
            # Extract target position from request
            x = request.target_pose.position.x
            y = request.target_pose.position.y
            z = request.target_pose.position.z
            
            # Compute inverse kinematics
            theta1, theta2, z_out = self.scara.inverse_kinematics(x, y, z)
            
            # Create response
            response.joint_angles = [theta1, theta2, z_out]
            response.success = True
            response.message = "Inverse kinematics computed successfully"
            
            # Calculate solution quality (distance from workspace center)
            workspace_center_radius = (self.scara.workspace_radius_max + self.scara.workspace_radius_min) / 2
            current_radius = (x*x + y*y)**0.5
            response.solution_quality = max(0.0, 1.0 - abs(current_radius - workspace_center_radius) / workspace_center_radius)
            
            self.get_logger().debug(f"IK: ({x:.3f}, {y:.3f}, {z:.3f}) -> [{theta1:.3f}, {theta2:.3f}, {z_out:.3f}]")
            
        except Exception as e:
            response.success = False
            response.message = f"Inverse kinematics failed: {str(e)}"
            response.joint_angles = [0.0, 0.0, 0.0]
            response.solution_quality = 0.0
            self.get_logger().error(response.message)
        
        return response
    
    def chess_square_callback(self, request, response):
        """Handle chess square kinematics service requests"""
        try:
            square_name = request.square_name.lower()
            piece_height = request.piece_height / 1000.0  # Convert mm to meters
            approach_direction = request.approach_direction.lower()
            
            # Validate square name
            if not self.board_mapper.is_valid_square(square_name):
                raise ValueError(f"Invalid chess square: {square_name}")
            
            # Get target position based on approach direction
            if approach_direction == "top":
                x, y, z = self.board_mapper.get_pickup_position(square_name)
                z += piece_height
            elif approach_direction == "side":
                # For side approach, add small offset for collision avoidance
                x, y, z = self.board_mapper.get_transport_position(square_name)
            else:
                # Default to pickup position
                x, y, z = self.board_mapper.get_pickup_position(square_name)
                z += piece_height
            
            # Compute inverse kinematics for target position
            theta1, theta2, z_joint = self.scara.inverse_kinematics(x, y, z)
            
            # Create response
            response.target_pose = Pose()
            response.target_pose.position = Point(x=x, y=y, z=z)
            response.target_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            response.joint_angles = [theta1, theta2, z_joint]
            response.success = True
            response.message = f"Chess square {square_name} kinematics computed successfully"
            
            self.get_logger().debug(f"Chess IK: {square_name} -> ({x:.3f}, {y:.3f}, {z:.3f}) -> [{theta1:.3f}, {theta2:.3f}, {z_joint:.3f}]")
            
        except Exception as e:
            response.success = False
            response.message = f"Chess square kinematics failed: {str(e)}"
            response.joint_angles = [0.0, 0.0, 0.0]
            self.get_logger().error(response.message)
        
        return response


def main(args=None):
    """Main entry point for kinematics server"""
    rclpy.init(args=args)
    
    try:
        kinematics_server = ChessMateKinematicsServer()
        rclpy.spin(kinematics_server)
    except KeyboardInterrupt:
        pass
    finally:
        if 'kinematics_server' in locals():
            kinematics_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
