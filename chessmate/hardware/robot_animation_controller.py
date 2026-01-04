#!/usr/bin/env python3
"""
Robot Animation Controller Node

This node manages robot animations and actions by sending commands directly
to the robot controller Arduino, replacing the inter-Arduino communication
that previously went through the chessboard controller.

Key responsibilities:
- Receive game flow events and trigger appropriate robot animations
- Send move execution commands directly to robot controller
- Manage animation state transitions (wake up -> think hard -> doze off)
- Handle robot positioning commands (home Z, home all, reset pose)
"""

import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from chessmate.msg import ArduinoCommand, RobotAnimation, ChessMove, BoardState
from chessmate.hardware.arduino_communication_node import ArduinoType
from std_msgs.msg import String
import time
from typing import Dict, Optional


class RobotAnimationController(Node):
    """
    ROS 2 Node for controlling robot animations and actions
    
    This node acts as the intelligence layer that was previously handled
    by the chessboard Arduino's inter-Arduino communication system.
    """
    
    def __init__(self):
        super().__init__('robot_animation_controller')
        
        # Declare parameters
        self.declare_parameter('animation_timeout', 30.0)  # Seconds before doze off
        self.declare_parameter('move_execution_timeout', 60.0)  # Max time for move execution
        
        # Get parameters
        self.animation_timeout = self.get_parameter('animation_timeout').value
        self.move_timeout = self.get_parameter('move_execution_timeout').value
        
        self.get_logger().info("Robot Animation Controller starting")
        
        # State management
        self.current_animation = None
        self.last_activity_time = time.time()
        self.is_robot_turn = False
        self.pending_move = None
        
        # Publishers
        self.arduino_command_publisher = self.create_publisher(
            ArduinoCommand,
            'arduino_command',
            10
        )
        
        self.robot_status_publisher = self.create_publisher(
            String,
            'robot_status',
            10
        )
        
        # Subscribers
        self.board_state_subscriber = self.create_subscription(
            BoardState,
            'board_state',
            self.board_state_callback,
            10
        )
        
        self.chess_move_subscriber = self.create_subscription(
            ChessMove,
            'computer_move',
            self.computer_move_callback,
            10
        )
        
        self.game_event_subscriber = self.create_subscription(
            String,
            'game_events',
            self.game_event_callback,
            10
        )
        
        # Animation management timer
        self.animation_timer = self.create_timer(
            1.0,  # Check every second
            self.animation_management_callback
        )
        
        self.get_logger().info("Robot Animation Controller initialized successfully")
    
    def board_state_callback(self, msg: BoardState):
        """Handle board state updates to determine whose turn it is"""
        try:
            # Update activity time
            self.last_activity_time = time.time()
            
            # Determine if it's robot's turn
            was_robot_turn = self.is_robot_turn
            self.is_robot_turn = (msg.active_color == "black")  # Assuming robot plays black
            
            # Trigger appropriate animation based on turn change
            if not was_robot_turn and self.is_robot_turn:
                # Human finished move, robot's turn now
                self.get_logger().info("Robot turn started - triggering wake up animation")
                self.send_robot_animation(RobotAnimation.WAKE_UP)
                self.publish_status("Robot turn started")
                
            elif was_robot_turn and not self.is_robot_turn:
                # Robot finished move, human's turn now
                self.get_logger().info("Human turn started - robot will doze off after timeout")
                self.publish_status("Human turn started")
                
        except Exception as e:
            self.get_logger().error(f"Error processing board state: {e}")
    
    def computer_move_callback(self, msg: ChessMove):
        """Handle computer move execution requests"""
        try:
            # Format move for robot controller (6-character format: e2pe4p)
            move_str = f"{msg.from_square}{msg.piece_type.lower()}{msg.to_square}{msg.piece_type.lower()}"
            
            # Handle captures
            if msg.captured_piece_type:
                move_str = f"{msg.from_square}{msg.piece_type.lower()}{msg.to_square}x"
            
            self.get_logger().info(f"Executing robot move: {move_str}")
            self.send_robot_move(move_str)
            self.publish_status(f"Executing move: {move_str}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing computer move: {e}")
    
    def game_event_callback(self, msg: String):
        """Handle game events that trigger robot behaviors"""
        try:
            event = msg.data.lower()
            
            if "checkmate" in event:
                self.get_logger().info("Checkmate detected - triggering home all")
                self.send_robot_animation(RobotAnimation.HOME_ALL)
                self.publish_status("Game over - checkmate")
                
            elif "invalid_move" in event or "move_failed" in event:
                self.get_logger().info("Invalid move detected - triggering home Z")
                self.send_robot_animation(RobotAnimation.HOME_Z)
                self.publish_status("Move validation failed")
                
            elif "game_reset" in event:
                self.get_logger().info("Game reset - triggering reset pose")
                self.send_robot_animation(RobotAnimation.RESET_POSE)
                self.publish_status("Game reset")
                
        except Exception as e:
            self.get_logger().error(f"Error processing game event: {e}")
    
    def animation_management_callback(self):
        """Manage animation timeouts and state transitions"""
        try:
            current_time = time.time()
            time_since_activity = current_time - self.last_activity_time
            
            # If it's human's turn and enough time has passed, doze off
            if not self.is_robot_turn and time_since_activity > self.animation_timeout:
                if self.current_animation != RobotAnimation.DOZE_OFF:
                    self.get_logger().info("Human turn timeout - robot dozing off")
                    self.send_robot_animation(RobotAnimation.DOZE_OFF)
                    self.publish_status("Robot dozing off due to inactivity")
                    
        except Exception as e:
            self.get_logger().error(f"Error in animation management: {e}")
    
    def send_robot_animation(self, animation_type: int, data: str = ""):
        """Send animation command to robot controller"""
        try:
            # Map animation types to Arduino commands
            animation_command_map = {
                RobotAnimation.RESET_POSE: ArduinoCommand.CMD_ROBOT_RESET_POSE,
                RobotAnimation.DOZE_OFF: ArduinoCommand.CMD_ROBOT_DOZE_OFF,
                RobotAnimation.WAKE_UP: ArduinoCommand.CMD_ROBOT_WAKE_UP,
                RobotAnimation.THINK_HARD: ArduinoCommand.CMD_ROBOT_THINK_HARD,
                RobotAnimation.HOME_Z: ArduinoCommand.CMD_ROBOT_HOME_Z,
                RobotAnimation.HOME_ALL: ArduinoCommand.CMD_ROBOT_HOME_ALL,
            }
            
            if animation_type not in animation_command_map:
                self.get_logger().error(f"Unknown animation type: {animation_type}")
                return
            
            # Create Arduino command
            cmd = ArduinoCommand()
            cmd.timestamp = self.get_clock().now().to_msg()
            cmd.command_type = animation_command_map[animation_type]
            cmd.target_arduino = ArduinoType.ROBOT_CONTROLLER.value
            cmd.data = data
            
            # Publish command
            self.arduino_command_publisher.publish(cmd)
            self.current_animation = animation_type
            
            self.get_logger().debug(f"Sent robot animation command: {animation_type}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending robot animation: {e}")
    
    def send_robot_move(self, move_str: str):
        """Send move execution command to robot controller"""
        try:
            # Create Arduino command for move execution
            cmd = ArduinoCommand()
            cmd.timestamp = self.get_clock().now().to_msg()
            cmd.command_type = ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE
            cmd.target_arduino = ArduinoType.ROBOT_CONTROLLER.value
            cmd.data = move_str
            
            # Publish command
            self.arduino_command_publisher.publish(cmd)
            self.pending_move = move_str
            
            self.get_logger().debug(f"Sent robot move command: {move_str}")
            
        except Exception as e:
            self.get_logger().error(f"Error sending robot move: {e}")
    
    def publish_status(self, status: str):
        """Publish robot status message"""
        try:
            status_msg = String()
            status_msg.data = f"RobotController: {status}"
            self.robot_status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = RobotAnimationController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in robot animation controller: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
