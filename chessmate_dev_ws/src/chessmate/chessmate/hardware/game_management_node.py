#!/usr/bin/env python3
"""
Game Management ROS 2 Node for ChessMate

This node coordinates between the chess engine, hardware interfaces,
and robot control systems. It's based on the game management logic
from ChessSoft.py but adapted for ROS 2 service-based architecture.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import asyncio
import threading
import time
from typing import Optional, Dict, List
from enum import Enum

from chessmate.msg import (
    RotaryEncoderEvent, LCDCommand, ArduinoCommand, 
    BoardState, ChessMove, RobotStatus
)
from chessmate.srv import (
    GetBestMove, EvaluatePosition, ValidateMove, UpdateGameState
)
from std_msgs.msg import String


class GameState(Enum):
    """Game state enumeration"""
    IDLE = "idle"
    MENU = "menu"
    GAME_SETUP = "game_setup"
    HUMAN_TURN = "human_turn"
    COMPUTER_TURN = "computer_turn"
    GAME_OVER = "game_over"


class GameMode(Enum):
    """Game mode enumeration"""
    HUMAN_VS_COMPUTER = "human_vs_computer"
    COMPUTER_VS_HUMAN = "computer_vs_human"
    ANALYSIS = "analysis"
    PUZZLE = "puzzle"


class GameManagementNode(Node):
    """
    ROS 2 Node for game management and coordination
    
    This node:
    - Coordinates between chess engine and hardware interfaces
    - Manages game state and flow
    - Handles user interface interactions
    - Controls robot arm movements
    - Manages menu system and game setup
    """
    
    def __init__(self):
        super().__init__('game_management_node')
        
        # Declare parameters
        self.declare_parameter('default_skill_level', 3)
        self.declare_parameter('default_time_limit', 5.0)
        self.declare_parameter('menu_timeout', 30.0)
        self.declare_parameter('move_timeout', 60.0)
        
        # Get parameters
        self.default_skill = self.get_parameter('default_skill_level').get_parameter_value().integer_value
        self.default_time = self.get_parameter('default_time_limit').get_parameter_value().double_value
        self.menu_timeout = self.get_parameter('menu_timeout').get_parameter_value().double_value
        self.move_timeout = self.get_parameter('move_timeout').get_parameter_value().double_value
        
        self.get_logger().info("Game Management Node starting")
        
        # Game state
        self.current_state = GameState.IDLE
        self.game_mode = GameMode.HUMAN_VS_COMPUTER
        self.skill_level = self.default_skill
        self.time_limit = self.default_time
        self.human_is_white = True
        
        # Menu state (based on original Menu.py)
        self.menu_options = [
            "New Game",
            "New Puzzle", 
            "Computer Time",
            "Human Time",
            "Color",
            "Level",
            "Shutdown",
            "Manual Move"
        ]
        self.submenu_options = [0] * 10
        self.current_menu_selection = 0
        self.menu_level = 0
        
        # Game tracking
        self.current_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
        self.move_history = []
        self.waiting_for_move = False
        self.last_move_time = None
        
        # Thread safety
        self.state_lock = threading.Lock()
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers
        self.encoder_subscriber = self.create_subscription(
            RotaryEncoderEvent,
            'rotary_encoder_events',
            self._encoder_event_callback,
            qos_profile
        )
        
        self.board_state_subscriber = self.create_subscription(
            BoardState,
            'board_state',
            self._board_state_callback,
            qos_profile
        )
        
        self.arduino_response_subscriber = self.create_subscription(
            String,
            'arduino_responses',
            self._arduino_response_callback,
            qos_profile
        )
        
        # Create publishers
        self.lcd_command_publisher = self.create_publisher(
            LCDCommand,
            'lcd_commands',
            qos_profile
        )
        
        self.arduino_command_publisher = self.create_publisher(
            ArduinoCommand,
            'arduino_commands',
            qos_profile
        )
        
        self.robot_status_publisher = self.create_publisher(
            RobotStatus,
            'robot_status',
            qos_profile
        )

        self.game_events_publisher = self.create_publisher(
            String,
            'game_events',
            qos_profile
        )
        
        # Create service clients for chess engine
        self.get_best_move_client = self.create_client(GetBestMove, 'get_best_move')
        self.evaluate_position_client = self.create_client(EvaluatePosition, 'evaluate_position')
        self.validate_move_client = self.create_client(ValidateMove, 'validate_move')
        self.update_game_state_client = self.create_client(UpdateGameState, 'update_game_state')
        
        # Wait for chess engine services
        self._wait_for_chess_engine_services()
        
        # Initialize display
        self._initialize_display()
        
        # Start main game loop
        self.game_timer = self.create_timer(0.1, self._game_loop)
        
        self.get_logger().info("Game Management Node initialized successfully")
    
    def _wait_for_chess_engine_services(self):
        """Wait for chess engine services to become available"""
        services = [
            (self.get_best_move_client, 'get_best_move'),
            (self.evaluate_position_client, 'evaluate_position'),
            (self.validate_move_client, 'validate_move'),
            (self.update_game_state_client, 'update_game_state')
        ]
        
        for client, service_name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().warning(f"Chess engine service '{service_name}' not available")
            else:
                self.get_logger().info(f"Connected to chess engine service '{service_name}'")
    
    def _initialize_display(self):
        """Initialize LCD display with welcome message"""
        self._send_lcd_command(
            LCDCommand.CMD_DISPLAY_TEXT,
            text="ChessMate Ready",
            font_size=1
        )
        
        # Show main menu after brief delay
        self.menu_timer = self.create_timer(2.0, self._show_main_menu_once)
    
    def _show_main_menu_once(self):
        """Show main menu once and cancel timer"""
        self.menu_timer.cancel()
        self._show_main_menu()

    def _show_main_menu_after_shutdown(self):
        """Show main menu after shutdown and cancel timer"""
        self.shutdown_timer.cancel()
        self._show_main_menu()

    def _show_main_menu(self):
        """Display main menu"""
        with self.state_lock:
            self.current_state = GameState.MENU
            self.menu_level = 0
            self.current_menu_selection = 0
        
        self._update_menu_display()
    
    def _encoder_event_callback(self, msg: RotaryEncoderEvent):
        """
        Handle rotary encoder events
        
        Args:
            msg: RotaryEncoderEvent message
        """
        try:
            with self.state_lock:
                if self.current_state == GameState.MENU:
                    if msg.event_type == RotaryEncoderEvent.EVENT_TYPE_ROTATION:
                        self._handle_menu_navigation(msg.direction)
                    elif msg.event_type == RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS:
                        self._handle_menu_selection()
                
                elif self.current_state in [GameState.HUMAN_TURN, GameState.COMPUTER_TURN]:
                    # Handle in-game encoder events (e.g., manual move entry)
                    if msg.event_type == RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS:
                        self._handle_game_button_press()
                
        except Exception as e:
            self.get_logger().error(f"Error handling encoder event: {e}")
    
    def _handle_menu_navigation(self, direction: int):
        """Handle menu navigation"""
        if self.menu_level == 0:
            # Main menu
            self.current_menu_selection = max(0, min(len(self.menu_options) - 1, 
                                                   self.current_menu_selection + direction))
        else:
            # Submenu
            if self.current_menu_selection < len(self.submenu_options):
                self.submenu_options[self.current_menu_selection] = max(0, 
                    self.submenu_options[self.current_menu_selection] + direction)
        
        self._update_menu_display()
    
    def _handle_menu_selection(self):
        """Handle menu selection (button press)"""
        if self.menu_level == 0:
            # Entering submenu
            self.menu_level = 1
            self._update_menu_display()
        else:
            # Process submenu selection
            self._process_menu_action()
    
    def _process_menu_action(self):
        """Process menu action based on current selection"""
        action = self.menu_options[self.current_menu_selection]
        
        if action == "New Game":
            self._start_new_game()
        elif action == "New Puzzle":
            self._start_puzzle()
        elif action == "Computer Time":
            self.time_limit = self.submenu_options[2]
            self._show_status(f"Computer time: {self.time_limit}s")
        elif action == "Human Time":
            human_time = self.submenu_options[3] * 10
            self._show_status(f"Human time: {human_time}s")
        elif action == "Color":
            self.human_is_white = self.submenu_options[4] == 0
            color = "White" if self.human_is_white else "Black"
            self._show_status(f"Human plays: {color}")
        elif action == "Level":
            self.skill_level = self.submenu_options[5]
            self._show_status(f"Skill level: {self.skill_level}")
        elif action == "Shutdown":
            self._shutdown_system()
        elif action == "Manual Move":
            self._enter_manual_move_mode()
        
        # Return to main menu
        self.menu_level = 0
        self._update_menu_display()
    
    def _update_menu_display(self):
        """Update menu display on LCD"""
        if self.menu_level == 0:
            # Show main menu
            self._send_lcd_command(
                LCDCommand.CMD_DISPLAY_MENU,
                menu_options=self.menu_options,
                selected_index=self.current_menu_selection,
                menu_level=0
            )
        else:
            # Show submenu
            action = self.menu_options[self.current_menu_selection]
            submenu_text = self._get_submenu_text(action)
            
            self._send_lcd_command(
                LCDCommand.CMD_DISPLAY_TEXT,
                text=submenu_text,
                font_size=1
            )
    
    def _get_submenu_text(self, action: str) -> str:
        """Get submenu text for display"""
        if action == "Computer Time":
            return f"Comp Time: {self.submenu_options[2]}s"
        elif action == "Human Time":
            return f"Human Time: {self.submenu_options[3] * 10}s"
        elif action == "Color":
            color = "White" if self.submenu_options[4] == 0 else "Black"
            return f"Human: {color}"
        elif action == "Level":
            return f"Level: {self.submenu_options[5]}"
        else:
            return f"{action}: {self.submenu_options[self.current_menu_selection]}"
    
    def _start_new_game(self):
        """Start a new chess game"""
        with self.state_lock:
            self.current_state = GameState.GAME_SETUP
            self.game_mode = GameMode.HUMAN_VS_COMPUTER
            self.current_fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
            self.move_history = []
        
        self._show_status("Starting new game...")
        
        # Initialize chess engine with new game
        self._update_chess_engine_state()
        
        # Send board reset command to Arduino
        self._send_arduino_command(ArduinoCommand.CMD_RESET, target_arduino=0)
        
        # Transition to appropriate turn
        if self.human_is_white:
            self.current_state = GameState.HUMAN_TURN
            self._show_status("Your turn (White)")
            self._publish_game_event("human_turn_started")
        else:
            self.current_state = GameState.COMPUTER_TURN
            self._show_status("Computer thinking...")
            self._publish_game_event("computer_turn_started")
    
    def _start_puzzle(self):
        """Start a chess puzzle"""
        with self.state_lock:
            self.current_state = GameState.GAME_SETUP
            self.game_mode = GameMode.PUZZLE
            # Set puzzle position (example from original code)
            self.current_fen = "r2qk2r/pb4pp/1n2Pb2/2B2Q2/p1p5/2P5/2B2PPP/RN2R1K1 w - - 1 0"
        
        self._show_status("Loading puzzle...")
        self._update_chess_engine_state()
        
        self.current_state = GameState.HUMAN_TURN
        self._show_status("Solve the puzzle!")
    
    def _board_state_callback(self, msg: BoardState):
        """
        Handle board state updates
        
        Args:
            msg: BoardState message
        """
        # Process board state changes
        self.get_logger().debug(f"Board state updated: {msg.fen}")
        
        # Update current position if provided
        if msg.fen:
            self.current_fen = msg.fen
    
    def _arduino_response_callback(self, msg: String):
        """
        Handle Arduino responses
        
        Args:
            msg: String message with Arduino response
        """
        response = msg.data
        self.get_logger().debug(f"Arduino response: {response}")
        
        # Process move confirmations, board state updates, etc.
        if "move_confirmed" in response:
            self._handle_move_confirmation(response)
    
    def _handle_move_confirmation(self, response: str):
        """Handle move confirmation from Arduino"""
        # Extract move from response and update game state
        # This would parse the move and update the chess engine
        pass
    
    def _game_loop(self):
        """Main game loop (called periodically)"""
        try:
            with self.state_lock:
                if self.current_state == GameState.COMPUTER_TURN:
                    self._handle_computer_turn()
                elif self.current_state == GameState.HUMAN_TURN:
                    self._handle_human_turn()
                elif self.current_state == GameState.GAME_OVER:
                    self._handle_game_over()
        
        except Exception as e:
            self.get_logger().error(f"Game loop error: {e}")
    
    def _handle_computer_turn(self):
        """Handle computer's turn"""
        if not self.waiting_for_move:
            self.waiting_for_move = True
            self.last_move_time = time.time()
            
            # Request best move from chess engine
            self._request_computer_move()
    
    def _handle_human_turn(self):
        """Handle human's turn"""
        # Check for move timeout
        if (self.last_move_time and 
            time.time() - self.last_move_time > self.move_timeout):
            self._show_status("Move timeout - your turn!")
            self.last_move_time = time.time()
    
    def _handle_game_over(self):
        """Handle game over state"""
        # Could show final position, offer new game, etc.
        pass
    
    def _request_computer_move(self):
        """Request best move from chess engine"""
        # This would be implemented as an async service call
        # For now, just a placeholder
        self.get_logger().info("Requesting computer move...")
    
    def _send_lcd_command(self, command_type: int, **kwargs):
        """Send command to LCD display"""
        msg = LCDCommand()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.command_type = command_type
        
        # Set message fields based on kwargs
        for key, value in kwargs.items():
            if hasattr(msg, key):
                setattr(msg, key, value)
        
        self.lcd_command_publisher.publish(msg)
    
    def _send_arduino_command(self, command_type: int, data: str = "", target_arduino: int = 0):
        """Send command to Arduino"""
        msg = ArduinoCommand()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.command_type = command_type
        msg.data = data
        msg.target_arduino = target_arduino
        
        self.arduino_command_publisher.publish(msg)
    
    def _show_status(self, status: str):
        """Show status message on display"""
        self._send_lcd_command(
            LCDCommand.CMD_DISPLAY_GAME_STATUS,
            text=status
        )
        self.get_logger().info(f"Status: {status}")

    def _publish_game_event(self, event: str):
        """Publish game event for robot animation controller"""
        try:
            msg = String()
            msg.data = event
            self.game_events_publisher.publish(msg)
            self.get_logger().info(f"Published game event: {event}")
        except Exception as e:
            self.get_logger().error(f"Error publishing game event: {e}")

    def _update_chess_engine_state(self):
        """Update chess engine with current game state"""
        # This would call the UpdateGameState service
        # For now, just log the action
        self.get_logger().info(f"Updating chess engine state: {self.current_fen}")
    
    def _shutdown_system(self):
        """Shutdown the system"""
        self._show_status("Shutting down...")
        self.get_logger().info("System shutdown requested")
        
        # Could trigger system shutdown here
        # For now, just return to menu
        self.shutdown_timer = self.create_timer(3.0, self._show_main_menu_after_shutdown)
    
    def _enter_manual_move_mode(self):
        """Enter manual move entry mode"""
        self._show_status("Enter move manually")
        # This would implement the manual move entry logic
        # from the original Menu.py get_manual_override method
    
    def _handle_game_button_press(self):
        """Handle button press during game"""
        # Could trigger manual move entry, pause game, etc.
        self.get_logger().info("Game button press - entering manual mode")
        self._enter_manual_move_mode()


def main(args=None):
    """Main entry point for the game management node"""
    rclpy.init(args=args)
    
    try:
        node = GameManagementNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start game management node: {e}")
        return 1
    
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
    
    return 0


if __name__ == '__main__':
    exit(main())
