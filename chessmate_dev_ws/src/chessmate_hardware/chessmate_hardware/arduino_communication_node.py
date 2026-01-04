#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Arduino Communication ROS 2 Node for ChessMate

This node handles serial communication with Arduino controllers
for board sensing and arm control. It's based on the serial
communication logic from ChessSoft.py but adapted for ROS 2.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import serial
import serial.tools.list_ports
import threading
import time
import queue
from typing import Optional, Dict, List, Callable
from enum import Enum

from chessmate_msgs.msg import ArduinoCommand, BoardState, ChessMove, ChessPiece
from chessmate_msgs.srv import ExecuteMove, SetBoardMode
from .gpio_abstraction import GPIOAbstraction
from std_msgs.msg import String


class ArduinoType(Enum):
    """Arduino controller types"""
    CHESSBOARD_CONTROLLER = 0
    ROBOT_CONTROLLER = 1


class ArduinoCommunicationNode(Node):
    """
    ROS 2 Node for Arduino serial communication
    
    This node:
    - Manages serial connections to multiple Arduino controllers
    - Subscribes to ArduinoCommand messages for sending commands
    - Publishes BoardState messages from board sensor feedback
    - Handles command queuing and response processing
    - Provides mock serial interface for development
    """
    
    def __init__(self):
        super().__init__('arduino_communication_node')
        
        # Declare parameters (using udev symlinks for persistent naming)
        self.declare_parameter('chessboard_controller_port', '/dev/chessboard')
        self.declare_parameter('robot_controller_port', '/dev/robot')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 1.0)
        self.declare_parameter('use_mock_hardware', not GPIOAbstraction.is_raspberry_pi())
        self.declare_parameter('command_timeout', 5.0)
        self.declare_parameter('heartbeat_interval', 10.0)

        # Get parameters
        self.chessboard_port = self.get_parameter('chessboard_controller_port').value
        self.robot_port = self.get_parameter('robot_controller_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        use_mock_hardware = self.get_parameter('use_mock_hardware').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        
        self.get_logger().info(f"Arduino Communication Node starting")
        self.get_logger().info(f"Chessboard controller: {self.chessboard_port}, Robot controller: {self.robot_port}")

        # Serial connection management
        self.use_mock_hardware = use_mock_hardware
        self.serial_connections: Dict[ArduinoType, serial.Serial] = {}
        self.command_queues: Dict[ArduinoType, queue.Queue] = {
            ArduinoType.CHESSBOARD_CONTROLLER: queue.Queue(),
            ArduinoType.ROBOT_CONTROLLER: queue.Queue()
        }
        self.response_callbacks: Dict[str, Callable] = {}
        
        # Thread management
        self.communication_threads: Dict[ArduinoType, threading.Thread] = {}
        self.running = True
        self.thread_lock = threading.Lock()
        
        # Initialize serial connections
        self._init_serial_connections()
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers
        self.arduino_command_subscriber = self.create_subscription(
            ArduinoCommand,
            'arduino_commands',
            self._arduino_command_callback,
            qos_profile
        )
        
        # Create publishers
        self.board_state_publisher = self.create_publisher(
            BoardState,
            'board_state',
            qos_profile
        )
        
        self.arduino_response_publisher = self.create_publisher(
            String,
            'arduino_responses',
            qos_profile
        )

        # Create services
        self.execute_move_service = self.create_service(
            ExecuteMove,
            'robot/execute_move',
            self._execute_move_callback
        )

        self.set_board_mode_service = self.create_service(
            SetBoardMode,
            'chessboard/set_mode',
            self._set_board_mode_callback
        )

        # Start communication threads
        self._start_communication_threads()
        
        # Create heartbeat timer
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval,
            self._send_heartbeat
        )
        
        self.get_logger().info("Arduino Communication Node initialized successfully")
    
    def _detect_serial_hardware(self) -> bool:
        """
        Detect if serial hardware is available
        
        Returns:
            True if serial ports detected, False otherwise
        """
        try:
            ports = list(serial.tools.list_ports.comports())
            available_ports = [port.device for port in ports]
            
            self.get_logger().info(f"Available serial ports: {available_ports}")
            
            # Check if our configured ports exist
            return (self.board_port in available_ports or 
                   self.arm_port in available_ports or
                   len(available_ports) > 0)
        except Exception as e:
            self.get_logger().warning(f"Serial port detection failed: {e}")
            return False
    
    def _init_serial_connections(self):
        """Initialize serial connections to Arduino controllers"""
        if self.use_mock_hardware:
            self._init_mock_serial()
        else:
            self._init_real_serial()
    
    def _init_real_serial(self):
        """Initialize real serial connections"""
        try:
            # Initialize chessboard controller connection
            try:
                self.serial_connections[ArduinoType.CHESSBOARD_CONTROLLER] = serial.Serial(
                    self.chessboard_port,
                    self.baud_rate,
                    timeout=self.timeout
                )
                self.get_logger().info(f"Chessboard controller connected on {self.chessboard_port}")
            except Exception as e:
                self.get_logger().warning(f"Failed to connect to chessboard controller: {e}")

            # Initialize robot controller connection
            try:
                self.serial_connections[ArduinoType.ROBOT_CONTROLLER] = serial.Serial(
                    self.robot_port,
                    self.baud_rate,
                    timeout=self.timeout
                )
                self.get_logger().info(f"Robot controller connected on {self.robot_port}")
            except Exception as e:
                self.get_logger().warning(f"Failed to connect to robot controller: {e}")
                
        except Exception as e:
            self.get_logger().error(f"Serial initialization failed: {e}, using mock serial")
            self.use_mock_hardware = True
            self._init_mock_serial()
    
    def _init_mock_serial(self):
        """Initialize mock serial connections for development"""
        self.serial_connections[ArduinoType.CHESSBOARD_CONTROLLER] = MockSerial("Chessboard Controller")
        self.serial_connections[ArduinoType.ROBOT_CONTROLLER] = MockSerial("Robot Controller")
        self.get_logger().info("Mock serial connections initialized for development")
    
    def _start_communication_threads(self):
        """Start communication threads for each Arduino"""
        for arduino_type in ArduinoType:
            if arduino_type in self.serial_connections:
                thread = threading.Thread(
                    target=self._communication_loop,
                    args=(arduino_type,),
                    daemon=True
                )
                self.communication_threads[arduino_type] = thread
                thread.start()
                
                self.get_logger().info(f"Communication thread started for {arduino_type.name}")
    
    def _communication_loop(self, arduino_type: ArduinoType):
        """
        Main communication loop for an Arduino controller
        
        Args:
            arduino_type: Type of Arduino controller
        """
        serial_conn = self.serial_connections.get(arduino_type)
        command_queue = self.command_queues[arduino_type]
        
        if not serial_conn:
            self.get_logger().error(f"No serial connection for {arduino_type.name}")
            return
        
        self.get_logger().info(f"Communication loop started for {arduino_type.name}")
        
        while self.running:
            try:
                # Process outgoing commands
                try:
                    command_data = command_queue.get(timeout=0.1)
                    self._send_command(serial_conn, command_data, arduino_type)
                except queue.Empty:
                    pass
                
                # Process incoming responses
                if self._has_incoming_data(serial_conn):
                    response = self._read_response(serial_conn)
                    if response:
                        self._process_response(response, arduino_type)
                
                time.sleep(0.01)  # Small delay to prevent tight loop
                
            except Exception as e:
                self.get_logger().error(f"Communication loop error for {arduino_type.name}: {e}")
                time.sleep(1)  # Longer delay on error
    
    def _arduino_command_callback(self, msg: ArduinoCommand):
        """
        Handle Arduino command messages
        
        Args:
            msg: ArduinoCommand message
        """
        try:
            arduino_type = ArduinoType(msg.target_arduino)
            
            # Format command based on type (matching original ChessSoft.py format)
            command_str = self._format_command(msg)
            
            # Queue command for sending
            self.command_queues[arduino_type].put(command_str)
            
            self.get_logger().debug(f"Queued command for {arduino_type.name}: {command_str}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing Arduino command: {e}")
    
    def _format_command(self, msg: ArduinoCommand) -> str:
        """
        Format Arduino command message into string format
        
        Args:
            msg: ArduinoCommand message
            
        Returns:
            Formatted command string
        """
        command_map = {
            ArduinoCommand.CMD_INIT: "init",
            ArduinoCommand.CMD_OCCUPANCY: "occupancy",
            ArduinoCommand.CMD_LEGAL_MOVES: "legal",
            ArduinoCommand.CMD_HINT: "hint",
            ArduinoCommand.CMD_CHECK: "check",
            ArduinoCommand.CMD_START_MOVE: "start",
            ArduinoCommand.CMD_COMPUTER_MOVE: "comp",
            ArduinoCommand.CMD_OVERRIDE_MOVE: "override",
            ArduinoCommand.CMD_RESET: "reset",
            ArduinoCommand.CMD_CHECKMATE: "checkmate",
            ArduinoCommand.CMD_HEARTBEAT: "heartbeat",

            # Robot animation commands (sent as single characters to robot controller)
            ArduinoCommand.CMD_ROBOT_WAKE_UP: "i",
            ArduinoCommand.CMD_ROBOT_DOZE_OFF: "s",
            ArduinoCommand.CMD_ROBOT_THINK_HARD: "i",  # Wake up triggers think hard
            ArduinoCommand.CMD_ROBOT_RESET_POSE: "z",
            ArduinoCommand.CMD_ROBOT_HOME_Z: "j",
            ArduinoCommand.CMD_ROBOT_HOME_ALL: "z",
            ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE: "move",  # Special handling for 6-char moves

            # Game flow commands (for chessboard controller)
            ArduinoCommand.CMD_GAME_COMPUTER_TURN: "comp_turn",
            ArduinoCommand.CMD_GAME_HUMAN_TURN: "human_turn",
            ArduinoCommand.CMD_GAME_MOVE_INVALID: "invalid",
            ArduinoCommand.CMD_GAME_CHECKMATE: "checkmate"
        }
        
        command_name = command_map.get(msg.command_type, "unknown")

        if command_name == "unknown":
            self.get_logger().warning(f"Unknown command type: {msg.command_type}")
            return "unknown\n"

        # Special handling for robot commands
        if msg.target_arduino == ArduinoType.ROBOT_CONTROLLER.value:
            return self._format_robot_command(msg, command_name)

        # Format chessboard controller commands
        if msg.data:
            # Special handling for occupancy command - convert algebraic to numeric
            if msg.command_type == ArduinoCommand.CMD_OCCUPANCY:
                formatted_data = self._format_occupancy_data(msg.data)
                return f"{command_name}:{formatted_data}\n"
            else:
                return f"{command_name}:{msg.data}\n"
        else:
            return f"{command_name}\n"

    def _format_robot_command(self, msg, command_name: str) -> str:
        """
        Format commands for robot controller Arduino

        Robot controller expects:
        - Single character indications: 'i', 'j', 'z', 's'
        - 6-character move commands: e.g., "e2pe4p"

        Args:
            msg: ArduinoCommand message
            command_name: Mapped command name

        Returns:
            Formatted command string
        """
        try:
            # Handle move execution specially
            if msg.command_type == ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE:
                if msg.data and len(msg.data) == 6:
                    # Send 6-character move directly
                    return f"{msg.data}\n"
                else:
                    self.get_logger().error(f"Invalid move format: {msg.data}")
                    return ""

            # Handle single-character indications
            robot_single_char_commands = {
                ArduinoCommand.CMD_ROBOT_WAKE_UP: "i",
                ArduinoCommand.CMD_ROBOT_DOZE_OFF: "s",
                ArduinoCommand.CMD_ROBOT_HOME_Z: "j",
                ArduinoCommand.CMD_ROBOT_HOME_ALL: "z",
                ArduinoCommand.CMD_ROBOT_RESET_POSE: "z"
            }

            if msg.command_type in robot_single_char_commands:
                return f"{robot_single_char_commands[msg.command_type]}\n"

            # Default formatting for other robot commands
            if msg.data:
                return f"{command_name}:{msg.data}\n"
            else:
                return f"{command_name}\n"

        except Exception as e:
            self.get_logger().error(f"Error formatting robot command: {e}")
            return ""

    def _send_command(self, serial_conn, command: str, arduino_type: ArduinoType):
        """
        Send command to Arduino
        
        Args:
            serial_conn: Serial connection
            command: Command string to send
            arduino_type: Arduino type
        """
        try:
            if hasattr(serial_conn, 'write'):
                serial_conn.write(command.encode('utf-8'))
                self.get_logger().debug(f"Sent to {arduino_type.name}: {command.strip()}")
            else:
                serial_conn.send(command)
                
        except Exception as e:
            self.get_logger().error(f"Failed to send command to {arduino_type.name}: {e}")
    
    def _has_incoming_data(self, serial_conn) -> bool:
        """
        Check if there's incoming data
        
        Args:
            serial_conn: Serial connection
            
        Returns:
            True if data is available
        """
        try:
            if hasattr(serial_conn, 'in_waiting'):
                return serial_conn.in_waiting > 0
            else:
                return serial_conn.has_data()
        except:
            return False
    
    def _read_response(self, serial_conn) -> Optional[str]:
        """
        Read response from Arduino
        
        Args:
            serial_conn: Serial connection
            
        Returns:
            Response string or None
        """
        try:
            if hasattr(serial_conn, 'readline'):
                response = serial_conn.readline().decode('utf-8').strip()
            else:
                response = serial_conn.receive()
            
            return response if response else None
            
        except Exception as e:
            self.get_logger().error(f"Failed to read response: {e}")
            return None
    
    def _process_response(self, response: str, arduino_type: ArduinoType):
        """
        Process response from Arduino
        
        Args:
            response: Response string
            arduino_type: Arduino type that sent the response
        """
        try:
            self.get_logger().debug(f"Received from {arduino_type.name}: {response}")
            
            # Publish raw response
            response_msg = String()
            response_msg.data = f"{arduino_type.name}:{response}"
            self.arduino_response_publisher.publish(response_msg)
            
            # Process specific response types
            if arduino_type == ArduinoType.CHESSBOARD_CONTROLLER:
                self._process_chessboard_response(response)
            elif arduino_type == ArduinoType.ROBOT_CONTROLLER:
                self._process_robot_response(response)
                
        except Exception as e:
            self.get_logger().error(f"Error processing response: {e}")
    
    def _process_chessboard_response(self, response: str):
        """
        Process chessboard controller response

        Arduino sends various response types:
        - Move strings: "e2e4", "a1h8", etc. (4 characters)
        - Special codes: "ffff" (hint override)
        - Single character indications: "i", "j", "z"
        - Debug messages: "Host initialized", "no mem", etc.

        Args:
            response: Response from chessboard controller
        """
        try:
            response = response.strip()

            # Check for move responses (4 character algebraic notation)
            if len(response) == 4 and response.isalnum() and response.islower():
                self._parse_move_response(response)
            # Check for special codes
            elif response == "ffff":
                self.get_logger().info("Board: Hint override detected")
            # Check for single character indications
            elif response in ["i", "j", "z"]:
                indication_map = {
                    "i": "Computer turn started",
                    "j": "Move validation failed - reset requested",
                    "z": "Checkmate detected"
                }
                self.get_logger().info(f"Board indication: {indication_map.get(response, response)}")
            # Check for debug/status messages
            elif "Host initialized" in response:
                self.get_logger().info("Chessboard controller initialized successfully")
            elif "no mem" in response:
                self.get_logger().error("Chessboard controller: Memory allocation failed")
            else:
                self.get_logger().info(f"Chessboard response: {response}")

        except Exception as e:
            self.get_logger().error(f"Error parsing chessboard response: {e}")

    def _parse_occupancy_response(self, response: str):
        """Parse board occupancy response and publish BoardState"""
        try:
            # Format: "occupancy_ack:square1:square2:square3..."
            parts = response.split(":")
            if len(parts) > 1:
                occupied_squares = parts[1:]

                # Create and publish BoardState message
                board_msg = BoardState()
                current_time = self.get_clock().now()
                board_msg.timestamp = int(current_time.nanoseconds)

                # Initialize board state with 64 empty squares
                squares = []
                for i in range(64):
                    piece = ChessPiece()
                    piece.type = "none"  # Use "none" for empty squares
                    piece.color = "empty"
                    piece.has_moved = False
                    piece.square_id = i
                    squares.append(piece)
                board_msg.squares = squares

                board_msg.active_color = "white"
                board_msg.castling_rights = "KQkq"
                board_msg.en_passant_target = "-"
                board_msg.halfmove_clock = 0
                board_msg.fullmove_number = 1
                board_msg.fen_string = ""  # Would need to reconstruct FEN from occupancy

                self.board_state_publisher.publish(board_msg)
                self.get_logger().debug(f"Published board occupancy: {occupied_squares}")

        except Exception as e:
            self.get_logger().error(f"Error parsing occupancy response: {e}")

    def _parse_move_response(self, move: str):
        """
        Parse move confirmation response

        Args:
            move: 4-character move string (e.g., "e2e4")
        """
        try:
            self.get_logger().info(f"Move confirmed by board: {move}")

            # Create and publish BoardState message with move
            board_msg = BoardState()
            current_time = self.get_clock().now()
            board_msg.timestamp = int(current_time.nanoseconds)

            # Initialize board state with 64 empty squares
            squares = []
            for i in range(64):
                piece = ChessPiece()
                piece.type = "none"  # Use "none" for empty squares
                piece.color = "empty"
                piece.has_moved = False
                piece.square_id = i
                squares.append(piece)
            board_msg.squares = squares

            board_msg.active_color = "white"
            board_msg.castling_rights = "KQkq"
            board_msg.en_passant_target = "-"
            board_msg.halfmove_clock = 0
            board_msg.fullmove_number = 1
            board_msg.fen_string = ""

            self.board_state_publisher.publish(board_msg)

        except Exception as e:
            self.get_logger().error(f"Error parsing move response: {e}")

    def _format_occupancy_data(self, data: str) -> str:
        """
        Convert occupancy data from algebraic notation to numeric format expected by Arduino

        Args:
            data: Colon-separated algebraic squares (e.g., "a1:b2:c3")

        Returns:
            Colon-separated numeric indices (e.g., "0:9:18")
        """
        try:
            if not data:
                return ""

            squares = data.split(":")
            numeric_indices = []

            for square in squares:
                if len(square) == 2:
                    col = ord(square[0].lower()) - ord('a')  # a=0, b=1, ..., h=7
                    row = int(square[1]) - 1  # 1=0, 2=1, ..., 8=7

                    if 0 <= col <= 7 and 0 <= row <= 7:
                        index = row * 8 + col  # Convert to 0-63 index
                        numeric_indices.append(str(index))
                    else:
                        self.get_logger().warning(f"Invalid square notation: {square}")
                else:
                    self.get_logger().warning(f"Invalid square format: {square}")

            return ":".join(numeric_indices)

        except Exception as e:
            self.get_logger().error(f"Error formatting occupancy data: {e}")
            return data  # Return original data if conversion fails

    def _process_robot_response(self, response: str):
        """
        Process robot controller response

        Args:
            response: Response from robot controller
        """
        # Parse robot controller responses
        self.get_logger().info(f"Robot response: {response}")

        # TODO: Parse robot status, movement completion, etc.
    
    def _send_heartbeat(self):
        """Send periodic heartbeat to Arduino controllers"""
        heartbeat_command = "heartbeat\n"
        
        for arduino_type in ArduinoType:
            if arduino_type in self.command_queues:
                try:
                    self.command_queues[arduino_type].put(heartbeat_command)
                except:
                    pass  # Queue might be full, skip this heartbeat

    def _execute_move_callback(self, request, response):
        """Handle robot move execution service requests"""
        try:
            self.get_logger().info(f"Execute move request: {request.move.from_square} -> {request.move.to_square}")

            # Create Arduino command for robot move execution
            move_str = f"{request.move.from_square}{request.move.piece_type.lower()}{request.move.to_square}{request.move.piece_type.lower()}"

            # Handle captures
            if hasattr(request.move, 'captured_piece_type') and request.move.captured_piece_type:
                move_str = f"{request.move.from_square}{request.move.piece_type.lower()}{request.move.to_square}x"

            # Send command to robot controller
            if ArduinoType.ROBOT_CONTROLLER in self.command_queues:
                self.command_queues[ArduinoType.ROBOT_CONTROLLER].put(move_str + "\n")

                # Give a brief moment for command processing
                time.sleep(0.5)

                response.success = True
                response.message = f"Move {move_str} queued for execution"
                response.execution_time = 5.0  # Estimated time

                # Create a basic board state response
                current_time = self.get_clock().now()
                response.resulting_board_state.timestamp = int(current_time.nanoseconds)

                # Initialize board state with 64 empty squares
                squares = []
                for i in range(64):
                    piece = ChessPiece()
                    piece.type = "none"  # Use "none" for empty squares
                    piece.color = "empty"
                    piece.has_moved = False
                    piece.square_id = i
                    squares.append(piece)
                response.resulting_board_state.squares = squares

                response.resulting_board_state.active_color = "white"
                response.resulting_board_state.castling_rights = "KQkq"
                response.resulting_board_state.en_passant_target = "-"
                response.resulting_board_state.halfmove_clock = 0
                response.resulting_board_state.fullmove_number = 1
                response.resulting_board_state.fen_string = ""

                self.get_logger().info(f"✅ Execute move service completed: {move_str}")

            else:
                response.success = False
                response.message = "Robot controller not available"
                response.execution_time = 0.0

        except Exception as e:
            self.get_logger().error(f"Execute move service error: {e}")
            response.success = False
            response.message = f"Service error: {str(e)}"
            response.execution_time = 0.0

        return response

    def _set_board_mode_callback(self, request, response):
        """Handle board mode setting service requests"""
        try:
            self.get_logger().info(f"Set board mode request: {request.mode}")

            # Map mode constants to string commands
            mode_map = {
                0: "idle",      # MODE_IDLE
                1: "setup",     # MODE_SETUP
                2: "playing",   # MODE_PLAYING
                3: "analysis",  # MODE_ANALYSIS
                4: "calibration" # MODE_CALIBRATION
            }

            mode_str = mode_map.get(request.mode, "idle")

            # Send mode command to chessboard controller
            if ArduinoType.CHESSBOARD_CONTROLLER in self.command_queues:
                mode_command = f"mode:{mode_str}\n"
                self.command_queues[ArduinoType.CHESSBOARD_CONTROLLER].put(mode_command)

                # Give a brief moment for command processing
                time.sleep(0.5)

                response.success = True
                response.previous_mode = 0  # Would track actual previous mode
                response.current_mode = request.mode
                response.message = f"Board mode set to {mode_str}"

                self.get_logger().info(f"✅ Board mode service completed: {mode_str}")

            else:
                response.success = False
                response.previous_mode = 0
                response.current_mode = 0
                response.message = "Chessboard controller not available"

        except Exception as e:
            self.get_logger().error(f"Set board mode service error: {e}")
            response.success = False
            response.previous_mode = 0
            response.current_mode = 0
            response.message = f"Service error: {str(e)}"

        return response

    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        try:
            self.get_logger().info("Shutting down Arduino Communication Node")
            
            # Stop communication threads
            self.running = False
            
            # Wait for threads to finish
            for thread in self.communication_threads.values():
                if thread.is_alive():
                    thread.join(timeout=2.0)
            
            # Close serial connections
            for serial_conn in self.serial_connections.values():
                if hasattr(serial_conn, 'close'):
                    serial_conn.close()
                elif hasattr(serial_conn, 'cleanup'):
                    serial_conn.cleanup()
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        
        super().destroy_node()


class MockSerial:
    """Mock serial connection for development"""
    
    def __init__(self, name: str):
        self.name = name
        self.data_queue = queue.Queue()
        self.response_queue = queue.Queue()
        
        # Start mock response generator
        self.response_thread = threading.Thread(target=self._generate_responses, daemon=True)
        self.response_thread.start()
    
    def write(self, data: bytes):
        """Mock write operation"""
        command = data.decode('utf-8').strip()
        print(f"Mock {self.name}: Received command '{command}'")
        self.data_queue.put(command)
    
    def send(self, data: str):
        """Mock send operation"""
        print(f"Mock {self.name}: Received command '{data.strip()}'")
        self.data_queue.put(data.strip())
    
    def has_data(self) -> bool:
        """Check if mock data is available"""
        return not self.response_queue.empty()
    
    def receive(self) -> str:
        """Receive mock response"""
        try:
            return self.response_queue.get_nowait()
        except queue.Empty:
            return ""
    
    def readline(self) -> bytes:
        """Mock readline operation"""
        try:
            response = self.response_queue.get(timeout=0.1)
            return response.encode('utf-8')
        except queue.Empty:
            return b""
    
    @property
    def in_waiting(self) -> int:
        """Mock in_waiting property"""
        return self.response_queue.qsize()
    
    def close(self):
        """Mock close operation"""
        print(f"Mock {self.name}: Connection closed")
    
    def _generate_responses(self):
        """Generate mock responses"""
        while True:
            try:
                command = self.data_queue.get(timeout=1.0)
                
                # Generate appropriate mock responses
                if command.startswith("occupancy"):
                    response = "occupancy_ack:1:2:3:4"
                elif command.startswith("legal"):
                    response = "legal_ack:e2e4:d2d4:g1f3"
                elif command.startswith("comp"):
                    response = "comp_ack:e2e4"
                elif command.startswith("start"):
                    response = "start_ack"
                elif command.startswith("heartbeat"):
                    response = "heartbeat_ack"
                else:
                    response = f"ack:{command}"
                
                time.sleep(0.5)  # Simulate processing delay
                self.response_queue.put(response)
                
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Mock {self.name} response generator error: {e}")


def main(args=None):
    """Main entry point for the Arduino communication node"""
    rclpy.init(args=args)
    
    try:
        node = ArduinoCommunicationNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start Arduino communication node: {e}")
        return 1
    
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
    
    return 0


if __name__ == '__main__':
    exit(main())
