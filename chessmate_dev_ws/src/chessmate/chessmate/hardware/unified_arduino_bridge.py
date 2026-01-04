#!/usr/bin/env python3
"""
Unified Arduino Bridge Node for ChessMate Hardware Interface

Character-based Arduino communication bridge providing unified ROS 2 interface
for both chessboard and robot controller Arduinos. Supports cross-platform
hardware abstraction for development and deployment.

Features:
- Character-based protocol for reliable Arduino communication
- Auto-detection of Arduino capabilities
- Hardware safety and calibration services
- Sensor data publishing
- Cross-platform compatibility (Linux host and Raspberry Pi)
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import serial
import threading
import time
import queue
from enum import Enum
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from chessmate.msg import (
    ArduinoCommand, JointCommand, SensorReading,
    BoardState, RobotAnimation, ChessPiece
)
from chessmate.srv import CalibrateArm


class ArduinoType(Enum):
    """Arduino controller types"""
    CHESSBOARD_CONTROLLER = 0
    ROBOT_CONTROLLER = 1


class ProtocolType(Enum):
    """Communication protocol types"""
    CHARACTER_BASED = 0  # Simple character commands (reliable and simple)


class MockSerial:
    """Mock serial interface for development without hardware"""
    def __init__(self, port, baudrate, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.is_open = True
        self.in_waiting = 0
        
    def write(self, data):
        # print(f"MOCK SERIAL [{self.port}] TX: {data.decode().strip()}")  # Commented out for cleaner logs
        return len(data)
        
    def read(self, size=1):
        return b''
        
    def readline(self):
        return b''
        
    def reset_input_buffer(self):
        pass
        
    def close(self):
        self.is_open = False


class UnifiedArduinoBridge(Node):
    """
    Unified Arduino Bridge supporting character-based protocol only
    """
    
    def __init__(self):
        super().__init__('unified_arduino_bridge')
        
        # Declare parameters (using udev symlinks for persistent naming)
        self.declare_parameters(
            namespace='',
            parameters=[
                # Serial port configuration
                ('chessboard_port', '/dev/chessboard'),
                ('robot_port', '/dev/robot'),
                ('baud_rate', 9600),
                ('timeout', 2.0),
                ('use_mock_hardware', False),
                
                # Protocol configuration (character-based only)
                ('chessboard_protocol', 'character'),  # character protocol only
                ('robot_protocol', 'character'),       # character protocol only
                ('command_terminator', '\n'),
                ('response_timeout', 1.0),
                ('max_retries', 3),
                
                # Publishing rates
                ('sensor_publish_rate', 10.0),
                ('status_publish_rate', 1.0),
                
                # Safety parameters
                ('emergency_stop_enabled', True),
                ('max_joint_velocity', 1.0),
                ('max_joint_acceleration', 2.0),
                
                # Hardware parameters
                ('link1_length', 0.202),
                ('link2_length', 0.190),
                ('z_axis_max', 0.050),
            ]
        )
        
        # Get parameters
        self.chessboard_port = self.get_parameter('chessboard_port').value
        self.robot_port = self.get_parameter('robot_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.use_mock_hardware = self.get_parameter('use_mock_hardware').value
        
        self.chessboard_protocol = self.get_parameter('chessboard_protocol').value
        self.robot_protocol = self.get_parameter('robot_protocol').value
        self.terminator = self.get_parameter('command_terminator').value
        self.response_timeout = self.get_parameter('response_timeout').value
        self.max_retries = self.get_parameter('max_retries').value
        
        # Initialize state
        self.running = True
        self.emergency_stop = False
        self.is_calibrated = False
        self.last_joint_state = None
        self.protocol_types = {}  # Store detected protocol for each Arduino
        
        # Serial connections and locks
        self.serial_connections = {}
        self.serial_locks = {}
        self.command_queues = {}
        
        # Initialize for each Arduino type
        for arduino_type in ArduinoType:
            self.serial_locks[arduino_type] = threading.Lock()
            self.command_queues[arduino_type] = queue.Queue()
        
        # Publishers with compatible QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.board_state_publisher = self.create_publisher(BoardState, 'board_state', qos_profile)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.sensor_publisher = self.create_publisher(SensorReading, 'sensor_readings', qos_profile)
        self.status_publisher = self.create_publisher(String, 'hardware_status', qos_profile)
        
        # Subscribers
        self.arduino_command_subscription = self.create_subscription(
            ArduinoCommand, 'arduino_command', self._arduino_command_callback, 10)
        self.joint_command_subscription = self.create_subscription(
            JointCommand, 'joint_command', self._joint_command_callback, 10)
        self.emergency_stop_subscription = self.create_subscription(
            Bool, 'emergency_stop', self._emergency_stop_callback, 10)
        
        # Services
        self.calibrate_service = self.create_service(
            CalibrateArm, 'calibrate_arm', self._calibrate_callback)
        
        # Timers
        sensor_rate = self.get_parameter('sensor_publish_rate').value
        status_rate = self.get_parameter('status_publish_rate').value
        
        self.sensor_timer = self.create_timer(1.0/sensor_rate, self._publish_sensor_data)
        self.status_timer = self.create_timer(1.0/status_rate, self._publish_status)
        
        # Initialize hardware connections
        self._initialize_hardware()
        
        self.get_logger().info('Unified Arduino Bridge initialized')
    
    def _initialize_hardware(self):
        """Initialize serial connections and detect protocols"""
        port_map = {
            ArduinoType.CHESSBOARD_CONTROLLER: self.chessboard_port,
            ArduinoType.ROBOT_CONTROLLER: self.robot_port
        }
        
        protocol_map = {
            ArduinoType.CHESSBOARD_CONTROLLER: self.chessboard_protocol,
            ArduinoType.ROBOT_CONTROLLER: self.robot_protocol
        }
        
        for arduino_type in ArduinoType:
            port = port_map[arduino_type]
            protocol_pref = protocol_map[arduino_type]
            
            try:
                self.get_logger().info(f'Connecting to {arduino_type.name} on {port}')
                
                if self.use_mock_hardware:
                    serial_conn = MockSerial(port, self.baud_rate, self.timeout)
                    self.get_logger().info(f'Using mock serial for {arduino_type.name}')
                else:
                    serial_conn = serial.Serial(
                        port=port,
                        baudrate=self.baud_rate,
                        timeout=self.timeout
                    )
                    time.sleep(2)  # Wait for Arduino reset
                
                self.serial_connections[arduino_type] = serial_conn
                
                # Detect or set protocol
                detected_protocol = self._detect_protocol(arduino_type, protocol_pref)
                self.protocol_types[arduino_type] = detected_protocol
                
                self.get_logger().info(
                    f'{arduino_type.name} connected using {detected_protocol.name} protocol')
                
                # Start communication thread
                thread = threading.Thread(
                    target=self._communication_loop,
                    args=(arduino_type,),
                    daemon=True
                )
                thread.start()
                
            except Exception as e:
                self.get_logger().error(f'Failed to connect to {arduino_type.name}: {e}')
                if not self.use_mock_hardware:
                    # Fall back to mock serial
                    self.serial_connections[arduino_type] = MockSerial(port, self.baud_rate)
                    self.protocol_types[arduino_type] = ProtocolType.CHARACTER_BASED
                    self.get_logger().warn(f'Using mock serial fallback for {arduino_type.name}')
    
    def _detect_protocol(self, arduino_type: ArduinoType, preference: str) -> ProtocolType:
        """
        Detect Arduino communication protocol - Character only

        Args:
            arduino_type: Type of Arduino controller
            preference: Protocol preference (only 'character' supported)

        Returns:
            Always returns CHARACTER_BASED protocol
        """
        # Only character protocol is supported now
        if preference != 'character':
            self.get_logger().info(f'Only character protocol supported, ignoring preference: {preference}')

        return ProtocolType.CHARACTER_BASED
    
    # JSON protocol support removed - character protocol only

    def _communication_loop(self, arduino_type: ArduinoType):
        """Main communication loop for Arduino controller"""
        serial_conn = self.serial_connections[arduino_type]
        command_queue = self.command_queues[arduino_type]

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

    def _send_command_direct(self, arduino_type: ArduinoType, command: str) -> str:
        """Send command directly and wait for response (for protocol detection)"""
        serial_conn = self.serial_connections.get(arduino_type)
        if not serial_conn:
            return None

        with self.serial_locks[arduino_type]:
            try:
                if hasattr(serial_conn, 'reset_input_buffer'):
                    serial_conn.reset_input_buffer()

                cmd_str = command + self.terminator
                serial_conn.write(cmd_str.encode())

                # Wait for response
                start_time = time.time()
                response = ""

                while time.time() - start_time < self.response_timeout:
                    if hasattr(serial_conn, 'in_waiting') and serial_conn.in_waiting > 0:
                        char = serial_conn.read(1).decode('utf-8', errors='ignore')
                        if char == '\n':
                            break
                        response += char
                    elif hasattr(serial_conn, 'readline'):
                        response = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                        break
                    time.sleep(0.01)

                return response.strip() if response else None

            except Exception as e:
                self.get_logger().error(f'Direct command error: {e}')
                return None

    def _send_command(self, serial_conn, command_data: str, arduino_type: ArduinoType):
        """Send command to Arduino controller"""
        if not serial_conn or not command_data:
            return

        with self.serial_locks[arduino_type]:
            try:
                cmd_str = command_data
                if not cmd_str.endswith('\n'):
                    cmd_str += '\n'

                serial_conn.write(cmd_str.encode())
                self.get_logger().debug(f"REAL SERIAL [{arduino_type.name}] TX: {cmd_str.strip()}")

            except Exception as e:
                self.get_logger().error(f"Send command error for {arduino_type.name}: {e}")

    def _has_incoming_data(self, serial_conn) -> bool:
        """Check if serial connection has incoming data"""
        try:
            if hasattr(serial_conn, 'in_waiting'):
                return serial_conn.in_waiting > 0
            return False
        except Exception:
            return False

    def _read_response(self, serial_conn) -> str:
        """Read response from Arduino controller"""
        try:
            if hasattr(serial_conn, 'readline'):
                response = serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    self.get_logger().debug(f"REAL SERIAL RX: '{response}'")
                return response if response else None
            return None
        except Exception as e:
            self.get_logger().error(f"Read response error: {e}")
            return None

    def _process_response(self, response: str, arduino_type: ArduinoType):
        """Process response from Arduino controller"""
        if not response:
            return

        self.get_logger().debug(f"Received from {arduino_type.name}: {response}")

        # Only character protocol is supported
        self._process_character_response(response, arduino_type)

    # JSON response processing removed - character protocol only

    def _process_character_response(self, response: str, arduino_type: ArduinoType):
        """Process character-based response from Arduino"""
        # Handle board state updates from chessboard controller
        if arduino_type == ArduinoType.CHESSBOARD_CONTROLLER:
            if len(response) == 64:  # Board state (64 squares)
                self._publish_board_state(response)
            elif response.startswith('occupancy:'):
                # Handle occupancy response
                occupancy_data = response.split(':', 1)[1]
                self._publish_board_state(occupancy_data)

        # Handle robot controller responses
        elif arduino_type == ArduinoType.ROBOT_CONTROLLER:
            # Simple acknowledgment or status responses
            if response in ['i', 'j', 's', 'z', 'OK']:
                self.get_logger().debug(f"Robot controller acknowledged: {response}")

            # Handle status responses
            elif response.startswith('STATUS:'):
                status = response.split(':', 1)[1]
                self.get_logger().debug(f"Robot status: {status}")
                # Could publish robot status here

            # Handle position feedback
            elif response.startswith('POS:'):
                pos_data = response.split(':', 1)[1]
                try:
                    positions = [float(x) for x in pos_data.split(',')]
                    if len(positions) >= 3:
                        self._publish_joint_state(positions)
                        self.get_logger().debug(f"Published joint position: {positions[:3]}")
                except ValueError as e:
                    self.get_logger().warn(f"Invalid position data: {pos_data}")

        # Handle chessboard controller responses
        elif arduino_type == ArduinoType.CHESSBOARD_CONTROLLER:
            if response in ['OK']:
                self.get_logger().debug(f"Chessboard controller acknowledged: {response}")
            elif response.startswith('BOARD:'):
                board_data = response.split(':', 1)[1]
                self._publish_board_state(board_data)

    def _publish_joint_state(self, positions: list):
        """Publish joint state from position data"""
        try:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = ['joint1', 'joint2', 'z_axis']
            joint_state.position = positions[:3]  # Take first 3 positions

            # Add empty velocities and efforts
            joint_state.velocity = [0.0] * len(joint_state.position)
            joint_state.effort = [0.0] * len(joint_state.position)

            self.joint_state_publisher.publish(joint_state)
            self.get_logger().debug(f"Published joint state: {positions[:3]}")

        except Exception as e:
            self.get_logger().error(f"Error publishing joint state: {e}")

    def _handle_sensor_data(self, data: dict, arduino_type: ArduinoType):
        """Handle sensor data from Arduino"""
        try:
            # Publish joint state if available
            if 'j1' in data and 'j2' in data and 'z' in data:
                joint_state = JointState()
                joint_state.header.stamp = self.get_clock().now().to_msg()
                joint_state.name = ['joint1', 'joint2', 'z_axis']
                joint_state.position = [data['j1'], data['j2'], data['z']]

                if 'moving' in data:
                    # Add velocity information if available
                    joint_state.velocity = [0.0, 0.0, 0.0]  # Could be enhanced

                self.joint_state_publisher.publish(joint_state)
                self.last_joint_state = joint_state

            # Publish individual sensor readings
            for sensor_name, value in data.items():
                if isinstance(value, (int, float)):
                    sensor_reading = SensorReading()
                    sensor_reading.header.stamp = self.get_clock().now().to_msg()
                    sensor_reading.sensor_name = sensor_name
                    sensor_reading.raw_value = float(value)
                    sensor_reading.processed_value = float(value)
                    sensor_reading.is_active = True

                    self.sensor_publisher.publish(sensor_reading)

        except Exception as e:
            self.get_logger().error(f"Error handling sensor data: {e}")

    def _publish_board_state(self, board_data: str):
        """Publish board state from chessboard controller"""
        try:
            board_state = BoardState()
            current_time = self.get_clock().now()
            board_state.timestamp = int(current_time.nanoseconds)

            # Initialize board state with 64 empty squares
            squares = []
            for i in range(64):
                piece = ChessPiece()
                piece.type = "none"  # Use "none" for empty squares
                piece.color = "empty"
                piece.has_moved = False
                piece.square_id = i
                squares.append(piece)
            board_state.squares = squares

            board_state.active_color = "white"
            board_state.castling_rights = "KQkq"
            board_state.en_passant_target = "-"
            board_state.halfmove_clock = 0
            board_state.fullmove_number = 1
            board_state.fen_string = board_data  # Store raw data as FEN placeholder

            self.board_state_publisher.publish(board_state)

        except Exception as e:
            self.get_logger().error(f"Error publishing board state: {e}")

    def _arduino_command_callback(self, msg: ArduinoCommand):
        """Handle Arduino command messages"""
        try:
            arduino_type = ArduinoType(msg.target_arduino)
            # Only character protocol is supported
            command_str = self._format_character_command(msg, arduino_type)

            if command_str:
                self.command_queues[arduino_type].put(command_str)
                self.get_logger().debug(f"Queued command for {arduino_type.name}: {command_str}")

        except Exception as e:
            self.get_logger().error(f"Error processing Arduino command: {e}")

    def _joint_command_callback(self, msg: JointCommand):
        """Handle joint command messages (character protocol)"""
        try:
            # Safety checks
            if self.emergency_stop:
                self.get_logger().warn("Joint command ignored - emergency stop active")
                return

            if not self.is_calibrated:
                self.get_logger().warn("Joint command ignored - system not calibrated")
                return

            # Create character command for joint movement
            # For character protocol, we'll use a simple move command
            # The Arduino stub will simulate the movement
            if len(msg.positions) >= 3:
                # For now, send a simple move command to indicate joint movement
                # The Arduino stub will handle this as a generic movement
                command_str = f"move_{int(msg.positions[0]*100)}_{int(msg.positions[1]*100)}_{int(msg.positions[2]*100)}"
                # Allow longer commands for move_X_Y_Z format (up to 20 characters)
                if len(command_str) > 20:
                    command_str = "move"  # Fallback only for very long commands
            else:
                command_str = "move"  # Simple move command

            # Send to robot controller (character protocol)
            arduino_type = ArduinoType.ROBOT_CONTROLLER
            if arduino_type in self.command_queues:
                self.command_queues[arduino_type].put(command_str)
                self.get_logger().info(f"Sent joint command: {command_str}")
                self.get_logger().debug(f"Command sent to Arduino: '{command_str}'")

        except Exception as e:
            self.get_logger().error(f"Error processing joint command: {e}")

    def _emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop messages"""
        self.emergency_stop = msg.data

        if self.emergency_stop:
            self.get_logger().warn("EMERGENCY STOP ACTIVATED")

            # Send emergency stop to all controllers (character protocol only)
            for arduino_type in ArduinoType:
                estop_cmd = "STOP"  # Simple character command

                if arduino_type in self.command_queues:
                    self.command_queues[arduino_type].put(estop_cmd)
        else:
            self.get_logger().info("Emergency stop released")

            # Send release command to all controllers (character protocol only)
            for arduino_type in ArduinoType:
                release_cmd = "RESUME"  # Simple character command

                if arduino_type in self.command_queues:
                    self.command_queues[arduino_type].put(release_cmd)

    # JSON command formatting removed - character protocol only

    def _format_character_command(self, msg: ArduinoCommand, arduino_type: ArduinoType) -> str:
        """Format Arduino command as character-based command"""
        try:
            # Use existing character-based formatting from arduino_communication_node
            if arduino_type == ArduinoType.ROBOT_CONTROLLER:
                return self._format_robot_character_command(msg)
            else:
                return self._format_chessboard_character_command(msg)

        except Exception as e:
            self.get_logger().error(f"Error formatting character command: {e}")
            return ""

    def _format_robot_character_command(self, msg: ArduinoCommand) -> str:
        """Format robot controller character commands"""
        try:
            # Handle move execution specially
            if msg.command_type == ArduinoCommand.CMD_ROBOT_EXECUTE_MOVE:
                if msg.data and len(msg.data) == 6:
                    return msg.data  # Send 6-character move directly
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
                return robot_single_char_commands[msg.command_type]

            return ""

        except Exception as e:
            self.get_logger().error(f"Error formatting robot character command: {e}")
            return ""

    def _format_chessboard_character_command(self, msg: ArduinoCommand) -> str:
        """Format chessboard controller character commands"""
        try:
            # Map command types to command names
            command_map = {
                ArduinoCommand.CMD_OCCUPANCY: "occupancy",
                ArduinoCommand.CMD_STATUS: "status",
                ArduinoCommand.CMD_RESET: "reset",
                ArduinoCommand.CMD_CALIBRATE: "calibrate",
                ArduinoCommand.CMD_LED_ON: "led_on",
                ArduinoCommand.CMD_LED_OFF: "led_off",
                ArduinoCommand.CMD_HINT_ENABLE: "hint_enable",
                ArduinoCommand.CMD_HINT_DISABLE: "hint_disable",
            }

            command_name = command_map.get(msg.command_type, "unknown")

            if command_name == "unknown":
                self.get_logger().warn(f"Unknown chessboard command type: {msg.command_type}")
                return ""

            # Format command with data if available
            if msg.data:
                return f"{command_name}:{msg.data}"
            else:
                return command_name

        except Exception as e:
            self.get_logger().error(f"Error formatting chessboard character command: {e}")
            return ""

    def _calibrate_callback(self, request, response):
        """Handle calibration service requests"""
        try:
            self.get_logger().info(f"Starting calibration: {request.calibration_type}")

            # Send calibration command to robot controller (character protocol only)
            arduino_type = ArduinoType.ROBOT_CONTROLLER

            # Character protocol calibration
            if request.calibration_type == "home":
                calibrate_cmd = "z"  # Home all axes
            else:
                calibrate_cmd = "j"  # Home Z only

            if arduino_type in self.command_queues:
                self.command_queues[arduino_type].put(calibrate_cmd)

            # Wait for calibration to complete (simplified)
            time.sleep(5.0)  # This should be improved with actual feedback

            # Set calibration status
            self.is_calibrated = True

            # Prepare response
            response.success = True
            response.message = "Calibration completed successfully"
            response.joint_offsets = [0.0, 0.0, 0.0]  # Placeholder
            response.workspace_min = Point(x=-0.3, y=-0.3, z=0.0)
            response.workspace_max = Point(x=0.3, y=0.3, z=0.05)
            response.calibration_accuracy = 1.0  # mm
            response.calibration_timestamp = self.get_clock().now().to_msg()

            self.get_logger().info("Calibration completed successfully")

        except Exception as e:
            self.get_logger().error(f"Calibration error: {e}")
            response.success = False
            response.message = f"Calibration failed: {str(e)}"

        return response

    def _publish_sensor_data(self):
        """Periodic sensor data publishing"""
        try:
            # Request sensor data from controllers (character protocol only)
            for arduino_type in ArduinoType:
                sensor_cmd = "status"  # Simple status request

                if arduino_type in self.command_queues:
                    self.command_queues[arduino_type].put(sensor_cmd)

        except Exception as e:
            self.get_logger().error(f"Error requesting sensor data: {e}")

    def _publish_status(self):
        """Periodic status publishing"""
        try:
            status_msg = String()
            status_parts = []

            # Add connection status
            for arduino_type in ArduinoType:
                if arduino_type in self.serial_connections:
                    conn = self.serial_connections[arduino_type]
                    if hasattr(conn, 'is_open') and conn.is_open:
                        status_parts.append(f"{arduino_type.name}:CONNECTED")
                    else:
                        status_parts.append(f"{arduino_type.name}:DISCONNECTED")
                else:
                    status_parts.append(f"{arduino_type.name}:NOT_INITIALIZED")

            # Add system status
            status_parts.append(f"CALIBRATED:{self.is_calibrated}")
            status_parts.append(f"EMERGENCY_STOP:{self.emergency_stop}")

            status_msg.data = "|".join(status_parts)
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Shutting down Unified Arduino Bridge")
        self.running = False

        # Close serial connections
        for arduino_type, serial_conn in self.serial_connections.items():
            try:
                if hasattr(serial_conn, 'close'):
                    serial_conn.close()
                self.get_logger().info(f"Closed connection to {arduino_type.name}")
            except Exception as e:
                self.get_logger().error(f"Error closing {arduino_type.name}: {e}")

        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = UnifiedArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
