#!/usr/bin/env python3
"""
ChessMate Controller Communication Test

This script validates USB Serial communication with both ChessBoard and Robot
controllers in mock mode, ensuring proper command/response patterns and timing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import serial
import time
import threading
from datetime import datetime
import sys

from chessmate.msg import BoardState, ChessMove, RobotStatus
from chessmate.srv import ValidateMove, GetBestMove


class ControllerCommunicationTest(Node):
    """Test node for validating controller communication"""
    
    def __init__(self):
        super().__init__('test_controller_communication')
        
        # Parameters
        self.declare_parameter('chessboard_port', '/dev/ttyACM0')
        self.declare_parameter('robot_port', '/dev/ttyACM1')
        self.declare_parameter('test_timeout', 30.0)
        
        self.chessboard_port = self.get_parameter('chessboard_port').value
        self.robot_port = self.get_parameter('robot_port').value
        self.test_timeout = self.get_parameter('test_timeout').value
        
        # Test results
        self.test_results = {
            'chessboard_connection': False,
            'robot_connection': False,
            'chessboard_commands': [],
            'robot_commands': [],
            'timing_tests': [],
            'overall_success': False
        }
        
        # Serial connections
        self.chessboard_serial = None
        self.robot_serial = None
        
        # ROS2 interfaces
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publishers for test results
        self.test_status_pub = self.create_publisher(
            BoardState, 'test_status', qos_profile
        )
        
        self.get_logger().info("Controller Communication Test initialized")
        
        # Start test sequence
        self.test_timer = self.create_timer(1.0, self.run_tests)
        self.test_started = False
    
    def run_tests(self):
        """Run the complete test sequence"""
        if self.test_started:
            return
            
        self.test_started = True
        self.get_logger().info("Starting controller communication tests...")
        
        try:
            # Test 1: Controller connections
            self.test_controller_connections()
            
            # Test 2: ChessBoard commands
            if self.test_results['chessboard_connection']:
                self.test_chessboard_commands()
            
            # Test 3: Robot commands
            if self.test_results['robot_connection']:
                self.test_robot_commands()
            
            # Test 4: Timing validation
            self.test_timing_requirements()
            
            # Generate final report
            self.generate_test_report()
            
        except Exception as e:
            self.get_logger().error(f"Test execution failed: {e}")
            self.test_results['overall_success'] = False
        
        finally:
            self.cleanup_connections()
            # Shutdown after test completion
            self.create_timer(2.0, lambda: rclpy.shutdown())
    
    def test_controller_connections(self):
        """Test USB Serial connections to both controllers"""
        self.get_logger().info("Testing controller connections...")
        
        # Test ChessBoard connection
        try:
            self.chessboard_serial = serial.Serial(
                self.chessboard_port, 9600, timeout=2.0
            )
            time.sleep(2)  # Allow connection to stabilize
            
            # Send test command
            self.chessboard_serial.write(b'mode:mock\n')
            self.chessboard_serial.flush()
            
            # Wait for response
            start_time = time.time()
            response_received = False
            
            while time.time() - start_time < 5.0:
                if self.chessboard_serial.in_waiting > 0:
                    response = self.chessboard_serial.readline().decode().strip()
                    if 'CHESSBOARD' in response or 'MODE' in response:
                        response_received = True
                        break
                time.sleep(0.1)
            
            self.test_results['chessboard_connection'] = response_received
            status = "✅ PASS" if response_received else "❌ FAIL"
            self.get_logger().info(f"ChessBoard connection: {status}")
            
        except Exception as e:
            self.get_logger().error(f"ChessBoard connection failed: {e}")
            self.test_results['chessboard_connection'] = False
        
        # Test Robot connection
        try:
            self.robot_serial = serial.Serial(
                self.robot_port, 9600, timeout=2.0
            )
            time.sleep(2)  # Allow connection to stabilize
            
            # Send test command
            self.robot_serial.write(b'mode:mock\n')
            self.robot_serial.flush()
            
            # Wait for response
            start_time = time.time()
            response_received = False
            
            while time.time() - start_time < 5.0:
                if self.robot_serial.in_waiting > 0:
                    response = self.robot_serial.readline().decode().strip()
                    if 'ROBOT' in response or 'MODE' in response:
                        response_received = True
                        break
                time.sleep(0.1)
            
            self.test_results['robot_connection'] = response_received
            status = "✅ PASS" if response_received else "❌ FAIL"
            self.get_logger().info(f"Robot connection: {status}")
            
        except Exception as e:
            self.get_logger().error(f"Robot connection failed: {e}")
            self.test_results['robot_connection'] = False
    
    def test_chessboard_commands(self):
        """Test ChessBoard command sequence"""
        self.get_logger().info("Testing ChessBoard commands...")
        
        commands = [
            ('init', 'INIT'),
            ('occupancy:0:1:2:3:4:5:6:7:48:49:50:51:52:53:54:55', 'OCCUPANCY'),
            ('legal:e2e4:d2d4:g1f3', 'LEGAL'),
            ('start', 'START')
        ]
        
        for command, expected_response in commands:
            success = self.send_and_verify_command(
                self.chessboard_serial, command, expected_response, 10.0
            )
            
            self.test_results['chessboard_commands'].append({
                'command': command,
                'expected': expected_response,
                'success': success
            })
            
            status = "✅ PASS" if success else "❌ FAIL"
            self.get_logger().info(f"ChessBoard '{command}': {status}")
            
            time.sleep(1)  # Brief pause between commands
    
    def test_robot_commands(self):
        """Test Robot command sequence"""
        self.get_logger().info("Testing Robot commands...")
        
        commands = [
            ('init', 'ROBOT'),
            ('home', 'ROBOT'),
            ('status', 'ROBOT'),
            ('e2pe4p', 'ROBOT')  # Test move command
        ]
        
        for command, expected_response in commands:
            timeout = 15.0 if command == 'e2pe4p' else 5.0  # Longer timeout for moves
            
            success = self.send_and_verify_command(
                self.robot_serial, command, expected_response, timeout
            )
            
            self.test_results['robot_commands'].append({
                'command': command,
                'expected': expected_response,
                'success': success
            })
            
            status = "✅ PASS" if success else "❌ FAIL"
            self.get_logger().info(f"Robot '{command}': {status}")
            
            time.sleep(1)  # Brief pause between commands
    
    def send_and_verify_command(self, serial_conn, command, expected_response, timeout):
        """Send command and verify response"""
        try:
            # Clear any pending data
            serial_conn.reset_input_buffer()
            
            # Send command
            command_bytes = (command + '\n').encode()
            serial_conn.write(command_bytes)
            serial_conn.flush()
            
            # Wait for response
            start_time = time.time()
            responses = []
            
            while time.time() - start_time < timeout:
                if serial_conn.in_waiting > 0:
                    response = serial_conn.readline().decode().strip()
                    if response:
                        responses.append(response)
                        
                        # Check if expected response received
                        if expected_response in response:
                            return True
                
                time.sleep(0.1)
            
            # Log all responses for debugging
            if responses:
                self.get_logger().debug(f"Responses for '{command}': {responses}")
            else:
                self.get_logger().warning(f"No response for '{command}'")
            
            return False
            
        except Exception as e:
            self.get_logger().error(f"Command '{command}' failed: {e}")
            return False
    
    def test_timing_requirements(self):
        """Test timing requirements for mock mode"""
        self.get_logger().info("Testing timing requirements...")
        
        if not self.test_results['robot_connection']:
            self.get_logger().warning("Skipping timing tests - Robot not connected")
            return
        
        # Test move execution timing (should be 3-8 seconds in mock mode)
        test_moves = ['d2pd4p', 'g1nf3n', 'b1nc3n']
        
        for move in test_moves:
            start_time = time.time()
            
            # Send move command
            self.robot_serial.reset_input_buffer()
            self.robot_serial.write((move + '\n').encode())
            self.robot_serial.flush()
            
            # Wait for completion
            move_completed = False
            while time.time() - start_time < 15.0:
                if self.robot_serial.in_waiting > 0:
                    response = self.robot_serial.readline().decode().strip()
                    if 'Move completed' in response:
                        move_completed = True
                        break
                time.sleep(0.1)
            
            execution_time = time.time() - start_time
            
            # Validate timing (3-15 seconds expected for mock mode)
            timing_valid = 3.0 <= execution_time <= 15.0 and move_completed
            
            self.test_results['timing_tests'].append({
                'move': move,
                'execution_time': execution_time,
                'completed': move_completed,
                'timing_valid': timing_valid
            })
            
            status = "✅ PASS" if timing_valid else "❌ FAIL"
            self.get_logger().info(
                f"Move '{move}' timing: {execution_time:.1f}s {status}"
            )
    
    def generate_test_report(self):
        """Generate comprehensive test report"""
        self.get_logger().info("Generating test report...")
        
        # Calculate overall success
        chessboard_success = (
            self.test_results['chessboard_connection'] and
            all(cmd['success'] for cmd in self.test_results['chessboard_commands'])
        )
        
        robot_success = (
            self.test_results['robot_connection'] and
            all(cmd['success'] for cmd in self.test_results['robot_commands'])
        )
        
        timing_success = all(
            test['timing_valid'] for test in self.test_results['timing_tests']
        )
        
        self.test_results['overall_success'] = (
            chessboard_success and robot_success and timing_success
        )
        
        # Print summary
        self.get_logger().info("=" * 50)
        self.get_logger().info("CONTROLLER COMMUNICATION TEST REPORT")
        self.get_logger().info("=" * 50)
        
        status = "✅ PASS" if chessboard_success else "❌ FAIL"
        self.get_logger().info(f"ChessBoard Tests: {status}")
        
        status = "✅ PASS" if robot_success else "❌ FAIL"
        self.get_logger().info(f"Robot Tests: {status}")
        
        status = "✅ PASS" if timing_success else "❌ FAIL"
        self.get_logger().info(f"Timing Tests: {status}")
        
        overall_status = "✅ PASS" if self.test_results['overall_success'] else "❌ FAIL"
        self.get_logger().info(f"Overall Result: {overall_status}")
        
        self.get_logger().info("=" * 50)
        
        # Exit with appropriate code
        if not self.test_results['overall_success']:
            sys.exit(1)
    
    def cleanup_connections(self):
        """Clean up serial connections"""
        if self.chessboard_serial and self.chessboard_serial.is_open:
            self.chessboard_serial.close()
        
        if self.robot_serial and self.robot_serial.is_open:
            self.robot_serial.close()


def main(args=None):
    rclpy.init(args=args)
    
    test_node = ControllerCommunicationTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
