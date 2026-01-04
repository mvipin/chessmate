#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
ChessBoard Controller USB Communication Test Script

This script tests the ChessBoard Pi Pico controller via USB Serial.
It verifies all command types and mock game flow.

Device naming:
  - Primary: /dev/chessboard (udev symlink for persistent naming)
  - Fallback: /dev/ttyACM0, /dev/ttyACM1 (if udev rules not installed)
"""

import serial
import threading
import time
import sys
import os
from datetime import datetime

# Configuration
BAUD_RATE = 9600
TIMEOUT = 1.0


def detect_chessboard_port():
    """Detect ChessBoard controller port with fallback logic."""
    ports_to_try = [
        '/dev/chessboard',  # Primary: udev symlink
        '/dev/ttyACM0',     # Fallback: typical first USB device
        '/dev/ttyACM1',     # Fallback: second USB device
    ]

    for port in ports_to_try:
        if os.path.exists(port):
            return port

    return None


class ChessBoardTester:
    def __init__(self, port=None, baud=BAUD_RATE):
        if port is None:
            port = detect_chessboard_port()
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        self.responses = []
        self.response_lock = threading.Lock()
        
    def connect(self):
        """Connect to the ChessBoard controller"""
        try:
            print(f"Connecting to ChessBoard controller at {self.port}...")
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=TIMEOUT)
            time.sleep(2)  # Allow time for connection to stabilize
            print(f"✅ Connected to {self.port}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the controller"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("🔌 Disconnected from ChessBoard controller")
    
    def start_reading(self):
        """Start the response reading thread"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_responses, daemon=True)
        self.read_thread.start()
    
    def _read_responses(self):
        """Read responses from the controller"""
        while self.running and self.serial_conn and self.serial_conn.is_open:
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                        response = f"[{timestamp}] {line}"
                        print(f"📥 {response}")
                        
                        with self.response_lock:
                            self.responses.append(line)
                            
                time.sleep(0.01)
            except Exception as e:
                if self.running:
                    print(f"❌ Read error: {e}")
                break
    
    def send_command(self, command, wait_time=2.0):
        """Send a command and wait for responses"""
        if not self.serial_conn or not self.serial_conn.is_open:
            print("❌ Not connected to controller")
            return False
            
        try:
            print(f"📤 Sending: {command}")
            self.serial_conn.write((command + '\n').encode())
            self.serial_conn.flush()
            
            if wait_time > 0:
                time.sleep(wait_time)
            
            return True
        except Exception as e:
            print(f"❌ Send error: {e}")
            return False
    
    def wait_for_response(self, expected_text, timeout=10.0):
        """Wait for a specific response"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            with self.response_lock:
                for response in self.responses:
                    if expected_text.lower() in response.lower():
                        return True
            time.sleep(0.1)
        return False
    
    def clear_responses(self):
        """Clear the response buffer"""
        with self.response_lock:
            self.responses.clear()
    
    def test_basic_commands(self):
        """Test basic command sequence"""
        print("\n🧪 Testing Basic Commands")
        print("=" * 50)
        
        commands = [
            ("mode:mock", "MODE: Mock simulation mode enabled"),
            ("init", "INIT: Board reset complete"),
            ("occupancy:0:1:2:3", "OCCUPANCY: Set 4 pieces"),
            ("legal:e2e4:d2d4", "LEGAL: Set 2 legal moves"),
        ]
        
        for command, expected in commands:
            self.clear_responses()
            if not self.send_command(command, 1.0):
                return False
                
            if not self.wait_for_response(expected, 3.0):
                print(f"⚠️  Expected response '{expected}' not received for command '{command}'")
            else:
                print(f"✅ Command '{command}' successful")
        
        return True
    
    def test_mock_game_flow(self):
        """Test complete mock game flow"""
        print("\n🎮 Testing Mock Game Flow")
        print("=" * 50)
        
        # Start the game
        self.clear_responses()
        if not self.send_command("start", 0.5):
            return False
        
        # Wait for game flow messages
        expected_messages = [
            "START: Human turn beginning",
            "LEGAL MOVES:",
            "MOCK: Ready for human turn",
            "MOCK: Transitioning to MOVE_START",
            "MOCK: Starting move",
            "MOCK: Move completed",
            "MOCK: Confirming move"
        ]
        
        print("⏳ Waiting for mock game simulation...")
        time.sleep(8)  # Wait for mock move to complete
        
        # Check if we got a move response
        move_received = False
        with self.response_lock:
            for response in self.responses:
                if len(response) == 4 and response.isalnum():  # Move format like "e2e4"
                    print(f"🎯 Move received: {response}")
                    move_received = True
                    break
        
        if move_received:
            print("✅ Mock game flow completed successfully")
            return True
        else:
            print("⚠️  No move response received")
            return False
    
    def test_interactive_mode(self):
        """Interactive testing mode"""
        print("\n🎮 Interactive Mode")
        print("=" * 50)
        print("Enter commands to send to ChessBoard controller.")
        print("Type 'quit' to exit, 'help' for command list.")
        
        while True:
            try:
                command = input("\nChessBoard> ").strip()
                
                if command.lower() == 'quit':
                    break
                elif command.lower() == 'help':
                    self._print_help()
                elif command:
                    self.send_command(command, 0.5)
                    
            except KeyboardInterrupt:
                break
    
    def _print_help(self):
        """Print available commands"""
        print("\n📋 Available Commands:")
        print("  mode:mock          - Enable mock simulation mode")
        print("  mode:real          - Enable real hardware mode")
        print("  init               - Initialize board")
        print("  occupancy:0:1:2:3  - Set piece occupancy")
        print("  legal:e2e4:d2d4    - Set legal moves")
        print("  start              - Start human turn")
        print("  reset              - Reset board")
        print("  quit               - Exit interactive mode")

def main():
    """Main test function"""
    print("🏁 ChessBoard Controller USB Test")
    print("=" * 50)

    # Detect ChessBoard controller port
    port = detect_chessboard_port()
    if port is None:
        print("❌ ChessBoard controller not found")
        print("   Checked: /dev/chessboard, /dev/ttyACM0, /dev/ttyACM1")
        print("   Make sure the ChessBoard Pi Pico is connected via USB")
        return 1

    if port == '/dev/chessboard':
        print(f"✅ Using udev symlink: {port}")
    else:
        print(f"⚠️  Using fallback port: {port}")
        print("   To enable persistent naming, install udev rules")

    tester = ChessBoardTester(port=port)
    
    try:
        # Connect to controller
        if not tester.connect():
            return 1
        
        # Start reading responses
        tester.start_reading()
        time.sleep(1)  # Allow reader to start
        
        # Run tests
        if len(sys.argv) > 1 and sys.argv[1] == 'interactive':
            tester.test_interactive_mode()
        else:
            # Automated tests
            print("🔄 Running automated tests...")

            if not tester.test_basic_commands():
                print("❌ Basic command tests failed")
                return 1

            if not tester.test_mock_game_flow():
                print("❌ Mock game flow test failed")
                return 1

            print("\n🎉 All tests completed!")

            # Only offer interactive mode if not in fully automated mode
            if len(sys.argv) > 1 and sys.argv[1] == 'automated':
                print("✅ Automated test completed successfully")
            else:
                # Offer interactive mode
                response = input("\nEnter interactive mode? (y/n): ")
                if response.lower().startswith('y'):
                    tester.test_interactive_mode()
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    finally:
        tester.disconnect()
    
    return 0

if __name__ == "__main__":
    exit(main())
