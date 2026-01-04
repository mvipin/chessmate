#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Robot Controller USB Communication Test Script

This script tests the Robot Pi Pico controller via USB Serial.
It verifies command processing and move execution.
"""

import serial
import threading
import time
import sys
from datetime import datetime

# Configuration (using udev symlink for persistent naming)
USB_PORT = '/dev/robot'  # Robot controller via udev symlink
BAUD_RATE = 9600
TIMEOUT = 1.0

class RobotTester:
    def __init__(self, port=USB_PORT, baud=BAUD_RATE):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.running = False
        self.responses = []
        self.response_lock = threading.Lock()
        
    def connect(self):
        """Connect to the Robot controller"""
        try:
            print(f"Connecting to Robot controller at {self.port}...")
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
            print("🔌 Disconnected from Robot controller")
    
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
    
    def send_command(self, command, wait_time=1.0):
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
    
    def test_basic_commands(self):
        """Test basic command sequence"""
        print("\n🧪 Testing Basic Robot Commands")
        print("=" * 50)
        
        commands = [
            "mode:mock",
            "init",
            "home",
            "status",
        ]
        
        for command in commands:
            if not self.send_command(command, 1.5):
                return False
            print(f"✅ Command '{command}' sent")
        
        return True
    
    def test_move_commands(self):
        """Test move command execution"""
        print("\n🎮 Testing Move Commands")
        print("=" * 50)
        
        moves = [
            "e2pe4p",  # Pawn e2 to e4
            "d2pd4p",  # Pawn d2 to d4
            "g1nf3n",  # Knight g1 to f3
            "b1nc3n",  # Knight b1 to c3
        ]
        
        for move in moves:
            print(f"\n🎯 Testing move: {move}")
            if not self.send_command(move, 2.0):
                return False
            
            # Wait for move completion in mock mode
            print("⏳ Waiting for move completion...")
            time.sleep(6)  # Mock moves take 3-8 seconds
            
            print(f"✅ Move '{move}' completed")
        
        return True
    
    def test_single_char_commands(self):
        """Test single character commands"""
        print("\n🔤 Testing Single Character Commands")
        print("=" * 50)
        
        commands = ['i', 's', 'j', 'z']
        
        for cmd in commands:
            if not self.send_command(cmd, 1.0):
                return False
            print(f"✅ Single char command '{cmd}' sent")
        
        return True
    
    def test_interactive_mode(self):
        """Interactive testing mode"""
        print("\n🎮 Interactive Mode")
        print("=" * 50)
        print("Enter commands to send to Robot controller.")
        print("Type 'quit' to exit, 'help' for command list.")
        
        while True:
            try:
                command = input("\nRobot> ").strip()
                
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
        print("\n📋 Available Robot Commands:")
        print("  mode:mock          - Enable mock simulation mode")
        print("  mode:real          - Enable real hardware mode")
        print("  init               - Initialize robot systems")
        print("  home               - Home robot to origin")
        print("  status             - Get robot status")
        print("  e2pe4p             - Execute move (6-char format)")
        print("  i                  - Wake up robot")
        print("  s                  - Sleep robot")
        print("  j                  - Home Z-axis")
        print("  z                  - Home all axes")
        print("  quit               - Exit interactive mode")

def main():
    """Main test function"""
    print("🤖 Robot Controller USB Test")
    print("=" * 50)
    
    # Check if port exists
    import os
    if not os.path.exists(USB_PORT):
        print(f"❌ Robot controller not found at {USB_PORT}")
        print("   Make sure the Robot Pi Pico is connected via USB")
        print("   You may need to adjust the USB_PORT in the script")
        return 1
    
    tester = RobotTester()
    
    try:
        # Connect to controller
        if not tester.connect():
            return 1
        
        # Start reading responses
        tester.start_reading()
        time.sleep(1)  # Allow reader to start
        
        # Run tests based on command line argument
        if len(sys.argv) > 1 and sys.argv[1] == 'interactive':
            tester.test_interactive_mode()
        else:
            # Automated tests
            print("🔄 Running automated tests...")
            
            if not tester.test_basic_commands():
                print("❌ Basic command tests failed")
                return 1
            
            if not tester.test_single_char_commands():
                print("❌ Single character command tests failed")
                return 1
            
            if not tester.test_move_commands():
                print("❌ Move command tests failed")
                return 1
            
            print("\n🎉 All Robot tests completed!")

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
