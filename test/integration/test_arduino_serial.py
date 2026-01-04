#!/usr/bin/env python3
"""
Arduino Serial Communication Test - Character Protocol Only

Tests direct serial communication with Arduino using character-based protocol.
This is the primary test for validating Arduino connectivity before ROS 2 integration.

Usage:
    ros2 run chessmate_hardware test_arduino_serial --port /dev/robot
    ros2 run chessmate_hardware test_arduino_serial --port /dev/chessboard
    python3 test_arduino_serial.py --port /dev/robot
"""

import serial
import time
import sys
import argparse


def test_arduino_character_protocol(port='/dev/robot', baud=9600):
    """Test character-based protocol with Arduino"""
    print(f"🔤 Testing Arduino Character Protocol on {port}...")
    
    try:
        arduino = serial.Serial(port, baud, timeout=2)
        time.sleep(3)  # Wait for Arduino reset
        arduino.reset_input_buffer()
        
        print("✅ Connected successfully!")
        print("=" * 50)
        
        # Test character commands
        char_tests = [
            ('i', 'Wake up'),
            ('j', 'Home Z'),
            ('z', 'Home all'),
            ('s', 'Sleep'),
            ('status', 'Status check'),
        ]
        
        success_count = 0
        for cmd, description in char_tests:
            print(f"🔤 Testing: {description} ('{cmd}')")
            arduino.write((cmd + '\n').encode())
            time.sleep(0.5)
            
            response = ""
            start_time = time.time()
            while time.time() - start_time < 1.0:
                if arduino.in_waiting > 0:
                    char = arduino.read(1).decode('utf-8', errors='ignore')
                    if char == '\n':
                        break
                    response += char
            
            if response:
                print(f"   📥 Response: '{response.strip()}'")
                success_count += 1
            else:
                print("   ⚠️  No response")
        
        # Test move command
        print("🤖 Testing move command...")
        arduino.write(b'e2e4p\n')
        time.sleep(0.5)
        
        response = ""
        start_time = time.time()
        while time.time() - start_time < 1.0:
            if arduino.in_waiting > 0:
                char = arduino.read(1).decode('utf-8', errors='ignore')
                if char == '\n':
                    break
                response += char
        
        if response:
            print(f"   📥 Move response: '{response.strip()}'")
            success_count += 1
        
        arduino.close()
        
        # Results
        total_tests = len(char_tests) + 1
        success_rate = (success_count / total_tests) * 100
        
        print("=" * 50)
        print(f"📊 Test Results: {success_count}/{total_tests} ({success_rate:.1f}%)")
        
        if success_rate >= 80:
            print("✅ Arduino character protocol working!")
            return True
        else:
            print("⚠️  Some tests failed")
            return False
        
    except Exception as e:
        print(f"❌ Arduino test failed: {e}")
        return False


def main():
    """Main test function"""
    parser = argparse.ArgumentParser(description='Arduino Serial Communication Test')
    parser.add_argument('--port', '-p', default='/dev/robot',
                       help='Serial port (default: /dev/robot, also: /dev/chessboard)')
    parser.add_argument('--baud', '-b', type=int, default=9600,
                       help='Baud rate (default: 9600)')
    
    args = parser.parse_args()
    
    print("🤖 ChessMate Arduino Serial Test")
    print("=" * 40)
    print(f"Port: {args.port}")
    print(f"Baud Rate: {args.baud}")
    print("Protocol: Character (simple and reliable)")
    print()
    
    try:
        success = test_arduino_character_protocol(args.port, args.baud)
        
        if success:
            print("\n🎯 Next steps:")
            print("1. Test ROS 2 integration:")
            print("   ros2 launch chessmate_hardware unified_hardware.launch.py hardware_mode:=real")
            print("2. Run comprehensive tests:")
            print("   ros2 run chessmate_hardware unified_hardware_test comprehensive")
        else:
            print("\n❌ Arduino test failed. Check:")
            print("   - Arduino is connected to the correct port")
            print("   - Arduino has the character stub uploaded")
            print("   - Serial port permissions (dialout group)")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
        sys.exit(0)


if __name__ == '__main__':
    main()
