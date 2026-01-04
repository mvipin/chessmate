#!/usr/bin/env python3
"""
Bare-bones Rotary Encoder GPIO Test

This script tests the rotary encoder using direct GPIO access without ROS2.
It's useful for hardware validation and debugging GPIO connections.
"""

import time
import sys
import signal
import argparse

# GPIO pin definitions (matching ChessMate hardware)
CLK_PIN = 11    # Rotary encoder CLK pin
DT_PIN = 12     # Rotary encoder DT pin  
BTN_PIN = 13    # Rotary encoder button pin

class RotaryEncoderGPIOTest:
    def __init__(self, mock_mode=False):
        self.mock_mode = mock_mode
        self.running = True
        self.position = 0
        self.last_clk = 1
        self.button_pressed = False
        
        # Set up signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)
        
        if not mock_mode:
            try:
                import RPi.GPIO as GPIO
                self.GPIO = GPIO
                self.setup_real_gpio()
            except ImportError:
                print("❌ RPi.GPIO not available. Use --mock for simulation.")
                sys.exit(1)
        else:
            print("🔧 Mock mode - simulating GPIO operations")
    
    def setup_real_gpio(self):
        """Setup real GPIO pins for rotary encoder"""
        print(f"🔧 Setting up GPIO pins: CLK={CLK_PIN}, DT={DT_PIN}, BTN={BTN_PIN}")
        
        # Use BCM pin numbering
        self.GPIO.setmode(self.GPIO.BCM)
        self.GPIO.setwarnings(False)
        
        # Setup pins as inputs with pull-up resistors
        self.GPIO.setup(CLK_PIN, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
        self.GPIO.setup(DT_PIN, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
        self.GPIO.setup(BTN_PIN, self.GPIO.IN, pull_up_down=self.GPIO.PUD_UP)
        
        # Add event detection for rotation
        self.GPIO.add_event_detect(CLK_PIN, self.GPIO.BOTH, callback=self.rotation_callback, bouncetime=50)
        
        # Add event detection for button press
        self.GPIO.add_event_detect(BTN_PIN, self.GPIO.BOTH, callback=self.button_callback, bouncetime=300)
        
        # Read initial state
        self.last_clk = self.GPIO.input(CLK_PIN)
        
        print("✅ GPIO setup completed")
    
    def rotation_callback(self, channel):
        """Handle rotary encoder rotation"""
        if not self.running:
            return
            
        try:
            clk_state = self.GPIO.input(CLK_PIN)
            dt_state = self.GPIO.input(DT_PIN)
            
            # Detect rotation direction
            if clk_state != self.last_clk:
                if dt_state != clk_state:
                    self.position += 1
                    direction = "CLOCKWISE"
                else:
                    self.position -= 1
                    direction = "COUNTER-CLOCKWISE"
                
                print(f"🔄 Rotation detected: {direction} (Position: {self.position})")
                
            self.last_clk = clk_state
            
        except Exception as e:
            print(f"❌ Rotation callback error: {e}")
    
    def button_callback(self, channel):
        """Handle button press/release"""
        if not self.running:
            return
            
        try:
            button_state = self.GPIO.input(BTN_PIN)
            
            if button_state == 0:  # Button pressed (active low)
                if not self.button_pressed:
                    self.button_pressed = True
                    print("🔘 Button PRESSED")
            else:  # Button released
                if self.button_pressed:
                    self.button_pressed = False
                    print("🔘 Button RELEASED")
                    
        except Exception as e:
            print(f"❌ Button callback error: {e}")
    
    def simulate_events(self):
        """Simulate encoder events in mock mode"""
        events = [
            ("rotation", "CLOCKWISE", 1),
            ("rotation", "COUNTER-CLOCKWISE", -1),
            ("button", "PRESSED", 0),
            ("button", "RELEASED", 0),
            ("rotation", "CLOCKWISE", 2),
            ("rotation", "CLOCKWISE", 3),
        ]
        
        for event_type, action, position in events:
            if not self.running:
                break
                
            time.sleep(1)
            
            if event_type == "rotation":
                self.position = position
                print(f"🔄 Mock rotation: {action} (Position: {self.position})")
            elif event_type == "button":
                print(f"🔘 Mock button: {action}")
    
    def run_test(self, duration=30):
        """Run the encoder test for specified duration"""
        print(f"\n🎯 Rotary Encoder GPIO Test Started")
        print(f"Mode: {'Mock' if self.mock_mode else 'Real Hardware'}")
        print(f"Duration: {duration} seconds")
        
        if not self.mock_mode:
            print("\n📋 Instructions:")
            print("  • Turn the rotary encoder clockwise/counter-clockwise")
            print("  • Press and release the encoder button")
            print("  • Press Ctrl+C to stop the test")
            print("\n🔍 Monitoring GPIO events...")
        else:
            print("\n🔧 Simulating encoder events...")
        
        start_time = time.time()
        
        try:
            if self.mock_mode:
                # Run simulation
                self.simulate_events()
            else:
                # Wait for real events
                while self.running and (time.time() - start_time) < duration:
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\n⏹️  Test stopped by user")
        
        self.cleanup()
        
        print(f"\n📊 Test Results:")
        print(f"  Final position: {self.position}")
        print(f"  Duration: {time.time() - start_time:.1f} seconds")
        print("✅ Rotary encoder GPIO test completed")
    
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print("\n⏹️  Stopping test...")
        self.running = False
    
    def cleanup(self):
        """Clean up GPIO resources"""
        self.running = False
        
        if not self.mock_mode and hasattr(self, 'GPIO'):
            try:
                self.GPIO.cleanup()
                print("🧹 GPIO cleanup completed")
            except Exception as e:
                print(f"⚠️  GPIO cleanup warning: {e}")

def main():
    parser = argparse.ArgumentParser(description='Rotary Encoder GPIO Test')
    parser.add_argument('--mock', action='store_true', help='Use mock mode (no real GPIO)')
    parser.add_argument('--duration', type=int, default=30, help='Test duration in seconds (default: 30)')
    
    args = parser.parse_args()
    
    # Create and run test
    test = RotaryEncoderGPIOTest(mock_mode=args.mock)
    test.run_test(duration=args.duration)

if __name__ == '__main__':
    main()
