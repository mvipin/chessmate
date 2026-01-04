#!/usr/bin/env python3
"""
Rotary Encoder ROS 2 Node for ChessMate

This node interfaces with the rotary encoder hardware and publishes
user input events to the ROS 2 system. It's based on the original
Rotary.py but adapted for ROS 2 architecture.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time
import threading
from typing import Optional

from chessmate.msg import RotaryEncoderEvent
from chessmate.hardware.gpio_abstraction import GPIOAbstraction, GPIOMode, GPIOPull, GPIOEdge


class RotaryEncoderNode(Node):
    """
    ROS 2 Node for handling rotary encoder input
    
    This node:
    - Interfaces with rotary encoder hardware (CLK, DT, BTN pins)
    - Publishes RotaryEncoderEvent messages for rotation and button events
    - Maintains encoder position state
    - Handles debouncing and calibration
    """
    
    def __init__(self):
        super().__init__('rotary_encoder_node')
        
        # Declare parameters
        self.declare_parameter('clk_pin', 11)
        self.declare_parameter('dt_pin', 12)
        self.declare_parameter('btn_pin', 13)
        self.declare_parameter('use_real_gpio', GPIOAbstraction.is_raspberry_pi())
        self.declare_parameter('publish_rate', 10.0)  # Hz
        
        # Get parameters
        self.clk_pin = self.get_parameter('clk_pin').value
        self.dt_pin = self.get_parameter('dt_pin').value
        self.btn_pin = self.get_parameter('btn_pin').value
        use_real_gpio = self.get_parameter('use_real_gpio').value
        
        self.get_logger().info(f"Rotary Encoder Node starting with pins: CLK={self.clk_pin}, DT={self.dt_pin}, BTN={self.btn_pin}")
        
        # Initialize GPIO abstraction
        try:
            self.gpio = GPIOAbstraction(use_real_gpio=use_real_gpio)
            self.get_logger().info(f'GPIO abstraction initialized - using real GPIO: {self.gpio.use_real_gpio}')
        except Exception as e:
            self.get_logger().error(f"Failed to initialize GPIO: {e}")
            raise
        
        # Encoder state variables (matching original Rotary.py logic)
        self.rotary_last_state = None
        self.btn_last_state = None
        self.clk_level = 0
        self.dt_level = 0
        self.calibrate = False
        self.encoder_position = 0
        self.button_pressed = False
        
        # Thread safety
        self.state_lock = threading.Lock()

        # Create publisher with appropriate QoS BEFORE setting up GPIO
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.encoder_publisher = self.create_publisher(
            RotaryEncoderEvent,
            'rotary_encoder_events',
            qos_profile
        )

        # Setup GPIO pins AFTER publisher is created
        self._setup_gpio()
        
        # Create timer for periodic status updates
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.status_timer = self.create_timer(1.0 / publish_rate, self._publish_status)
        
        self.get_logger().info("Rotary Encoder Node initialized successfully")
    
    def _setup_gpio(self):
        """Setup GPIO pins and event detection"""
        try:
            # Setup pins
            self.gpio.setup_pin(self.clk_pin, GPIOMode.INPUT, GPIOPull.UP)
            self.gpio.setup_pin(self.dt_pin, GPIOMode.INPUT, GPIOPull.UP)
            self.gpio.setup_pin(self.btn_pin, GPIOMode.INPUT, GPIOPull.UP)
            
            # Add event detection with proper bouncetime
            self.get_logger().debug(f'Setting up GPIO event detection on pins CLK={self.clk_pin}, DT={self.dt_pin}, SW={self.btn_pin}')
            self.gpio.add_event_detect(self.clk_pin, GPIOEdge.BOTH, self._pulse_callback, bouncetime=50)
            self.gpio.add_event_detect(self.dt_pin, GPIOEdge.BOTH, self._pulse_callback, bouncetime=50)
            self.gpio.add_event_detect(self.btn_pin, GPIOEdge.FALLING, self._button_callback, bouncetime=300)
            self.get_logger().debug('GPIO event detection setup complete')
            
            # Initialize states
            with self.state_lock:
                self.rotary_last_state = None
                self.btn_last_state = self.gpio.read_pin(self.btn_pin)
                self.button_pressed = not self.btn_last_state  # Inverted logic for pull-up
            
            self.get_logger().info("GPIO setup completed")
            
        except Exception as e:
            self.get_logger().error(f"GPIO setup failed: {e}")
            raise
    
    def _pulse_callback(self, channel):
        """
        Handle rotary encoder pulse events (CLK/DT pin changes)

        This implements the same logic as the original Rotary.py __pulse method
        """

        try:
            clk_state = self.gpio.read_pin(self.clk_pin)
            dt_state = self.gpio.read_pin(self.dt_pin)

            self.get_logger().debug(f"Pulse callback: channel={channel}, CLK={clk_state}, DT={dt_state}, last_state={self.rotary_last_state}")

            with self.state_lock:
                if clk_state != self.rotary_last_state:
                    self.rotary_last_state = clk_state

                    # Simplified rotation detection - detect on every CLK falling edge
                    if clk_state == 0:  # CLK falling edge
                        # Determine rotation direction
                        if dt_state != clk_state:
                            direction = -1  # Counter-clockwise
                            self.encoder_position -= 1
                        else:
                            direction = 1   # Clockwise
                            self.encoder_position += 1

                        self.get_logger().info(f"Rotation detected: direction={direction}, position={self.encoder_position}")

                        # Publish rotation event
                        self._publish_rotation_event(direction)
                        
        except Exception as e:
            self.get_logger().error(f"Pulse callback error: {e}")
    
    def _button_callback(self, channel):
        """
        Handle button press events

        This implements the same logic as the original Rotary.py __button method
        """

        try:
            with self.state_lock:
                current_state = self.gpio.read_pin(self.btn_pin)
                self.button_pressed = not current_state  # Inverted logic for pull-up
                
                # Publish button press event
                self._publish_button_event()
                
        except Exception as e:
            self.get_logger().error(f"Button callback error: {e}")
    
    def _publish_rotation_event(self, direction: int):
        """
        Publish a rotation event
        
        Args:
            direction: Rotation direction (-1 for CCW, +1 for CW)
        """
        msg = RotaryEncoderEvent()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.event_type = RotaryEncoderEvent.EVENT_TYPE_ROTATION
        msg.direction = direction
        msg.button_pressed = self.button_pressed
        msg.encoder_position = self.encoder_position
        
        self.encoder_publisher.publish(msg)
        
        self.get_logger().debug(f"Published rotation event: direction={direction}, position={self.encoder_position}")
    
    def _publish_button_event(self):
        """Publish a button press event"""
        msg = RotaryEncoderEvent()
        msg.timestamp = self.get_clock().now().to_msg()
        msg.event_type = RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS
        msg.direction = 0  # Not applicable for button events
        msg.button_pressed = self.button_pressed
        msg.encoder_position = self.encoder_position
        
        self.encoder_publisher.publish(msg)
        
        self.get_logger().info(f"Published button event: pressed={self.button_pressed}")
    
    def _publish_status(self):
        """Publish periodic status updates (for debugging/monitoring)"""
        # This is optional - only publish if there are subscribers
        if self.encoder_publisher.get_subscription_count() > 0:
            with self.state_lock:
                current_button_state = self.gpio.read_pin(self.btn_pin)
                current_button_pressed = not current_button_state
                
                # Only publish if button state changed
                if current_button_pressed != self.button_pressed:
                    self.button_pressed = current_button_pressed
                    event_type = (RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS if current_button_pressed 
                                else RotaryEncoderEvent.EVENT_TYPE_BUTTON_RELEASE)
                    
                    msg = RotaryEncoderEvent()
                    msg.timestamp = self.get_clock().now().to_msg()
                    msg.event_type = event_type
                    msg.direction = 0
                    msg.button_pressed = self.button_pressed
                    msg.encoder_position = self.encoder_position
                    
                    self.encoder_publisher.publish(msg)
                    
                    self.get_logger().debug(f"Status update: button_pressed={self.button_pressed}")
    
    def get_encoder_position(self) -> int:
        """
        Get current encoder position
        
        Returns:
            Current encoder position
        """
        with self.state_lock:
            return self.encoder_position
    
    def reset_encoder_position(self):
        """Reset encoder position to zero"""
        with self.state_lock:
            self.encoder_position = 0
            self.get_logger().info("Encoder position reset to zero")
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        try:
            self.get_logger().info("Shutting down Rotary Encoder Node")
            
            # Remove event detection
            self.gpio.remove_event_detect(self.clk_pin)
            self.gpio.remove_event_detect(self.dt_pin)
            self.gpio.remove_event_detect(self.btn_pin)
            
            # Cleanup GPIO
            self.gpio.cleanup()
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        
        super().destroy_node()


def main(args=None):
    """Main entry point for the rotary encoder node"""
    rclpy.init(args=args)
    
    try:
        node = RotaryEncoderNode()
        
        # Use MultiThreadedExecutor for better callback handling
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        
        try:
            rclpy.spin(node, executor=executor)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start rotary encoder node: {e}")
        return 1
    
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
    
    return 0


if __name__ == '__main__':
    exit(main())
