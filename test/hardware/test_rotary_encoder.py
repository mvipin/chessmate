#!/usr/bin/env python3
"""
Simple test script to monitor rotary encoder events
"""

import rclpy
from rclpy.node import Node
from chessmate.msg import RotaryEncoderEvent

class RotaryEncoderTest(Node):
    def __init__(self):
        super().__init__('rotary_encoder_test')
        
        # Subscribe to rotary encoder events
        self.subscription = self.create_subscription(
            RotaryEncoderEvent,
            'rotary_encoder_events',
            self.encoder_callback,
            10
        )
        
        self.get_logger().info('🎯 Rotary Encoder Test Node Started')
        self.get_logger().info('Turn the encoder or press the button to see events...')
        
    def encoder_callback(self, msg):
        """Handle rotary encoder events"""
        # Use the constants from the message definition
        if msg.event_type == RotaryEncoderEvent.EVENT_TYPE_ROTATION:
            direction_str = "clockwise" if msg.direction > 0 else "counterclockwise"
            self.get_logger().info(f'🔄 ROTATION: {direction_str} (direction: {msg.direction}, position: {msg.encoder_position})')
        elif msg.event_type == RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS:
            self.get_logger().info(f'🔘 BUTTON PRESS (position: {msg.encoder_position})')
        elif msg.event_type == RotaryEncoderEvent.EVENT_TYPE_BUTTON_RELEASE:
            self.get_logger().info(f'🔘 BUTTON RELEASE (position: {msg.encoder_position})')
        else:
            self.get_logger().info(f'❓ UNKNOWN EVENT: type={msg.event_type}, direction={msg.direction}, button={msg.button_pressed}, pos={msg.encoder_position}')

def main(args=None):
    rclpy.init(args=args)
    
    test_node = RotaryEncoderTest()
    
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info('Test stopped by user')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
