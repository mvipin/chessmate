#!/usr/bin/env python3
"""
LCD Display ROS 2 Node for ChessMate

This node interfaces with the SSD1306 OLED display and handles
menu rendering and game status display. It's based on the original
Menu.py and LCD module but adapted for ROS 2 architecture.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os
import threading
import time
from typing import List, Optional, Dict, Any

from chessmate.msg import LCDCommand, RotaryEncoderEvent
from chessmate.hardware.gpio_abstraction import GPIOAbstraction

# PIL imports for image processing
from PIL import Image, ImageDraw, ImageFont

# Real hardware imports (only imported when needed)
try:
    import board
    import busio
    import adafruit_ssd1306
    REAL_HARDWARE_AVAILABLE = True
except ImportError:
    REAL_HARDWARE_AVAILABLE = False


class LCDDisplayNode(Node):
    """
    ROS 2 Node for handling LCD display operations
    
    This node:
    - Subscribes to LCDCommand messages for display updates
    - Subscribes to RotaryEncoderEvent messages for menu navigation
    - Manages SSD1306 OLED display (real or mock)
    - Handles menu rendering and game status display
    """
    
    def __init__(self):
        super().__init__('lcd_display_node')
        
        # Declare parameters
        self.declare_parameter('use_real_display', GPIOAbstraction.is_raspberry_pi())
        self.declare_parameter('i2c_bus', 11)
        self.declare_parameter('display_width', 128)
        self.declare_parameter('display_height', 32)
        self.declare_parameter('font_path', '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf')
        self.declare_parameter('small_font_size', 8)
        self.declare_parameter('large_font_size', 15)
        
        # Get parameters
        use_real_display = self.get_parameter('use_real_display').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.display_width = self.get_parameter('display_width').value
        self.display_height = self.get_parameter('display_height').value
        self.font_path = self.get_parameter('font_path').value
        self.small_font_size = self.get_parameter('small_font_size').value
        self.large_font_size = self.get_parameter('large_font_size').value
        
        self.get_logger().info(f"LCD Display Node starting with {self.display_width}x{self.display_height} display")
        
        # Initialize display abstraction
        self.use_real_display = use_real_display
        self._init_display()
        
        # Menu state variables (based on original Menu.py)
        self.menu_options = []
        self.menu_option = None  # Currently selected menu item
        self.submenu_options = [0] * 10  # Submenu selections
        self.menu_level = 0  # 0 = main menu, 1 = submenu
        self.row_count = 3  # Number of visible menu rows
        self.current_text = "ChessMate Ready"
        
        # Thread safety
        self.display_lock = threading.Lock()
        self.render_thread = None
        
        # Create QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create subscribers
        self.lcd_command_subscriber = self.create_subscription(
            LCDCommand,
            'lcd_commands',
            self._lcd_command_callback,
            qos_profile
        )
        
        self.encoder_subscriber = self.create_subscription(
            RotaryEncoderEvent,
            'rotary_encoder_events',
            self._encoder_event_callback,
            qos_profile
        )
        
        # Initialize display
        self._clear_display()
        self._display_startup_message()
        
        self.get_logger().info("LCD Display Node initialized successfully")
    
    def _detect_display_hardware(self) -> bool:
        """
        Detect if real display hardware is available
        
        Returns:
            True if real display hardware detected, False otherwise
        """
        try:
            # Try to detect I2C bus
            i2c_path = f"/dev/i2c-{self.i2c_bus}"
            return os.path.exists(i2c_path)
        except:
            return False
    
    def _init_display(self):
        """Initialize display hardware (real or mock)"""
        if self.use_real_display:
            self._init_real_display()
        else:
            self._init_mock_display()
    
    def _init_real_display(self):
        """Initialize real SSD1306 display using Adafruit library"""
        try:
            if not REAL_HARDWARE_AVAILABLE:
                raise ImportError("Real hardware libraries not available")

            # Initialize I2C bus
            i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize SSD1306 display (128x32 pixels)
            self.oled = adafruit_ssd1306.SSD1306_I2C(
                self.display_width,
                self.display_height,
                i2c,
                addr=0x3C  # Default I2C address for SSD1306
            )

            # Clear display
            self.oled.fill(0)
            self.oled.show()

            # Initialize PIL objects for drawing
            self.image = Image.new('1', (self.display_width, self.display_height))
            self.draw = ImageDraw.Draw(self.image)
            
            # Load fonts
            try:
                self.font = ImageFont.truetype(self.font_path, self.small_font_size)
                self.bigfont = ImageFont.truetype(self.font_path, self.large_font_size)
            except:
                # Fallback to default font
                self.font = ImageFont.load_default()
                self.bigfont = ImageFont.load_default()
            
            self.get_logger().info("Real SSD1306 display initialized")
            
        except ImportError as e:
            self.get_logger().warning(f"Real display libraries not available: {e}, using mock display")
            self.use_real_display = False
            self._init_mock_display()
        except Exception as e:
            self.get_logger().error(f"Failed to initialize real display: {e}, using mock display")
            self.use_real_display = False
            self._init_mock_display()
    
    def _init_mock_display(self):
        """Initialize mock display for development"""
        self.oled = MockDisplay(self.display_width, self.display_height)
        self.image = MockImage(self.display_width, self.display_height)
        self.draw = MockDraw()
        self.font = MockFont()
        self.bigfont = MockFont()
        
        self.get_logger().info("Mock display initialized for development")
    
    def _lcd_command_callback(self, msg: LCDCommand):
        """
        Handle LCD command messages
        
        Args:
            msg: LCDCommand message
        """
        try:
            with self.display_lock:
                if msg.command_type == LCDCommand.CMD_CLEAR:
                    self._clear_display()
                    
                elif msg.command_type == LCDCommand.CMD_DISPLAY_TEXT:
                    self._display_text(msg.text, msg.x_position, msg.y_position, 
                                     msg.font_size, msg.invert_display)
                    
                elif msg.command_type == LCDCommand.CMD_DISPLAY_MENU:
                    self._display_menu(msg.menu_options, msg.selected_index, msg.menu_level)
                    
                elif msg.command_type == LCDCommand.CMD_DISPLAY_GAME_STATUS:
                    self._display_game_status(msg.text)
                    
                elif msg.command_type == LCDCommand.CMD_SET_BRIGHTNESS:
                    self._set_brightness(msg.brightness)
                
                self.get_logger().debug(f"Processed LCD command: type={msg.command_type}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing LCD command: {e}")
    
    def _encoder_event_callback(self, msg: RotaryEncoderEvent):
        """
        Handle rotary encoder events for menu navigation
        
        Args:
            msg: RotaryEncoderEvent message
        """
        try:
            with self.display_lock:
                if msg.event_type == RotaryEncoderEvent.EVENT_TYPE_ROTATION:
                    self._handle_menu_navigation(msg.direction)
                    
                elif msg.event_type == RotaryEncoderEvent.EVENT_TYPE_BUTTON_PRESS:
                    self._handle_menu_selection()
                
                self.get_logger().debug(f"Processed encoder event: type={msg.event_type}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing encoder event: {e}")
    
    def _handle_menu_navigation(self, direction: int):
        """
        Handle menu navigation based on encoder rotation
        
        Args:
            direction: Rotation direction (-1 for CCW, +1 for CW)
        """
        if self.menu_level == 0:
            # Main menu navigation
            if self.menu_option is None:
                self.menu_option = 0
            else:
                self.menu_option = max(0, min(len(self.menu_options) - 1, self.menu_option + direction))
        else:
            # Submenu navigation
            if self.menu_option is not None and self.menu_option < len(self.submenu_options):
                self.submenu_options[self.menu_option] = max(0, self.submenu_options[self.menu_option] + direction)
        
        self._render_menu()
    
    def _handle_menu_selection(self):
        """Handle menu selection (button press)"""
        self.menu_level = 1 - self.menu_level  # Toggle between 0 and 1
        self._render_menu()
    
    def _clear_display(self):
        """Clear the display"""
        if self.use_real_display:
            self.draw.rectangle((0, 0, self.display_width, self.display_height),
                              outline=0, fill=0)
            self.oled.image(self.image)
            self.oled.show()
        else:
            self.oled.clear()
    
    def _display_text(self, text: str, x: int = 3, y: int = 8, font_size: int = 0, invert: bool = False):
        """
        Display text on the screen
        
        Args:
            text: Text to display
            x: X position
            y: Y position
            font_size: Font size (0=small, 1=large)
            invert: Whether to invert display
        """
        font = self.bigfont if font_size == 1 else self.font
        
        if self.use_real_display:
            self._clear_display()
            self.draw.text((x, y), text, font=font, fill=1)
            self.oled.image(self.image)
            self.oled.show()
        else:
            self.oled.display_text(text, x, y)
        
        self.current_text = text
    
    def _display_game_status(self, status: str):
        """
        Display game status message
        
        Args:
            status: Status message to display
        """
        self._display_text(status, font_size=1)
        self.get_logger().info(f"Game status: {status}")
    
    def _display_menu(self, options: List[str], selected: int, level: int):
        """
        Display menu with options
        
        Args:
            options: List of menu options
            selected: Selected option index
            level: Menu level
        """
        self.menu_options = options
        self.menu_option = selected if selected >= 0 else None
        self.menu_level = level
        
        self._render_menu()
    
    def _render_menu(self):
        """Render the current menu state"""
        if self.render_thread is None or not self.render_thread.is_alive():
            self.render_thread = threading.Thread(target=self._do_render_menu)
            self.render_thread.start()
    
    def _do_render_menu(self):
        """Actually render the menu (in separate thread)"""
        try:
            if self.menu_level == 0:
                self._render_main_menu()
            else:
                self._render_submenu()
        except Exception as e:
            self.get_logger().error(f"Menu rendering error: {e}")
    
    def _render_main_menu(self):
        """Render main menu"""
        if not self.menu_options:
            return
        
        # Calculate visible range
        if self.menu_option is None or self.menu_option < self.row_count:
            start = 0
            end = min(self.row_count, len(self.menu_options))
        elif self.menu_option >= len(self.menu_options) - self.row_count:
            end = len(self.menu_options)
            start = max(0, end - self.row_count)
        else:
            start = self.menu_option
            end = start + self.row_count
        
        if self.use_real_display:
            self._clear_display()
            
            # Draw menu options
            top = 0
            for x in range(start, end):
                fill = 1
                if self.menu_option is not None and self.menu_option == x:
                    self.draw.rectangle([0, top, self.oled.width, top + 11], outline=0, fill=1)
                    fill = 0
                
                self.draw.text((3, top + 1), self.menu_options[x], font=self.font, fill=fill)
                top += 10
            
            self.oled.image(self.image)
            self.oled.show()
        else:
            self.oled.display_menu(self.menu_options[start:end], 
                                 self.menu_option - start if self.menu_option is not None else -1)
    
    def _render_submenu(self):
        """Render submenu"""
        if self.menu_option is None or self.menu_option >= len(self.submenu_options):
            return
        
        # Create submenu text based on selection
        text = f"Option {self.menu_option}: {self.submenu_options[self.menu_option]}"
        self._display_text(text, font_size=1)
    
    def _set_brightness(self, brightness: int):
        """
        Set display brightness
        
        Args:
            brightness: Brightness level (0-255)
        """
        if self.use_real_display and hasattr(self.oled, 'set_brightness'):
            self.oled.set_brightness(brightness)
        
        self.get_logger().debug(f"Display brightness set to: {brightness}")
    
    def _display_startup_message(self):
        """Display startup message"""
        self._display_text("ChessMate Ready", font_size=1)
        time.sleep(2)
        self._display_text("Waiting for commands...")
    
    def destroy_node(self):
        """Clean up resources when node is destroyed"""
        try:
            self.get_logger().info("Shutting down LCD Display Node")
            
            with self.display_lock:
                self._display_text("Shutting down...", font_size=1)
                time.sleep(1)
                self._clear_display()
            
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        
        super().destroy_node()


# Mock classes for development
class MockDisplay:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        
    def clear(self):
        print("Mock Display: Clear")
        
    def display_text(self, text, x, y):
        print(f"Mock Display: Text '{text}' at ({x}, {y})")
        
    def display_menu(self, options, selected):
        print(f"Mock Display: Menu {options}, selected: {selected}")


class MockImage:
    def __init__(self, width, height):
        self.width = width
        self.height = height


class MockDraw:
    def rectangle(self, *args, **kwargs):
        pass
        
    def text(self, pos, text, **kwargs):
        print(f"Mock Draw: '{text}' at {pos}")


class MockFont:
    pass


def main(args=None):
    """Main entry point for the LCD display node"""
    rclpy.init(args=args)
    
    try:
        node = LCDDisplayNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("Keyboard interrupt received")
        finally:
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start LCD display node: {e}")
        return 1
    
    finally:
        try:
            rclpy.shutdown()
        except:
            pass
    
    return 0


if __name__ == '__main__':
    exit(main())
