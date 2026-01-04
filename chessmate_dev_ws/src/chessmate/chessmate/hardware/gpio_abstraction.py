#!/usr/bin/env python3
"""
GPIO Abstraction Layer for ChessMate Hardware Interface

This module provides a hardware abstraction layer that works on both:
- Raspberry Pi (with actual GPIO hardware)
- Development machines (with mock/simulation)

The abstraction allows the same code to run in both environments.
"""

import logging
import threading
import time
from typing import Callable, Optional, Dict, Any
from enum import Enum

class GPIOMode(Enum):
    """GPIO pin modes"""
    INPUT = "INPUT"
    OUTPUT = "OUTPUT"

class GPIOPull(Enum):
    """GPIO pull-up/pull-down modes"""
    UP = "UP"
    DOWN = "DOWN"
    NONE = "NONE"

class GPIOEdge(Enum):
    """GPIO edge detection modes"""
    RISING = "RISING"
    FALLING = "FALLING"
    BOTH = "BOTH"

class GPIOAbstraction:
    """
    Abstract GPIO interface that can work with real hardware or simulation
    """

    @staticmethod
    def is_raspberry_pi() -> bool:
        """
        Detect if running on Raspberry Pi

        Returns:
            bool: True if running on Raspberry Pi, False otherwise
        """
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpuinfo = f.read()
                return 'BCM' in cpuinfo or 'Raspberry Pi' in cpuinfo
        except (FileNotFoundError, PermissionError):
            return False

    def __init__(self, use_real_gpio: bool = None):
        """
        Initialize GPIO abstraction
        
        Args:
            use_real_gpio: If None, auto-detect. If True, force real GPIO. If False, use mock.
        """
        self.logger = logging.getLogger(__name__)
        self._callbacks: Dict[int, Callable] = {}
        self._pin_states: Dict[int, bool] = {}
        self._pin_modes: Dict[int, GPIOMode] = {}
        
        # Auto-detect or use specified mode
        if use_real_gpio is None:
            self.use_real_gpio = self._detect_raspberry_pi()
        else:
            self.use_real_gpio = use_real_gpio
            
        self.logger.info(f"GPIO Abstraction initialized: {'Real GPIO' if self.use_real_gpio else 'Mock GPIO'}")
        
        # Initialize the appropriate GPIO backend
        if self.use_real_gpio:
            self._init_real_gpio()
        else:
            self._init_mock_gpio()
    
    def _detect_raspberry_pi(self) -> bool:
        """
        Detect if running on Raspberry Pi
        
        Returns:
            True if running on Raspberry Pi, False otherwise
        """
        try:
            with open('/proc/cpuinfo', 'r') as f:
                cpuinfo = f.read()
                return 'BCM' in cpuinfo and 'ARM' in cpuinfo
        except FileNotFoundError:
            return False
    
    def _init_real_gpio(self):
        """Initialize real Raspberry Pi GPIO"""
        try:
            from RPi import GPIO
            self.GPIO = GPIO
            self.GPIO.setmode(GPIO.BCM)
            # Disable GPIO warnings
            self.GPIO.setwarnings(False)
            self.logger.info("Real GPIO initialized successfully with BCM mode")
        except ImportError:
            self.logger.error("RPi.GPIO not available, falling back to mock GPIO")
            self.use_real_gpio = False
            self._init_mock_gpio()
    
    def _init_mock_gpio(self):
        """Initialize mock GPIO for development/testing"""
        self.GPIO = MockGPIO()
        self.logger.info("Mock GPIO initialized for development")
    
    def setup_pin(self, pin: int, mode: GPIOMode, pull: GPIOPull = GPIOPull.NONE):
        """
        Setup a GPIO pin
        
        Args:
            pin: Pin number
            mode: Pin mode (INPUT/OUTPUT)
            pull: Pull-up/pull-down configuration
        """
        self._pin_modes[pin] = mode
        
        if self.use_real_gpio:
            gpio_mode = self.GPIO.IN if mode == GPIOMode.INPUT else self.GPIO.OUT
            
            if pull == GPIOPull.UP:
                pull_mode = self.GPIO.PUD_UP
            elif pull == GPIOPull.DOWN:
                pull_mode = self.GPIO.PUD_DOWN
            else:
                pull_mode = self.GPIO.PUD_OFF
                
            self.GPIO.setup(pin, gpio_mode, pull_up_down=pull_mode)
        else:
            self.GPIO.setup(pin, mode.value, pull.value)
        
        # Initialize pin state
        if mode == GPIOMode.INPUT:
            self._pin_states[pin] = self.read_pin(pin)
        else:
            self._pin_states[pin] = False
    
    def read_pin(self, pin: int) -> bool:
        """
        Read the state of a GPIO pin
        
        Args:
            pin: Pin number
            
        Returns:
            Pin state (True/False)
        """
        if self.use_real_gpio:
            state = bool(self.GPIO.input(pin))
        else:
            state = self.GPIO.input(pin)
        
        self._pin_states[pin] = state
        return state
    
    def write_pin(self, pin: int, state: bool):
        """
        Write to a GPIO pin
        
        Args:
            pin: Pin number
            state: State to write (True/False)
        """
        if self._pin_modes.get(pin) != GPIOMode.OUTPUT:
            raise ValueError(f"Pin {pin} is not configured as OUTPUT")
        
        if self.use_real_gpio:
            self.GPIO.output(pin, self.GPIO.HIGH if state else self.GPIO.LOW)
        else:
            self.GPIO.output(pin, state)
        
        self._pin_states[pin] = state
    
    def add_event_detect(self, pin: int, edge: GPIOEdge, callback: Callable, bouncetime: int = 100):
        """
        Add edge detection to a pin
        
        Args:
            pin: Pin number
            edge: Edge type to detect
            callback: Callback function to call on edge detection
            bouncetime: Bounce time in milliseconds
        """
        self._callbacks[pin] = callback
        
        if self.use_real_gpio:
            gpio_edge = {
                GPIOEdge.RISING: self.GPIO.RISING,
                GPIOEdge.FALLING: self.GPIO.FALLING,
                GPIOEdge.BOTH: self.GPIO.BOTH
            }[edge]

            # Ensure bouncetime is valid for RPi.GPIO (must be > 0)
            if bouncetime <= 0:
                bouncetime = 100  # Default to 100ms if invalid

            print(f"DEBUG: Setting up GPIO pin {pin} with bouncetime={bouncetime}")
            self.GPIO.add_event_detect(pin, gpio_edge, callback=callback, bouncetime=bouncetime)
        else:
            self.GPIO.add_event_detect(pin, edge.value, callback, bouncetime)
    
    def remove_event_detect(self, pin: int):
        """
        Remove edge detection from a pin
        
        Args:
            pin: Pin number
        """
        if pin in self._callbacks:
            del self._callbacks[pin]
        
        if self.use_real_gpio:
            self.GPIO.remove_event_detect(pin)
        else:
            self.GPIO.remove_event_detect(pin)
    
    def cleanup(self):
        """Clean up GPIO resources"""
        if self.use_real_gpio:
            self.GPIO.cleanup()
        else:
            self.GPIO.cleanup()
        
        self._callbacks.clear()
        self._pin_states.clear()
        self._pin_modes.clear()
        self.logger.info("GPIO cleanup completed")


class MockGPIO:
    """
    Mock GPIO implementation for development and testing
    """
    
    # Constants to match RPi.GPIO
    BOARD = "BOARD"
    IN = "IN"
    OUT = "OUT"
    HIGH = True
    LOW = False
    PUD_UP = "PUD_UP"
    PUD_DOWN = "PUD_DOWN"
    PUD_OFF = "PUD_OFF"
    RISING = "RISING"
    FALLING = "FALLING"
    BOTH = "BOTH"
    
    def __init__(self):
        self.logger = logging.getLogger(f"{__name__}.MockGPIO")
        self._pin_states: Dict[int, bool] = {}
        self._pin_modes: Dict[int, str] = {}
        self._callbacks: Dict[int, Callable] = {}
        self._simulation_thread: Optional[threading.Thread] = None
        self._simulation_running = False
        
        # Start simulation thread for mock interactions
        self._start_simulation()
    
    def setmode(self, mode):
        """Set GPIO mode (mock)"""
        self.logger.debug(f"Mock GPIO mode set to: {mode}")
    
    def setup(self, pin: int, mode: str, pull: str = "PUD_OFF"):
        """Setup pin (mock)"""
        self._pin_modes[pin] = mode
        if mode == "INPUT":
            self._pin_states[pin] = True  # Default to HIGH for pull-up
        else:
            self._pin_states[pin] = False
        self.logger.debug(f"Mock GPIO pin {pin} setup: mode={mode}, pull={pull}")
    
    def input(self, pin: int) -> bool:
        """Read pin state (mock)"""
        return self._pin_states.get(pin, False)
    
    def output(self, pin: int, state: bool):
        """Write pin state (mock)"""
        self._pin_states[pin] = state
        self.logger.debug(f"Mock GPIO pin {pin} output: {state}")
    
    def add_event_detect(self, pin: int, edge: str, callback: Callable, bouncetime: int = 0):
        """Add event detection (mock)"""
        self._callbacks[pin] = callback
        self.logger.debug(f"Mock GPIO event detect added for pin {pin}: edge={edge}")
    
    def remove_event_detect(self, pin: int):
        """Remove event detection (mock)"""
        if pin in self._callbacks:
            del self._callbacks[pin]
        self.logger.debug(f"Mock GPIO event detect removed for pin {pin}")
    
    def cleanup(self):
        """Cleanup (mock)"""
        self._simulation_running = False
        if self._simulation_thread and self._simulation_thread.is_alive():
            self._simulation_thread.join(timeout=1.0)
        self.logger.debug("Mock GPIO cleanup completed")
    
    def _start_simulation(self):
        """Start simulation thread for mock user interactions"""
        self._simulation_running = True
        self._simulation_thread = threading.Thread(target=self._simulation_loop, daemon=True)
        self._simulation_thread.start()
    
    def _simulation_loop(self):
        """Simulation loop for generating mock events"""
        self.logger.info("Mock GPIO simulation started - generating periodic test events")
        
        counter = 0
        while self._simulation_running:
            time.sleep(5)  # Generate events every 5 seconds
            
            # Simulate rotary encoder rotation
            if 11 in self._callbacks:  # CLK pin
                counter += 1
                if counter % 2 == 0:
                    self.logger.debug("Simulating clockwise rotation")
                    self._callbacks[11](11)
                else:
                    self.logger.debug("Simulating counter-clockwise rotation")
                    self._callbacks[11](11)
            
            # Simulate button press every 10th event
            if counter % 10 == 0 and 13 in self._callbacks:  # Button pin
                self.logger.debug("Simulating button press")
                self._callbacks[13](13)
