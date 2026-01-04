#!/usr/bin/env python3
"""
Configuration Loader for ChessMate Unified Hardware

Cross-platform configuration management that automatically detects the platform
and loads appropriate settings for Linux host or Raspberry Pi deployment.

Features:
- Automatic platform detection
- Platform-specific configuration overrides
- Parameter validation
- Environment variable support
- Configuration merging and inheritance
"""

import os
import yaml
import platform
import subprocess
from typing import Dict, Any, Optional
from pathlib import Path


class ConfigLoader:
    """
    Unified configuration loader for ChessMate hardware
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize configuration loader
        
        Args:
            config_file: Path to configuration file (optional)
        """
        self.platform_type = self._detect_platform()
        self.config_file = config_file or self._find_config_file()
        self.config = {}
        self.load_config()
    
    def _detect_platform(self) -> str:
        """
        Detect the current platform
        
        Returns:
            Platform type: 'raspberry_pi', 'linux_host', or 'unknown'
        """
        try:
            # Check for Raspberry Pi specific indicators
            if os.path.exists('/proc/device-tree/model'):
                with open('/proc/device-tree/model', 'r') as f:
                    model = f.read().lower()
                    if 'raspberry pi' in model:
                        return 'raspberry_pi'
            
            # Check for GPIO directory (Raspberry Pi indicator)
            if os.path.exists('/sys/class/gpio'):
                try:
                    # Try to detect BCM GPIO (Raspberry Pi specific)
                    gpio_files = os.listdir('/sys/class/gpio')
                    if any('gpiochip' in f for f in gpio_files):
                        # Additional check for Raspberry Pi
                        if os.path.exists('/boot/config.txt') or os.path.exists('/boot/firmware/config.txt'):
                            return 'raspberry_pi'
                except (OSError, PermissionError):
                    pass
            
            # Check system information
            system_info = platform.uname()
            
            # Check for common Raspberry Pi indicators
            if 'arm' in system_info.machine.lower():
                # Could be Raspberry Pi or other ARM device
                if 'raspberrypi' in system_info.node.lower():
                    return 'raspberry_pi'
            
            # Default to Linux host for other Linux systems
            if system_info.system.lower() == 'linux':
                return 'linux_host'
            
            return 'unknown'
            
        except Exception as e:
            print(f"Platform detection error: {e}")
            return 'unknown'
    
    def _find_config_file(self) -> str:
        """
        Find the configuration file in standard locations
        
        Returns:
            Path to configuration file
        """
        # Possible configuration file locations
        search_paths = [
            # Environment variable override
            os.environ.get('CHESSMATE_CONFIG'),
            
            # Current directory
            './unified_hardware_config.yaml',
            './config/unified_hardware_config.yaml',
            
            # Package directory
            os.path.join(os.path.dirname(__file__), '..', 'config', 'unified_hardware_config.yaml'),
            
            # System-wide locations
            '/etc/chessmate/unified_hardware_config.yaml',
            '/opt/chessmate/config/unified_hardware_config.yaml',
            
            # User home directory
            os.path.expanduser('~/.chessmate/unified_hardware_config.yaml'),
        ]
        
        for path in search_paths:
            if path and os.path.exists(path):
                return path
        
        # If no config file found, create a default one
        default_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'unified_hardware_config.yaml')
        print(f"Warning: No configuration file found. Using default: {default_path}")
        return default_path
    
    def load_config(self):
        """Load and process configuration file"""
        try:
            with open(self.config_file, 'r') as f:
                self.config = yaml.safe_load(f)
            
            # Apply platform-specific overrides
            self._apply_platform_overrides()
            
            # Apply environment variable overrides
            self._apply_env_overrides()
            
            # Validate configuration
            self._validate_config()
            
            print(f"Configuration loaded successfully for platform: {self.platform_type}")
            
        except FileNotFoundError:
            print(f"Configuration file not found: {self.config_file}")
            self.config = self._get_default_config()
        except yaml.YAMLError as e:
            print(f"YAML parsing error: {e}")
            self.config = self._get_default_config()
        except Exception as e:
            print(f"Configuration loading error: {e}")
            self.config = self._get_default_config()
    
    def _apply_platform_overrides(self):
        """Apply platform-specific configuration overrides"""
        if 'platform_overrides' in self.config and self.platform_type in self.config['platform_overrides']:
            overrides = self.config['platform_overrides'][self.platform_type]
            self._merge_config(self.config, overrides)
    
    def _apply_env_overrides(self):
        """Apply environment variable overrides"""
        env_mappings = {
            'CHESSMATE_CHESSBOARD_PORT': ['serial', 'chessboard_port'],
            'CHESSMATE_ROBOT_PORT': ['serial', 'robot_port'],
            'CHESSMATE_BAUD_RATE': ['serial', 'baud_rate'],
            'CHESSMATE_USE_MOCK': ['serial', 'use_mock_hardware'],
            'CHESSMATE_LOG_LEVEL': ['logging', 'console_level'],
            'CHESSMATE_AUTO_CALIBRATE': ['calibration', 'auto_calibrate'],
        }
        
        for env_var, config_path in env_mappings.items():
            if env_var in os.environ:
                value = os.environ[env_var]
                
                # Type conversion
                if value.lower() in ['true', 'false']:
                    value = value.lower() == 'true'
                elif value.isdigit():
                    value = int(value)
                elif self._is_float(value):
                    value = float(value)
                
                # Set nested configuration value
                self._set_nested_value(self.config, config_path, value)
    
    def _merge_config(self, base: Dict[str, Any], override: Dict[str, Any]):
        """Recursively merge configuration dictionaries"""
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(base[key], value)
            else:
                base[key] = value
    
    def _set_nested_value(self, config: Dict[str, Any], path: list, value: Any):
        """Set a nested configuration value"""
        current = config
        for key in path[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]
        current[path[-1]] = value
    
    def _is_float(self, value: str) -> bool:
        """Check if string represents a float"""
        try:
            float(value)
            return True
        except ValueError:
            return False
    
    def _validate_config(self):
        """Validate configuration parameters"""
        if not self.config.get('validation', {}).get('validate_on_startup', True):
            return
        
        # Validate serial ports
        serial_config = self.config.get('serial', {})
        if not serial_config.get('use_mock_hardware', False):
            chessboard_port = serial_config.get('chessboard_port')
            robot_port = serial_config.get('robot_port')
            
            if chessboard_port == robot_port:
                print("Warning: Chessboard and robot ports are the same")
        
        # Validate hardware parameters
        hardware_config = self.config.get('hardware', {})
        scara_config = hardware_config.get('scara', {})
        
        if scara_config.get('link1_length', 0) <= 0:
            print("Warning: Invalid link1_length")
        if scara_config.get('link2_length', 0) <= 0:
            print("Warning: Invalid link2_length")
        
        # Validate safety parameters
        safety_config = hardware_config.get('safety', {})
        max_vel = safety_config.get('max_joint_velocity', 0)
        max_acc = safety_config.get('max_joint_acceleration', 0)
        
        if max_vel <= 0 or max_acc <= 0:
            print("Warning: Invalid safety parameters")
    
    def _get_default_config(self) -> Dict[str, Any]:
        """Get minimal default configuration"""
        return {
            'platform': {'auto_detect': True, 'override': ''},
            'serial': {
                'chessboard_port': '/dev/chessboard',
                'robot_port': '/dev/robot',
                'baud_rate': 9600,
                'timeout': 2.0,
                'use_mock_hardware': True
            },
            'protocol': {
                'chessboard_protocol': 'character',
                'robot_protocol': 'character',
                'command_terminator': '\n',
                'response_timeout': 1.0,
                'max_retries': 3
            },
            'hardware': {
                'scara': {
                    'link1_length': 0.202,
                    'link2_length': 0.190,
                    'z_axis_max': 0.050
                },
                'safety': {
                    'emergency_stop_enabled': True,
                    'max_joint_velocity': 1.0,
                    'max_joint_acceleration': 2.0
                }
            },
            'publishing': {
                'sensor_publish_rate': 10.0,
                'status_publish_rate': 1.0
            },
            'logging': {
                'console_level': 'INFO'
            }
        }
    
    def get(self, key_path: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation
        
        Args:
            key_path: Dot-separated path (e.g., 'serial.baud_rate')
            default: Default value if key not found
            
        Returns:
            Configuration value
        """
        keys = key_path.split('.')
        current = self.config
        
        try:
            for key in keys:
                current = current[key]
            return current
        except (KeyError, TypeError):
            return default
    
    def get_platform_type(self) -> str:
        """Get detected platform type"""
        return self.platform_type
    
    def get_serial_config(self) -> Dict[str, Any]:
        """Get serial configuration for current platform"""
        base_config = self.config.get('serial', {})
        
        # Apply platform-specific serial settings
        platform_serial = base_config.get(self.platform_type, {})
        result = base_config.copy()
        result.update(platform_serial)
        
        # Remove platform-specific sections from result
        for platform in ['linux_host', 'raspberry_pi', 'mock']:
            result.pop(platform, None)
        
        return result
    
    def get_protocol_config(self) -> Dict[str, Any]:
        """Get protocol configuration"""
        return self.config.get('protocol', {})
    
    def get_hardware_config(self) -> Dict[str, Any]:
        """Get hardware configuration"""
        return self.config.get('hardware', {})
    
    def get_safety_config(self) -> Dict[str, Any]:
        """Get safety configuration"""
        return self.config.get('hardware', {}).get('safety', {})
    
    def get_publishing_config(self) -> Dict[str, Any]:
        """Get publishing rate configuration"""
        return self.config.get('publishing', {})
    
    def is_mock_mode(self) -> bool:
        """Check if running in mock hardware mode"""
        hardware_mode = self.config.get('hardware_mode', 'real')
        return (hardware_mode in ['mock', 'simulation'] or
                self.config.get('use_mock_hardware', False))
    
    def print_config_summary(self):
        """Print configuration summary"""
        print("=" * 50)
        print("ChessMate Hardware Configuration Summary")
        print("=" * 50)
        print(f"Platform: {self.platform_type}")
        print(f"Config File: {self.config_file}")
        print(f"Mock Mode: {self.is_mock_mode()}")
        
        serial_config = self.get_serial_config()
        print(f"Chessboard Port: {serial_config.get('chessboard_port')}")
        print(f"Robot Port: {serial_config.get('robot_port')}")
        print(f"Baud Rate: {serial_config.get('baud_rate')}")
        
        protocol_config = self.get_protocol_config()
        print(f"Chessboard Protocol: {protocol_config.get('chessboard_protocol')}")
        print(f"Robot Protocol: {protocol_config.get('robot_protocol')}")
        
        print("=" * 50)


# Global configuration instance
_config_instance = None


def get_config(config_file: Optional[str] = None) -> ConfigLoader:
    """
    Get global configuration instance (singleton pattern)
    
    Args:
        config_file: Path to configuration file (optional)
        
    Returns:
        ConfigLoader instance
    """
    global _config_instance
    if _config_instance is None:
        _config_instance = ConfigLoader(config_file)
    return _config_instance


def reload_config(config_file: Optional[str] = None):
    """Reload configuration (useful for testing)"""
    global _config_instance
    _config_instance = ConfigLoader(config_file)
    return _config_instance
