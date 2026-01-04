from setuptools import find_packages, setup

package_name = 'chessmate_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/unified_hardware.launch.py',
            'launch/integration_testing.launch.py',
            'launch/pi_headless_testing.launch.py',
            'launch/host_visualization.launch.py',
            'launch/distributed_testing.launch.py'
        ]),
        ('share/' + package_name + '/config', [
            'config/unified_hardware_config.yaml'
        ]),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
        'pillow',  # For LCD image processing
        'adafruit-circuitpython-ssd1306',
        'adafruit-circuitpython-busio',
        # Alternative: 'luma.oled',  # Uncomment if using luma instead
    ],
    zip_safe=True,
    maintainer='smtuser',
    maintainer_email='rubbotix@gmail.com',
    description='ChessMate hardware interface package for Raspberry Pi GPIO, LCD, and Arduino communication',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Production nodes (actively used)
            'arduino_communication_node = chessmate_hardware.arduino_communication_node:main',
            'unified_arduino_bridge = chessmate_hardware.unified_arduino_bridge:main',
            'topic_arduino_communication = chessmate_hardware.topic_arduino_communication:main',
            'game_management_node = chessmate_hardware.game_management_node:main',

            # UI components (for Raspberry Pi deployment)
            'rotary_encoder_node = chessmate_hardware.rotary_encoder_node:main',
            'lcd_display_node = chessmate_hardware.lcd_display_node:main',
            'robot_animation_controller = chessmate_hardware.robot_animation_controller:main',

            # Test utilities
            'test_arduino_serial = chessmate_hardware.test_arduino_serial:main',
            'unified_hardware_test = chessmate_hardware.unified_hardware_test:main',
        ],
    },
)
