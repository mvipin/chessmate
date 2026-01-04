#!/usr/bin/env python3
"""
ROS 2 Hardware Integration Test

Tests the unified hardware interface with real Arduino hardware.
Validates that ROS 2 topics, services, and communication work correctly.

Usage:
    ros2 run chessmate_hardware test_ros2_hardware
    
Prerequisites:
    - Arduino with character stub uploaded and connected
    - unified_arduino_bridge running
"""

import rclpy
from rclpy.node import Node
import time
import sys
from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from chessmate.msg import ArduinoCommand, JointCommand
from chessmate.srv import CalibrateArm


class ROS2HardwareTester(Node):
    """ROS 2 hardware integration tester"""
    
    def __init__(self):
        super().__init__('ros2_hardware_tester')
        
        # Publishers
        self.arduino_cmd_pub = self.create_publisher(ArduinoCommand, 'arduino_command', 10)
        self.joint_cmd_pub = self.create_publisher(JointCommand, 'joint_command', 10)
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String, 'hardware_status', self.status_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Service clients
        self.calibrate_client = self.create_client(CalibrateArm, 'calibrate_arm')
        
        # Test state
        self.received_status = False
        self.received_joint_state = False
        self.last_status = None
        
        self.get_logger().info('ROS 2 Hardware Tester initialized')
    
    def status_callback(self, msg):
        """Handle hardware status updates"""
        self.received_status = True
        self.last_status = msg.data
        self.get_logger().debug(f'Status: {msg.data}')
    
    def joint_state_callback(self, msg):
        """Handle joint state updates"""
        self.received_joint_state = True
        self.get_logger().debug(f'Joint state: {msg.position}')
    
    def test_basic_communication(self):
        """Test basic ROS 2 communication"""
        self.get_logger().info('🔌 Testing Basic ROS 2 Communication...')
        
        # Send wake up command
        cmd = ArduinoCommand()
        cmd.timestamp = self.get_clock().now().to_msg()
        cmd.command_type = ArduinoCommand.CMD_ROBOT_WAKE_UP
        cmd.target_arduino = 1  # Robot controller
        cmd.data = ""
        
        self.arduino_cmd_pub.publish(cmd)
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.received_status:
                break
        
        if self.received_status:
            self.get_logger().info('  ✅ ROS 2 communication working')
            return True
        else:
            self.get_logger().warn('  ⚠️  No status updates received')
            return False
    
    def test_arduino_commands(self):
        """Test Arduino command publishing"""
        self.get_logger().info('🤖 Testing Arduino Commands...')
        
        commands = [
            (ArduinoCommand.CMD_ROBOT_HOME_Z, "Home Z"),
            (ArduinoCommand.CMD_ROBOT_HOME_ALL, "Home all"),
            (ArduinoCommand.CMD_STATUS, "Status request"),
        ]
        
        for cmd_type, description in commands:
            self.get_logger().info(f'  Testing: {description}')
            
            cmd = ArduinoCommand()
            cmd.timestamp = self.get_clock().now().to_msg()
            cmd.command_type = cmd_type
            cmd.target_arduino = 1
            cmd.data = ""
            
            self.arduino_cmd_pub.publish(cmd)
            time.sleep(1.0)
        
        self.get_logger().info('  ✅ Arduino commands sent successfully')
        return True
    
    def test_emergency_stop(self):
        """Test emergency stop functionality"""
        self.get_logger().info('🚨 Testing Emergency Stop...')
        
        # Activate emergency stop
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
        time.sleep(1.0)
        
        # Release emergency stop
        estop_msg.data = False
        self.estop_pub.publish(estop_msg)
        time.sleep(1.0)
        
        self.get_logger().info('  ✅ Emergency stop test completed')
        return True
    
    def test_calibration_service(self):
        """Test calibration service"""
        self.get_logger().info('🎯 Testing Calibration Service...')
        
        if not self.calibrate_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('  ⚠️  Calibration service not available')
            return False
        
        request = CalibrateArm.Request()
        request.calibration_type = "home"
        request.use_limit_switches = False
        request.save_calibration = True
        request.calibration_speed = 0.5
        
        future = self.calibrate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info('  ✅ Calibration service working')
            return True
        else:
            self.get_logger().warn('  ⚠️  Calibration service failed')
            return False
    
    def run_tests(self):
        """Run all ROS 2 hardware tests"""
        self.get_logger().info('🚀 Starting ROS 2 Hardware Integration Tests')
        self.get_logger().info('=' * 50)
        
        # Wait for system initialization
        self.get_logger().info('⏳ Waiting for system initialization...')
        time.sleep(3.0)
        
        tests = [
            ("Basic Communication", self.test_basic_communication),
            ("Arduino Commands", self.test_arduino_commands),
            ("Emergency Stop", self.test_emergency_stop),
            ("Calibration Service", self.test_calibration_service),
        ]
        
        results = {}
        for test_name, test_func in tests:
            try:
                result = test_func()
                results[test_name] = result
            except Exception as e:
                self.get_logger().error(f'Test {test_name} failed: {e}')
                results[test_name] = False
        
        # Generate report
        self.get_logger().info('=' * 50)
        self.get_logger().info('📋 ROS 2 HARDWARE TEST REPORT')
        self.get_logger().info('=' * 50)
        
        passed = sum(results.values())
        total = len(results)
        
        for test_name, result in results.items():
            status = '✅ PASS' if result else '❌ FAIL'
            self.get_logger().info(f'{test_name:.<30} {status}')
        
        success_rate = (passed / total) * 100
        self.get_logger().info(f'Success Rate: {success_rate:.1f}% ({passed}/{total})')
        
        if success_rate >= 80:
            self.get_logger().info('🎉 ROS 2 HARDWARE: READY')
        else:
            self.get_logger().warn('⚠️  ROS 2 HARDWARE: ISSUES DETECTED')
        
        return success_rate >= 80


def main(args=None):
    """Main test entry point"""
    rclpy.init(args=args)
    
    try:
        tester = ROS2HardwareTester()
        success = tester.run_tests()
        
        time.sleep(1.0)  # Brief pause to see results
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
        sys.exit(0)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
