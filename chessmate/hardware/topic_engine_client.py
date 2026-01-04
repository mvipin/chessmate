#!/usr/bin/env python3
"""
Topic-Based Engine Client

This provides the same interface as ROS2 service clients but uses
topic-based communication to work around service issues.
"""

from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
from chessmate.srv import CalculateMove
from chessmate.msg import ChessMove

class TopicEngineClient:
    """Topic-based client that mimics ROS2 service client interface"""
    
    def __init__(self, node: Node, service_name: str):
        self.node = node
        self.service_name = service_name
        
        # Topic names based on service name
        request_topic = f"{service_name}_request"
        response_topic = f"{service_name}_response"
        
        # Publishers and subscribers
        self.request_publisher = node.create_publisher(String, request_topic, 10)
        self.response_subscriber = node.create_subscription(
            String, response_topic, self._handle_response, 10)
        
        # Response tracking
        self.pending_requests = {}
        self.response_lock = threading.Lock()
        
        node.get_logger().info(f"🔧 Topic-based client created for {service_name}")
    
    def _handle_response(self, msg):
        """Handle incoming response"""
        try:
            response_data = json.loads(msg.data)
            request_id = response_data.get('id')
            
            with self.response_lock:
                if request_id in self.pending_requests:
                    self.pending_requests[request_id] = response_data
                    
        except Exception as e:
            self.node.get_logger().error(f"❌ Error processing response: {e}")
    
    def wait_for_service(self, timeout_sec=5.0):
        """Wait for service to be available (always returns True for topics)"""
        # Topics don't need service discovery, so this always succeeds
        time.sleep(0.1)  # Brief delay to ensure publishers/subscribers are ready
        return True
    
    def call_async(self, request):
        """Asynchronous call that returns a future-like object"""
        return TopicFuture(self, request)
    
    def call(self, request):
        """Synchronous call (blocks until response)"""
        future = self.call_async(request)
        
        # Wait for response with timeout
        start_time = time.time()
        timeout = 10.0  # 10 second timeout
        
        while (time.time() - start_time) < timeout:
            if future.done():
                return future.result()
            time.sleep(0.01)
        
        # Timeout
        self.node.get_logger().error("❌ Topic-based service call timed out")
        return None
    
    def _send_request(self, request, request_id):
        """Send request via topic"""
        try:
            # Convert ROS2 service request to JSON
            request_data = {
                'id': request_id,
                'fen': request.fen,
                'time_limit': request.time_limit,
                'skill_level': request.skill_level,
                'white_to_move': request.white_to_move
            }
            
            # Publish request
            request_msg = String()
            request_msg.data = json.dumps(request_data)
            
            with self.response_lock:
                self.pending_requests[request_id] = None
            
            self.request_publisher.publish(request_msg)
            
        except Exception as e:
            self.node.get_logger().error(f"❌ Error sending request: {e}")

class TopicFuture:
    """Future-like object for topic-based async calls"""
    
    def __init__(self, client: TopicEngineClient, request):
        self.client = client
        self.request = request
        self.request_id = f"req_{int(time.time() * 1000000)}"
        self._result = None
        self._done = False
        
        # Send the request
        self.client._send_request(request, self.request_id)
    
    def done(self):
        """Check if response is ready"""
        if self._done:
            return True
            
        with self.client.response_lock:
            response_data = self.client.pending_requests.get(self.request_id)
            if response_data is not None:
                self._result = self._convert_response(response_data)
                self._done = True
                # Clean up
                self.client.pending_requests.pop(self.request_id, None)
                return True
        
        return False
    
    def result(self):
        """Get the result (blocks until available)"""
        if self._done:
            return self._result
            
        # Wait for result
        start_time = time.time()
        timeout = 10.0
        
        while (time.time() - start_time) < timeout:
            if self.done():
                return self._result
            time.sleep(0.01)
        
        # Timeout
        return None
    
    def _convert_response(self, response_data):
        """Convert JSON response back to ROS2 service response"""
        try:
            response = CalculateMove.Response()
            response.success = response_data.get('success', False)
            response.message = response_data.get('message', '')
            response.evaluation = response_data.get('evaluation', 0.0)
            response.calculation_time = response_data.get('calculation_time', 0.0)
            
            # Convert best_move
            best_move_data = response_data.get('best_move', {})
            response.best_move = ChessMove()
            response.best_move.from_square = best_move_data.get('from_square', '')
            response.best_move.to_square = best_move_data.get('to_square', '')
            response.best_move.piece_type = best_move_data.get('piece_type', '')
            response.best_move.promotion_piece = best_move_data.get('promotion_piece', '')
            response.best_move.is_capture = best_move_data.get('is_capture', False)
            response.best_move.move_type = best_move_data.get('move_type', 'normal')
            
            return response
            
        except Exception as e:
            self.client.node.get_logger().error(f"❌ Error converting response: {e}")
            # Return error response
            response = CalculateMove.Response()
            response.success = False
            response.message = f"Response conversion error: {e}"
            return response
