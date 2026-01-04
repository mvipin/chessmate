#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Topic-Based Chess Engine Server

This provides the same chess engine functionality as chess_engine_server.py
but uses topic-based communication instead of ROS2 services to work around
service communication issues in the Pi environment.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import chess
from chessmate.msg import ChessMove
from chessmate.srv import CalculateMove

# Local imports - use absolute imports for standalone execution
from chessmate.engine.stockfish_interface import StockfishInterface, DifficultyLevel, EngineConfig
from chessmate.engine.message_converters import MessageConverter


class TopicChessEngineServer(Node):
    """
    Topic-Based Chess Engine Server Node
    
    Provides chess engine functionality using topic-based communication:
    - engine/calculate_move_request -> engine/calculate_move_response
    """
    
    def __init__(self):
        super().__init__('topic_chess_engine_server')
        
        # Initialize components
        self.stockfish = StockfishInterface()
        self.converter = MessageConverter()
        
        # Topic-based service interface
        self.setup_topic_services()
        
        # Response tracking
        self.pending_requests = {}
        self.response_lock = threading.Lock()
        
        # Initialize engine
        if not self.stockfish.initialize():
            self.get_logger().error("❌ Failed to initialize Stockfish engine")
        else:
            stockfish_info = self.stockfish.get_engine_info()
            self.get_logger().info(f"✅ Stockfish initialized: {stockfish_info}")
            self.get_logger().info("🧠 Topic-based Chess Engine Server initialized")

    def destroy_node(self):
        """Clean up resources on shutdown"""
        try:
            if self.stockfish:
                self.stockfish.shutdown()
                self.get_logger().info("🔧 Stockfish engine shutdown complete")
        except Exception as e:
            self.get_logger().error(f"❌ Error during Stockfish shutdown: {e}")
        super().destroy_node()
    
    def setup_topic_services(self):
        """Setup topic-based service interfaces"""
        
        # Calculate move service (topic-based)
        self.calculate_request_subscriber = self.create_subscription(
            String, 'engine/calculate_move_request', self.handle_calculate_request, 10)
        self.calculate_response_publisher = self.create_publisher(
            String, 'engine/calculate_move_response', 10)
        
        self.get_logger().info("🔧 Topic-based service interfaces setup complete")
    
    def handle_calculate_request(self, msg):
        """Handle calculate move requests via topic"""
        try:
            # Parse request
            request_data = json.loads(msg.data)
            request_id = request_data.get('id', 'unknown')
            fen = request_data.get('fen', chess.STARTING_FEN)
            time_limit = request_data.get('time_limit', 3.0)
            skill_level = request_data.get('skill_level', 5)
            white_to_move = request_data.get('white_to_move', True)
            
            self.get_logger().info(f"🧠 Processing move calculation request {request_id}")
            self.get_logger().debug(f"   FEN: {fen}")
            self.get_logger().debug(f"   Time limit: {time_limit}s, Skill: {skill_level}")
            
            # Configure engine based on skill level (0-20)
            if 0 <= skill_level <= 20:
                # Set skill level directly
                self.stockfish.set_skill_level(skill_level)
                self.get_logger().debug(f"Set skill level to: {skill_level}")
            else:
                self.get_logger().warn(f"Invalid skill level: {skill_level}, using default (20)")
            
            # Calculate best move
            analysis = self.stockfish.get_best_move(
                fen, 
                time_limit if time_limit > 0 else None
            )
            
            if analysis and analysis.best_move:
                # Create a simple ChessMove manually (avoid complex converter)
                uci_move = analysis.best_move

                # Prepare response with basic move info
                response_data = {
                    'id': request_id,
                    'success': True,
                    'best_move': {
                        'from_square': uci_move[:2],
                        'to_square': uci_move[2:4],
                        'piece_type': 'pawn',  # Simplified - would need board analysis for accuracy
                        'move_type': 'promotion' if len(uci_move) == 5 else 'normal',
                        'promotion_piece': uci_move[4] if len(uci_move) == 5 else '',
                        'is_capture': False,  # Simplified - would need board analysis
                        'uci_move': uci_move
                    },
                    'evaluation': float(analysis.evaluation) if analysis.evaluation else 0.0,
                    'message': f"Best move calculated: {uci_move}",
                    'calculation_time': float(time_limit)
                }
                
                self.get_logger().info(f"✅ Calculated best move: {analysis.best_move} (eval: {analysis.evaluation})")
                
            else:
                # No move found
                response_data = {
                    'id': request_id,
                    'success': False,
                    'best_move': {
                        'from_square': '',
                        'to_square': '',
                        'piece_type': '',
                        'move_type': 'normal',
                        'promotion_piece': '',
                        'is_capture': False,
                        'uci_move': ''
                    },
                    'evaluation': 0.0,
                    'message': "No valid move found",
                    'calculation_time': 0.0
                }
                
                self.get_logger().error("❌ No valid move found by engine")
            
            # Send response
            response_msg = String()
            response_msg.data = json.dumps(response_data)
            self.calculate_response_publisher.publish(response_msg)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing calculate request: {e}")
            
            # Send error response
            error_response = {
                'id': request_data.get('id', 'unknown') if 'request_data' in locals() else 'unknown',
                'success': False,
                'best_move': {
                    'from_square': '',
                    'to_square': '',
                    'piece_type': '',
                    'move_type': 'normal',
                    'promotion_piece': '',
                    'is_capture': False,
                    'uci_move': ''
                },
                'evaluation': 0.0,
                'message': f"Engine error: {e}",
                'calculation_time': 0.0
            }
            
            error_msg = String()
            error_msg.data = json.dumps(error_response)
            self.calculate_response_publisher.publish(error_msg)
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.stockfish.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        engine_server = TopicChessEngineServer()
        rclpy.spin(engine_server)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            engine_server.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
