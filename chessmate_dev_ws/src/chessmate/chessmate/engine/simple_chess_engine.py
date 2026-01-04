#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Simple Chess Engine Node
Basic chess engine integration without custom services (using standard ROS2 messages)
"""

import rclpy
from rclpy.node import Node
import chess
import time

# ROS2 standard message imports
from std_msgs.msg import String
from geometry_msgs.msg import Point

# ChessMate message imports
from chessmate.msg import BoardState, ChessMove

# Local imports
from chessmate.engine.stockfish_interface import StockfishInterface, DifficultyLevel, EngineConfig
from chessmate.engine.message_converters import MessageConverter


class SimpleChessEngine(Node):
    """
    Simple Chess Engine Node
    
    Subscribes to board state and publishes recommended moves
    Uses only standard ROS2 messages to avoid service generation issues
    """
    
    def __init__(self):
        super().__init__('simple_chess_engine')
        
        # Initialize components
        self.stockfish = StockfishInterface()
        self.converter = MessageConverter()
        
        # Game state
        self.current_board_state: BoardState = None
        self.last_analysis_time = 0.0
        self.analysis_interval = 2.0  # Analyze every 2 seconds
        
        # ROS2 Publishers
        self.move_publisher = self.create_publisher(
            ChessMove, 'chess_engine_move', 10)
        
        self.analysis_publisher = self.create_publisher(
            String, 'chess_engine_analysis', 10)
        
        # ROS2 Subscribers
        self.board_subscriber = self.create_subscription(
            BoardState, 'board_state', self.board_state_callback, 10)
        
        self.command_subscriber = self.create_subscription(
            String, 'chess_engine_command', self.command_callback, 10)
        
        # Timer for periodic analysis
        self.analysis_timer = self.create_timer(1.0, self.analysis_timer_callback)
        
        # Initialize engine
        if not self.stockfish.initialize():
            self.get_logger().error("Failed to initialize Stockfish engine")
        else:
            self.get_logger().info("Simple Chess Engine initialized successfully")
            
            # Set default difficulty
            config = EngineConfig.from_difficulty(DifficultyLevel.INTERMEDIATE)
            self.stockfish.config = config
            self.get_logger().info(f"Set difficulty to {DifficultyLevel.INTERMEDIATE.value}")
    
    def board_state_callback(self, msg: BoardState):
        """Handle board state updates"""
        self.current_board_state = msg
        self.get_logger().debug(f"Board state updated: {msg.fen_string[:20]}...")
    
    def command_callback(self, msg: String):
        """Handle engine commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received command: {command}")
        
        if command.startswith("difficulty:"):
            # Change difficulty: "difficulty:beginner"
            difficulty_str = command.split(":", 1)[1].strip()
            try:
                difficulty = DifficultyLevel(difficulty_str)
                config = EngineConfig.from_difficulty(difficulty)
                self.stockfish.config = config
                self.get_logger().info(f"Changed difficulty to {difficulty.value}")
                
                # Publish confirmation
                response = String()
                response.data = f"Difficulty changed to {difficulty.value}"
                self.analysis_publisher.publish(response)
                
            except ValueError:
                self.get_logger().warn(f"Invalid difficulty: {difficulty_str}")
                
        elif command == "analyze":
            # Force immediate analysis
            self.perform_analysis()
            
        elif command == "reset":
            # Reset to starting position
            self.stockfish.set_position(chess.STARTING_FEN)
            self.get_logger().info("Engine reset to starting position")
            
        elif command.startswith("move:"):
            # Make a move: "move:e2e4"
            move_uci = command.split(":", 1)[1].strip()
            self.make_move(move_uci)
    
    def analysis_timer_callback(self):
        """Periodic analysis timer"""
        current_time = time.time()
        if (self.current_board_state and 
            current_time - self.last_analysis_time > self.analysis_interval):
            self.perform_analysis()
            self.last_analysis_time = current_time
    
    def perform_analysis(self):
        """Perform chess analysis and publish results"""
        if not self.current_board_state:
            return
        
        try:
            fen = self.current_board_state.fen_string
            if not fen:
                return
            
            # Get best move
            analysis = self.stockfish.get_best_move(fen, time_limit=1.0)
            
            if analysis.best_move:
                # Convert to ChessMove and publish
                chess_move = self.converter.uci_to_chess_move(
                    analysis.best_move, self.current_board_state)
                
                chess_move.confidence = min(1.0, abs(analysis.evaluation) / 100.0)
                chess_move.timestamp = self.get_clock().now().nanoseconds
                
                self.move_publisher.publish(chess_move)
                
                # Publish analysis text
                analysis_text = String()
                if abs(analysis.evaluation) < 50:
                    eval_text = "Position is equal"
                elif analysis.evaluation > 0:
                    eval_text = f"White is better (+{analysis.evaluation/100:.1f})"
                else:
                    eval_text = f"Black is better ({analysis.evaluation/100:.1f})"
                
                analysis_text.data = (
                    f"Best move: {analysis.best_move} | "
                    f"Eval: {eval_text} | "
                    f"Time: {analysis.analysis_time:.2f}s"
                )
                self.analysis_publisher.publish(analysis_text)
                
                self.get_logger().info(
                    f"Analysis: {analysis.best_move} "
                    f"(eval: {analysis.evaluation:.1f}, "
                    f"time: {analysis.analysis_time:.2f}s)")
            
        except Exception as e:
            self.get_logger().error(f"Analysis failed: {e}")
    
    def make_move(self, move_uci: str):
        """Make a move and update position"""
        if not self.current_board_state:
            self.get_logger().warn("No board state available for move")
            return
        
        try:
            # Validate move
            is_legal, message, resulting_fen = self.stockfish.validate_move(
                self.current_board_state.fen_string, move_uci)
            
            if is_legal and resulting_fen:
                # Update engine position
                self.stockfish.set_position(resulting_fen)
                
                # Create and publish move
                chess_move = self.converter.uci_to_chess_move(
                    move_uci, self.current_board_state)
                chess_move.timestamp = self.get_clock().now().nanoseconds
                
                self.move_publisher.publish(chess_move)
                
                # Publish confirmation
                response = String()
                response.data = f"Move {move_uci} executed: {message}"
                self.analysis_publisher.publish(response)
                
                self.get_logger().info(f"Move executed: {move_uci}")
            else:
                self.get_logger().warn(f"Illegal move {move_uci}: {message}")
                
                # Publish error
                response = String()
                response.data = f"Illegal move {move_uci}: {message}"
                self.analysis_publisher.publish(response)
                
        except Exception as e:
            self.get_logger().error(f"Move execution failed: {e}")
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.stockfish:
            self.stockfish.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    chess_engine = SimpleChessEngine()
    
    try:
        rclpy.spin(chess_engine)
    except KeyboardInterrupt:
        pass
    finally:
        chess_engine.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
