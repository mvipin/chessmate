#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Chess Game Manager Node
Manages chess game state and coordinates between board sensors, engine, and robot
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import chess
import time
from typing import Optional, Dict

# ROS2 message imports
from chessmate.msg import BoardState, ChessMove, ChessPiece
from chessmate.srv import (
    GetBestMove, EvaluatePosition, ValidateMove, UpdateGameState
)
from chessmate.action import ExecuteChessMove

# Local imports
from chessmate.engine.stockfish_interface import StockfishInterface, DifficultyLevel, EngineConfig
from chessmate.engine.message_converters import MessageConverter


class GameMode:
    """Game mode constants"""
    HUMAN_VS_ENGINE = "human_vs_engine"
    ENGINE_VS_ENGINE = "engine_vs_engine" 
    ANALYSIS = "analysis"


class ChessGameManager(Node):
    """
    Chess Game Manager Node
    
    Coordinates chess game flow:
    - Receives board state updates from sensors
    - Requests moves from Stockfish engine
    - Sends moves to robot for execution
    - Manages game state and rules
    """
    
    def __init__(self):
        super().__init__('chess_game_manager')
        
        # Initialize components
        self.stockfish = StockfishInterface()
        self.converter = MessageConverter()
        
        # Game state
        self.current_board_state: Optional[BoardState] = None
        self.game_mode = GameMode.HUMAN_VS_ENGINE
        self.engine_plays_white = False
        self.game_active = False
        self.last_move: Optional[ChessMove] = None
        
        # ROS2 Publishers
        self.move_publisher = self.create_publisher(
            ChessMove, 'chess_engine_moves', 10)
        
        # ROS2 Subscribers
        self.board_subscriber = self.create_subscription(
            BoardState, 'board_state', self.board_state_callback, 10)
        
        # ROS2 Services (Chess Engine Services)
        self.get_best_move_service = self.create_service(
            GetBestMove, 'get_best_move', self.get_best_move_callback)
        
        self.evaluate_position_service = self.create_service(
            EvaluatePosition, 'evaluate_position', self.evaluate_position_callback)
        
        self.validate_move_service = self.create_service(
            ValidateMove, 'validate_move', self.validate_move_callback)
        
        self.update_game_state_service = self.create_service(
            UpdateGameState, 'update_game_state', self.update_game_state_callback)
        
        # ROS2 Action Client (for robot move execution)
        self.execute_move_client = ActionClient(
            self, ExecuteChessMove, 'execute_chess_move')
        
        # Initialize engine
        if not self.stockfish.initialize():
            self.get_logger().error("Failed to initialize Stockfish engine")
        else:
            self.get_logger().info("Chess Game Manager initialized successfully")
    
    def board_state_callback(self, msg: BoardState):
        """Handle board state updates from sensors"""
        self.current_board_state = msg
        self.get_logger().debug(f"Board state updated: {msg.fen_string}")
        
        # If it's engine's turn and game is active, generate move
        if self.game_active and self.is_engine_turn():
            self.generate_engine_move()
    
    def is_engine_turn(self) -> bool:
        """Check if it's the engine's turn to move"""
        if not self.current_board_state:
            return False
        
        if self.game_mode == GameMode.ENGINE_VS_ENGINE:
            return True
        elif self.game_mode == GameMode.HUMAN_VS_ENGINE:
            white_to_move = self.current_board_state.active_color == "white"
            return white_to_move == self.engine_plays_white
        else:
            return False
    
    def generate_engine_move(self):
        """Generate and execute engine move"""
        if not self.current_board_state:
            return
        
        try:
            # Get best move from engine
            analysis = self.stockfish.get_best_move(
                self.current_board_state.fen_string)
            
            if analysis.best_move:
                # Convert to ChessMove message
                chess_move = self.converter.uci_to_chess_move(
                    analysis.best_move, self.current_board_state)
                
                chess_move.confidence = min(1.0, abs(analysis.evaluation) / 100.0)
                chess_move.timestamp = self.get_clock().now().nanoseconds
                
                # Publish move
                self.move_publisher.publish(chess_move)
                
                # Execute move on robot
                self.execute_move_on_robot(chess_move)
                
                self.get_logger().info(
                    f"Engine move: {chess_move.from_square}->{chess_move.to_square} "
                    f"(eval: {analysis.evaluation:.1f})")
            
        except Exception as e:
            self.get_logger().error(f"Failed to generate engine move: {e}")
    
    def execute_move_on_robot(self, move: ChessMove):
        """Send move to robot for execution"""
        if not self.execute_move_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Execute move action server not available")
            return
        
        # Create action goal
        goal = ExecuteChessMove.Goal()
        goal.move = move
        goal.animate_move = True
        goal.move_speed = 1.0
        
        # Send goal
        future = self.execute_move_client.send_goal_async(goal)
        future.add_done_callback(self.move_execution_callback)
    
    def move_execution_callback(self, future):
        """Handle move execution result"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Move execution rejected")
            return
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.move_result_callback)
    
    def move_result_callback(self, future):
        """Handle move execution completion"""
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Move executed successfully: {result.completion_message}")
        else:
            self.get_logger().error(f"Move execution failed: {result.completion_message}")
    
    # Service Callbacks
    
    def get_best_move_callback(self, request, response):
        """Handle GetBestMove service requests"""
        try:
            # Configure engine based on difficulty
            if request.difficulty_level:
                difficulty = DifficultyLevel(request.difficulty_level)
                config = EngineConfig.from_difficulty(difficulty)
                self.stockfish.config = config
            
            # Get analysis
            analysis = self.stockfish.get_best_move(
                request.fen_string,
                request.time_limit if request.time_limit > 0 else None,
                request.depth_limit if request.depth_limit > 0 else None
            )
            
            if analysis.best_move:
                # Convert to ChessMove using proper board state
                board_state = self.converter.fen_to_board_state(request.fen_string)

                response.best_move = self.converter.uci_to_chess_move(
                    analysis.best_move, board_state)
                response.best_move.confidence = min(1.0, abs(analysis.evaluation) / 100.0)
            
            response.evaluation = analysis.evaluation
            response.analysis_time = analysis.analysis_time
            response.nodes_searched = analysis.nodes_searched
            response.engine_info = self.stockfish.get_engine_info()
            
        except Exception as e:
            self.get_logger().error(f"GetBestMove service error: {e}")
            response.engine_info = f"Error: {e}"
        
        return response
    
    def evaluate_position_callback(self, request, response):
        """Handle EvaluatePosition service requests"""
        try:
            analysis = self.stockfish.evaluate_position(
                request.fen_string, request.analysis_time)
            
            response.evaluation = analysis.evaluation
            response.mate_in = analysis.mate_in or 0.0
            
            # Generate evaluation text
            if analysis.mate_in:
                if analysis.mate_in > 0:
                    response.evaluation_text = f"White mates in {analysis.mate_in}"
                else:
                    response.evaluation_text = f"Black mates in {abs(analysis.mate_in)}"
            elif abs(analysis.evaluation) < 50:
                response.evaluation_text = "Position is equal"
            elif analysis.evaluation > 0:
                response.evaluation_text = f"White is better (+{analysis.evaluation/100:.1f})"
            else:
                response.evaluation_text = f"Black is better ({analysis.evaluation/100:.1f})"
            
            # Include best line if requested
            if request.include_best_line and analysis.principal_variation:
                response.best_line = " ".join(analysis.principal_variation)
            
            # Check for game ending conditions
            game_over, result = self.stockfish.is_game_over(request.fen_string)
            response.is_checkmate = game_over and result in ["1-0", "0-1"]
            response.is_stalemate = game_over and result == "1/2-1/2"
            
            response.engine_info = self.stockfish.get_engine_info()
            
        except Exception as e:
            self.get_logger().error(f"EvaluatePosition service error: {e}")
            response.evaluation_text = f"Error: {e}"
        
        return response
    
    def validate_move_callback(self, request, response):
        """Handle ValidateMove service requests"""
        try:
            # Convert ChessMove to UCI
            uci_move = self.converter.chess_move_to_uci(request.proposed_move)
            
            # Validate move
            is_legal, message, resulting_fen = self.stockfish.validate_move(
                request.fen_string, uci_move)
            
            response.is_legal = is_legal
            response.validation_message = message
            
            if resulting_fen:
                response.resulting_fen = resulting_fen
                
                # Check for check/checkmate
                temp_stockfish = StockfishInterface()
                temp_stockfish.initialize()
                temp_stockfish.set_position(resulting_fen)
                
                response.is_check = temp_stockfish.board.is_check()
                response.is_checkmate = temp_stockfish.board.is_checkmate()
                
                # Check if move is capture
                response.is_capture = request.proposed_move.is_capture
                
                temp_stockfish.shutdown()
            
        except Exception as e:
            self.get_logger().error(f"ValidateMove service error: {e}")
            response.is_legal = False
            response.validation_message = f"Error: {e}"
        
        return response
    
    def update_game_state_callback(self, request, response):
        """Handle UpdateGameState service requests"""
        try:
            if request.reset_game:
                # Reset to starting position
                self.current_board_state = None
                self.last_move = None
                self.game_active = True
                response.current_fen = chess.STARTING_FEN
                response.move_number = 1
                response.active_player = "white"
            else:
                # Update current state
                self.current_board_state = request.board_state
                self.last_move = request.last_move if request.last_move.from_square else None
                response.current_fen = request.board_state.fen_string
                response.move_number = request.board_state.fullmove_number
                response.active_player = request.board_state.active_color
            
            # Set game mode
            self.game_mode = request.game_mode
            
            # Check if game is over
            if self.current_board_state:
                game_over, result = self.stockfish.is_game_over(
                    self.current_board_state.fen_string)
                response.game_over = game_over
                response.game_result = result
                
                if game_over:
                    self.game_active = False
            
            response.success = True
            response.status_message = "Game state updated successfully"
            
        except Exception as e:
            self.get_logger().error(f"UpdateGameState service error: {e}")
            response.success = False
            response.status_message = f"Error: {e}"
        
        return response
    
    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.stockfish.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    chess_manager = ChessGameManager()
    
    try:
        rclpy.spin(chess_manager)
    except KeyboardInterrupt:
        pass
    finally:
        chess_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
