#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Chess Engine Server
Standalone ROS2 node providing chess engine services without game management
"""

import rclpy
from rclpy.node import Node
import chess

# ROS2 message imports
from chessmate.msg import BoardState, ChessMove
from chessmate.srv import (
    GetBestMove, EvaluatePosition, ValidateMove, UpdateGameState, CalculateMove
)

# Local imports
from chessmate.engine.stockfish_interface import StockfishInterface, DifficultyLevel, EngineConfig
from chessmate.engine.message_converters import MessageConverter


class ChessEngineServer(Node):
    """
    Chess Engine Server Node
    
    Provides chess engine services without game state management:
    - GetBestMove: Get best move for a position
    - EvaluatePosition: Evaluate a chess position
    - ValidateMove: Check if a move is legal
    - UpdateGameState: Update engine configuration
    """
    
    def __init__(self):
        super().__init__('chess_engine_server')
        
        # Initialize components
        self.stockfish = StockfishInterface()
        self.converter = MessageConverter()
        
        # ROS2 Services
        self.get_best_move_service = self.create_service(
            GetBestMove, 'get_best_move', self.get_best_move_callback)
        
        self.evaluate_position_service = self.create_service(
            EvaluatePosition, 'evaluate_position', self.evaluate_position_callback)
        
        self.validate_move_service = self.create_service(
            ValidateMove, 'validate_move', self.validate_move_callback)
        
        self.update_game_state_service = self.create_service(
            UpdateGameState, 'update_game_state', self.update_game_state_callback)

        self.calculate_move_service = self.create_service(
            CalculateMove, 'engine/calculate_move', self.calculate_move_callback)

        # Initialize engine
        if not self.stockfish.initialize():
            self.get_logger().error("Failed to initialize Stockfish engine")
        else:
            self.get_logger().info("Chess Engine Server initialized successfully")
    
    def get_best_move_callback(self, request, response):
        """Handle GetBestMove service requests"""
        try:
            self.get_logger().debug(f"GetBestMove request: {request.fen_string[:20]}...")
            
            # Configure engine based on difficulty
            if request.difficulty_level:
                try:
                    difficulty = DifficultyLevel(request.difficulty_level)
                    config = EngineConfig.from_difficulty(difficulty)
                    self.stockfish.config = config
                    self.get_logger().info(f"Set difficulty: {request.difficulty_level}")
                except ValueError:
                    self.get_logger().warn(f"Invalid difficulty level: {request.difficulty_level}")
            
            # Get analysis
            analysis = self.stockfish.get_best_move(
                request.fen_string,
                request.time_limit if request.time_limit > 0 else None,
                request.depth_limit if request.depth_limit > 0 else None
            )
            
            if analysis.best_move:
                # Create a minimal board state for conversion
                board_state = BoardState()
                board_state.fen_string = request.fen_string
                board_state = self.converter.fen_to_board_state(request.fen_string)
                
                # Convert to ChessMove
                response.best_move = self.converter.uci_to_chess_move(
                    analysis.best_move, board_state)
                response.best_move.confidence = min(1.0, abs(analysis.evaluation) / 100.0)
                
                self.get_logger().info(f"Best move: {analysis.best_move} (eval: {analysis.evaluation:.1f})")
            else:
                self.get_logger().warn("No best move found")
            
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
            self.get_logger().debug(f"EvaluatePosition request: {request.fen_string[:20]}...")
            
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
            
            self.get_logger().info(f"Position evaluation: {response.evaluation_text}")
            
        except Exception as e:
            self.get_logger().error(f"EvaluatePosition service error: {e}")
            response.evaluation_text = f"Error: {e}"
        
        return response
    
    def validate_move_callback(self, request, response):
        """Handle ValidateMove service requests"""
        try:
            self.get_logger().debug(f"ValidateMove request: {request.proposed_move.from_square}->{request.proposed_move.to_square}")
            
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
                if temp_stockfish.initialize():
                    temp_stockfish.set_position(resulting_fen)
                    
                    response.is_check = temp_stockfish.board.is_check()
                    response.is_checkmate = temp_stockfish.board.is_checkmate()
                    
                    temp_stockfish.shutdown()
                
                # Check if move is capture
                response.is_capture = request.proposed_move.is_capture
            
            self.get_logger().info(f"Move validation: {message}")
            
        except Exception as e:
            self.get_logger().error(f"ValidateMove service error: {e}")
            response.is_legal = False
            response.validation_message = f"Error: {e}"
        
        return response
    
    def update_game_state_callback(self, request, response):
        """Handle UpdateGameState service requests"""
        try:
            self.get_logger().info(f"UpdateGameState request: mode={request.game_mode}")
            
            if request.reset_game:
                # Reset engine to starting position
                self.stockfish.set_position(chess.STARTING_FEN)
                response.current_fen = chess.STARTING_FEN
                response.move_number = 1
                response.active_player = "white"
                response.game_result = "*"
            else:
                # Update with provided state
                response.current_fen = request.board_state.fen_string
                response.move_number = request.board_state.fullmove_number
                response.active_player = request.board_state.active_color
                
                # Check if game is over
                game_over, result = self.stockfish.is_game_over(
                    request.board_state.fen_string)
                response.game_over = game_over
                response.game_result = result
            
            response.success = True
            response.status_message = "Game state updated successfully"
            
            self.get_logger().info(f"Game state updated: {response.status_message}")
            
        except Exception as e:
            self.get_logger().error(f"UpdateGameState service error: {e}")
            response.success = False
            response.status_message = f"Error: {e}"
        
        return response

    def calculate_move_callback(self, request, response):
        """Handle CalculateMove service requests"""
        self.get_logger().info("=== CalculateMove service callback started ===")
        try:
            self.get_logger().info(f"Calculate move request for FEN: {request.fen[:50]}...")

            # Set up the position
            board = chess.Board(request.fen)

            # Configure engine settings
            if hasattr(request, 'skill_level') and request.skill_level > 0:
                # Check if stockfish interface has set_skill_level method
                if hasattr(self.stockfish, 'set_skill_level'):
                    self.stockfish.set_skill_level(request.skill_level)
                else:
                    self.get_logger().debug(f"Skill level {request.skill_level} requested but not supported")

            # Set time limit
            time_limit = getattr(request, 'time_limit', 5.0)

            # Get best move from engine
            analysis_result = self.stockfish.get_best_move(
                fen_string=request.fen,
                time_limit=time_limit
            )

            if analysis_result and analysis_result.best_move:
                try:
                    # Convert UCI move to ChessMove message
                    best_move_uci = analysis_result.best_move
                    move = chess.Move.from_uci(best_move_uci)

                    # Create a minimal ChessMove to test serialization
                    chess_move = ChessMove()
                    chess_move.from_square = str(chess.square_name(move.from_square))
                    chess_move.to_square = str(chess.square_name(move.to_square))
                    chess_move.piece_type = "pawn"  # Simplified for testing
                    chess_move.promotion_piece = ""
                    chess_move.is_capture = False  # Simplified for testing
                    chess_move.move_type = "normal"

                    # Use evaluation from analysis result
                    evaluation = analysis_result.evaluation if analysis_result.evaluation is not None else 0.0

                    # Create a minimal response to test
                    response.success = True
                    response.best_move = chess_move
                    response.evaluation = float(evaluation)  # Ensure it's a float
                    response.message = f"Best move calculated: {best_move_uci}"
                    response.calculation_time = float(time_limit)  # Ensure it's a float

                    self.get_logger().info(f"Calculated best move: {best_move_uci} (eval: {evaluation})")
                    self.get_logger().info(f"Response prepared successfully: {chess_move.from_square} -> {chess_move.to_square}")

                except Exception as move_error:
                    self.get_logger().error(f"Error creating move response: {move_error}")
                    response.success = False
                    response.best_move = ChessMove()
                    response.evaluation = 0.0
                    response.message = f"Move conversion error: {move_error}"
                    response.calculation_time = 0.0

            else:
                response.success = False
                response.best_move = ChessMove()  # Empty move
                response.evaluation = 0.0
                response.message = "No valid move found"
                response.calculation_time = 0.0

        except Exception as e:
            self.get_logger().error(f"CalculateMove service error: {e}")
            response.success = False
            response.best_move = ChessMove()  # Empty move
            response.evaluation = 0.0
            response.message = f"Error: {e}"
            response.calculation_time = 0.0

        # Always log the response being sent
        self.get_logger().info(f"Sending CalculateMove response: success={response.success}")

        # Add a small delay to ensure response is properly sent
        import time
        time.sleep(0.1)

        self.get_logger().info("=== CalculateMove service callback completed ===")
        return response

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        self.stockfish.shutdown()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    engine_server = ChessEngineServer()
    
    try:
        rclpy.spin(engine_server)
    except KeyboardInterrupt:
        pass
    finally:
        engine_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
