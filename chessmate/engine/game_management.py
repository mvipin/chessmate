#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Game Management Node

ROS2 node that orchestrates the complete chess game flow using topic-based
communication. Manages turn-taking, move validation, and game state.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
import chess
import random
from chessmate.srv import CalculateMove, ExecuteMove, SetBoardMode
from chessmate.msg import GameState, ChessMove
from chessmate.hardware.engine_client import EngineClient
from chessmate.hardware.arduino_clients import RobotClient, BoardClient


class GameManagement(Node):
    def __init__(self):
        super().__init__('game_management')
        
        # Parameters
        self.hardware_mode = self.declare_parameter('hardware_mode', 'mock').value
        self.auto_start = self.declare_parameter('auto_start', True).value
        self.skill_level = self.declare_parameter('skill_level', 5).value
        self.time_limit = self.declare_parameter('time_limit', 3.0).value
        
        # Game state
        self.chess_board = chess.Board()
        self.game_active = False
        self.current_player = 'human'  # 'human' or 'computer'
        self.move_count = 0
        self.game_history = []
        
        # Service clients for communication with other nodes
        self.engine_client = EngineClient(self, 'engine/calculate_move')
        self.robot_client = RobotClient(self, 'robot/execute_move')
        self.board_client = BoardClient(self, 'chessboard/set_mode')
        
        # Game state publisher
        self.game_state_publisher = self.create_publisher(GameState, 'game/state', 10)
        
        # Game control subscribers
        self.game_control_subscriber = self.create_subscription(
            String, 'game/control', self.handle_game_control, 10)
        
        # Human move input subscriber
        self.human_move_subscriber = self.create_subscription(
            String, 'game/human_move', self.handle_human_move, 10)
        
        self.get_logger().info("🎮 Game Management initialized")
        self.get_logger().info(f"Hardware mode: {self.hardware_mode}")
        self.get_logger().info(f"Skill level: {self.skill_level}")
        
        # Initialize game system
        self.initialize_game_system()
    
    def initialize_game_system(self):
        """Initialize the complete game system"""
        self.get_logger().info("🔧 Initializing game system...")
        
        def init_thread():
            time.sleep(2.0)  # Brief delay for services to be ready
            
            try:
                # Wait for all services
                self.get_logger().info("⏳ Waiting for services...")
                
                if not self.engine_client.wait_for_service(timeout_sec=10.0):
                    self.get_logger().error("❌ Engine service not available")
                    return
                self.get_logger().info("✅ Engine service ready")
                
                if not self.robot_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error("❌ Robot service not available")
                    return
                self.get_logger().info("✅ Robot service ready")
                
                if not self.board_client.wait_for_service(timeout_sec=5.0):
                    self.get_logger().error("❌ Board service not available")
                    return
                self.get_logger().info("✅ Board service ready")
                
                # Initialize board mode
                self.get_logger().info("🔧 Setting board to game mode...")
                board_request = SetBoardMode.Request()
                board_request.mode = SetBoardMode.Request.MODE_PLAYING
                board_request.force_mode_change = True
                board_request.additional_params = "New game initialization"
                
                board_response = self.board_client.call(board_request)
                if board_response and board_response.success:
                    self.get_logger().info("✅ Board initialized for game")
                else:
                    self.get_logger().warn("⚠️  Board initialization failed, continuing...")
                
                # Start game if auto_start enabled
                if self.auto_start:
                    self.start_new_game()
                else:
                    self.get_logger().info("🎮 Game system ready - waiting for start command")
                    
            except Exception as e:
                self.get_logger().error(f"❌ Game system initialization failed: {e}")
        
        thread = threading.Thread(target=init_thread)
        thread.daemon = True
        thread.start()
    
    def handle_game_control(self, msg):
        """Handle game control commands"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            
            if command == 'start':
                self.start_new_game()
            elif command == 'stop':
                self.stop_game()
            elif command == 'reset':
                self.reset_game()
            elif command == 'status':
                self.publish_game_state()
            else:
                self.get_logger().warn(f"Unknown game command: {command}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Error processing game control: {e}")
    
    def handle_human_move(self, msg):
        """Handle human move input - should come from controller picking from legal moves"""
        if not self.game_active or self.current_player != 'human':
            self.get_logger().warn("⚠️  Human move received but not human's turn")
            return

        try:
            move_data = json.loads(msg.data)
            move_uci = move_data.get('move', '')

            # Ignore trigger messages from test scripts - these are used to signal
            # the mock chessboard to pick a move, not actual moves
            if move_uci == 'trigger_legal_move':
                return

            self.get_logger().info(f"👤 Human move received: {move_uci}")

            # Process the move directly (it should come from chessboard controller)
            self.process_human_move(move_uci)

        except Exception as e:
            self.get_logger().error(f"❌ Error processing human move: {e}")

    def send_legal_moves_for_human_turn(self):
        """Send legal moves to chessboard controller and start human turn"""
        try:
            # Get legal moves from current board state
            legal_moves = list(self.chess_board.legal_moves)

            if not legal_moves:
                self.get_logger().error("❌ No legal moves available!")
                return

            # Convert to UCI format
            legal_moves_uci = [move.uci() for move in legal_moves]

            self.get_logger().info(f"📋 Sending {len(legal_moves_uci)} legal moves to controller")
            self.get_logger().info(f"📋 Legal moves: {legal_moves_uci[:5]}...")  # Show first 5

            # Send legal moves to chessboard controller via Arduino communication
            self.send_legal_moves_to_chessboard(legal_moves_uci)

            # Now wait for the chessboard controller to send back a move via /game/human_move
            self.get_logger().info("⏳ Waiting for chessboard controller to send human move...")

        except Exception as e:
            self.get_logger().error(f"❌ Error in sending legal moves for human turn: {e}")

    def send_legal_moves_to_chessboard(self, legal_moves_uci):
        """Send legal moves to chessboard controller via Arduino communication"""
        try:
            # Send legal moves via serial command to chessboard controller
            # This needs to be implemented by sending a command to the Arduino communication node
            # For now, we'll create a publisher to send the legal moves
            if not hasattr(self, 'legal_moves_publisher'):
                self.legal_moves_publisher = self.create_publisher(String, 'chessboard/legal_moves', 10)

            legal_moves_data = {
                'command': 'legal_moves',
                'moves': legal_moves_uci,
                'timestamp': time.time()
            }

            msg = String()
            msg.data = json.dumps(legal_moves_data)
            self.legal_moves_publisher.publish(msg)

            self.get_logger().info(f"📤 Legal moves sent to chessboard controller")

        except Exception as e:
            self.get_logger().error(f"❌ Error sending legal moves to chessboard: {e}")

    def start_new_game(self):
        """Start a new chess game"""
        self.get_logger().info("🎮 Starting new chess game...")

        # Reset game state
        self.chess_board = chess.Board()
        self.game_active = True
        self.current_player = 'human'  # Human plays white, goes first
        self.move_count = 0
        self.game_history = []

        self.publish_game_state()

        self.get_logger().info("✅ New game started - waiting for human move")
        self.get_logger().info(f"Current position: {self.chess_board.fen()}")

        # Immediately send legal moves to chessboard controller for human's first turn
        self.send_legal_moves_for_human_turn()

    def stop_game(self):
        """Stop the current game"""
        self.get_logger().info("🛑 Stopping current game...")
        self.game_active = False
        self.publish_game_state()
    
    def reset_game(self):
        """Reset and start a new game"""
        self.stop_game()
        time.sleep(0.5)
        self.start_new_game()
    
    def process_human_move(self, move_uci):
        """Process a human move"""
        try:
            # Parse and validate move
            human_move = chess.Move.from_uci(move_uci)
            
            if human_move not in self.chess_board.legal_moves:
                self.get_logger().error(f"❌ Illegal human move: {move_uci}")
                return
            
            # Execute human move
            self.chess_board.push(human_move)
            self.move_count += 1
            
            # Record move
            self.game_history.append({
                'move_number': self.move_count,
                'player': 'human',
                'move': move_uci,
                'fen': self.chess_board.fen()
            })
            
            self.get_logger().info(f"✅ Human played: {move_uci}")
            
            # Check game end conditions
            if self.chess_board.is_game_over():
                self.handle_game_over()
                return
            
            # Switch to computer turn
            self.current_player = 'computer'
            self.publish_game_state()
            
            # Process computer move
            self.process_computer_move()
            
        except Exception as e:
            self.get_logger().error(f"❌ Error processing human move: {e}")
    
    def process_computer_move(self):
        """Process computer move using engine and robot"""
        self.get_logger().info("🧠 Processing computer move...")
        
        def computer_move_thread():
            try:
                # Get move from engine
                engine_request = CalculateMove.Request()
                engine_request.fen = self.chess_board.fen()
                engine_request.time_limit = self.time_limit
                engine_request.skill_level = self.skill_level
                engine_request.white_to_move = self.chess_board.turn
                
                self.get_logger().info("📤 Requesting move from engine...")
                engine_response = self.engine_client.call(engine_request)
                
                if not (engine_response and engine_response.success):
                    self.get_logger().error("❌ Engine calculation failed")
                    return
                
                # Handle promotion moves correctly
                move_uci = f"{engine_response.best_move.from_square}{engine_response.best_move.to_square}"
                if engine_response.best_move.promotion_piece and engine_response.best_move.promotion_piece != '':
                    # Add promotion piece to UCI (q, r, b, n)
                    promotion_map = {'queen': 'q', 'rook': 'r', 'bishop': 'b', 'knight': 'n'}
                    promotion_char = promotion_map.get(engine_response.best_move.promotion_piece.lower(), 'q')
                    move_uci += promotion_char

                computer_move = chess.Move.from_uci(move_uci)

                # Debug board state and legal moves
                self.get_logger().info(f"🔍 Current board FEN: {self.chess_board.fen()}")
                legal_moves_uci = [move.uci() for move in self.chess_board.legal_moves]
                self.get_logger().info(f"🔍 Legal moves: {legal_moves_uci[:10]}...")  # Show first 10

                # Validate move - this should ALWAYS be legal since Stockfish generated it
                if computer_move not in self.chess_board.legal_moves:
                    self.get_logger().error(f"❌ CRITICAL: Stockfish returned illegal move: {move_uci}")
                    self.get_logger().error(f"❌ This indicates a serious board state sync issue!")
                    self.get_logger().error(f"❌ Engine FEN: {engine_request.fen}")
                    self.get_logger().error(f"❌ Board FEN: {self.chess_board.fen()}")
                    self.get_logger().error(f"❌ Engine response: {engine_response}")

                    # Try to recover by using a random legal move
                    if legal_moves_uci:
                        fallback_move = random.choice(list(self.chess_board.legal_moves))
                        self.get_logger().warn(f"🔄 Using fallback legal move: {fallback_move.uci()}")
                        computer_move = fallback_move
                        move_uci = fallback_move.uci()
                    else:
                        self.get_logger().error(f"❌ No legal moves available - game should be over!")
                        return
                
                self.get_logger().info(f"✅ Engine calculated: {move_uci} (eval: {engine_response.evaluation})")
                
                # Execute move via robot
                self.get_logger().info("🤖 Executing computer move...")
                
                robot_request = ExecuteMove.Request()
                robot_request.move.from_square = engine_response.best_move.from_square
                robot_request.move.to_square = engine_response.best_move.to_square
                robot_request.move.piece_type = engine_response.best_move.piece_type
                robot_request.move.is_capture = engine_response.best_move.is_capture
                
                robot_response = self.robot_client.call(robot_request)
                
                if robot_response and robot_response.success:
                    self.get_logger().info("✅ Robot move executed successfully")
                    
                    # Update game state
                    self.chess_board.push(computer_move)
                    self.move_count += 1
                    
                    # Record move
                    self.game_history.append({
                        'move_number': self.move_count,
                        'player': 'computer',
                        'move': move_uci,
                        'evaluation': engine_response.evaluation,
                        'fen': self.chess_board.fen()
                    })
                    
                    self.get_logger().info(f"✅ Computer played: {move_uci}")
                    
                    # Check game end conditions
                    if self.chess_board.is_game_over():
                        self.handle_game_over()
                        return
                    
                    # Switch to human turn
                    self.current_player = 'human'
                    self.publish_game_state()

                    self.get_logger().info("👤 Waiting for human move...")

                    # Send legal moves for the next human turn
                    self.send_legal_moves_for_human_turn()

                else:
                    self.get_logger().error("❌ Robot move execution failed")
                    
            except Exception as e:
                self.get_logger().error(f"❌ Computer move processing failed: {e}")
        
        thread = threading.Thread(target=computer_move_thread)
        thread.daemon = True
        thread.start()
    
    def handle_game_over(self):
        """Handle game over conditions"""
        self.game_active = False
        
        result = "Unknown"
        if self.chess_board.is_checkmate():
            winner = "Human" if self.chess_board.turn == chess.BLACK else "Computer"
            result = f"Checkmate - {winner} wins!"
        elif self.chess_board.is_stalemate():
            result = "Stalemate - Draw"
        elif self.chess_board.is_insufficient_material():
            result = "Insufficient material - Draw"
        elif self.chess_board.is_fivefold_repetition():
            result = "Fivefold repetition - Draw"
        
        self.get_logger().info(f"🏁 Game Over: {result}")
        self.get_logger().info(f"Total moves: {self.move_count}")
        
        self.publish_game_state()
    
    def publish_game_state(self):
        """Publish current game state"""
        try:
            game_state = GameState()
            game_state.fen = self.chess_board.fen()
            game_state.status = GameState.STATUS_PLAYING if self.game_active else GameState.STATUS_WAITING
            game_state.white_to_move = self.chess_board.turn
            game_state.move_number = self.chess_board.fullmove_number
            game_state.halfmove_clock = self.chess_board.halfmove_clock

            # Set result if game is over
            if self.chess_board.is_game_over():
                game_state.status = GameState.STATUS_FINISHED
                if self.chess_board.is_checkmate():
                    game_state.result = GameState.RESULT_WHITE_WINS if not self.chess_board.turn else GameState.RESULT_BLACK_WINS
                else:
                    game_state.result = GameState.RESULT_DRAW
            else:
                game_state.result = GameState.RESULT_NONE

            # Set player names
            game_state.white_player = "Human"
            game_state.black_player = "ChessMate"
            game_state.event_name = "ChessMate Game"

            # Time control (not implemented yet)
            game_state.time_control_enabled = False
            game_state.white_time_remaining = 0.0
            game_state.black_time_remaining = 0.0

            self.game_state_publisher.publish(game_state)
            
        except Exception as e:
            self.get_logger().error(f"❌ Error publishing game state: {e}")

def main():
    rclpy.init()
    
    try:
        game_manager = GameManagement()
        rclpy.spin(game_manager)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            game_manager.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()
