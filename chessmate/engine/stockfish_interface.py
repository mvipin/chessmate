#!/usr/bin/env python3
# Copyright (c) 2025 Vipin M
# Licensed under the MIT License. See LICENSE file in the project root for full license information.

"""
Stockfish Chess Engine Interface
Provides Python wrapper for Stockfish UCI chess engine
"""

import chess
import chess.engine
import time
from typing import Optional, Dict, Tuple, List
from dataclasses import dataclass
from enum import Enum


class DifficultyLevel(Enum):
    """Chess engine difficulty levels"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate" 
    ADVANCED = "advanced"
    EXPERT = "expert"


@dataclass
class EngineConfig:
    """Stockfish engine configuration"""
    path: str = "/usr/games/stockfish"  # Default Stockfish path
    threads: int = 1
    hash_size: int = 16  # MB
    skill_level: int = 20  # 0-20, 20 = maximum strength
    depth_limit: int = 15
    time_limit: float = 1.0  # seconds
    
    @classmethod
    def from_difficulty(cls, difficulty: DifficultyLevel) -> 'EngineConfig':
        """Create config based on difficulty level"""
        configs = {
            DifficultyLevel.BEGINNER: cls(skill_level=5, depth_limit=8, time_limit=0.5),
            DifficultyLevel.INTERMEDIATE: cls(skill_level=10, depth_limit=12, time_limit=1.0),
            DifficultyLevel.ADVANCED: cls(skill_level=15, depth_limit=15, time_limit=2.0),
            DifficultyLevel.EXPERT: cls(skill_level=20, depth_limit=20, time_limit=5.0)
        }
        return configs.get(difficulty, cls())


@dataclass
class AnalysisResult:
    """Chess engine analysis result"""
    best_move: Optional[str] = None  # UCI format (e.g., "e2e4")
    evaluation: float = 0.0  # Centipawns
    mate_in: Optional[int] = None  # Moves to mate
    principal_variation: List[str] = None  # Best line
    nodes_searched: int = 0
    analysis_time: float = 0.0
    depth_reached: int = 0
    
    def __post_init__(self):
        if self.principal_variation is None:
            self.principal_variation = []


class StockfishInterface:
    """
    Stockfish Chess Engine Interface
    
    Provides high-level interface to Stockfish UCI engine for:
    - Move generation and analysis
    - Position evaluation
    - Move validation
    - Game state management
    """
    
    def __init__(self, config: Optional[EngineConfig] = None):
        """Initialize Stockfish interface"""
        self.config = config or EngineConfig()
        self.engine: Optional[chess.engine.SimpleEngine] = None
        self.board = chess.Board()
        self._is_initialized = False
        
    def initialize(self) -> bool:
        """Initialize the Stockfish engine"""
        try:
            print(f"Initializing Stockfish from path: {self.config.path}")

            # Start Stockfish engine
            self.engine = chess.engine.SimpleEngine.popen_uci(self.config.path)
            print("Stockfish process started successfully")

            # Configure engine settings
            self.engine.configure({
                "Threads": self.config.threads,
                "Hash": self.config.hash_size,
                "Skill Level": self.config.skill_level
            })
            print("Stockfish configuration applied successfully")

            self._is_initialized = True
            return True

        except Exception as e:
            print(f"Failed to initialize Stockfish: {e}")
            print(f"Exception type: {type(e).__name__}")
            if hasattr(e, 'args'):
                print(f"Exception args: {e.args}")
            return False
    
    def shutdown(self):
        """Shutdown the engine"""
        if self.engine:
            self.engine.quit()
            self.engine = None
        self._is_initialized = False
    
    def set_position(self, fen_string: str) -> bool:
        """Set board position from FEN string"""
        try:
            self.board.set_fen(fen_string)
            return True
        except Exception as e:
            print(f"Invalid FEN string: {e}")
            return False
    
    def get_best_move(self, fen_string: str, time_limit: float = None, 
                     depth_limit: int = None) -> AnalysisResult:
        """Get best move for given position"""
        if not self._is_initialized:
            if not self.initialize():
                return AnalysisResult()
        
        # Set position
        if not self.set_position(fen_string):
            return AnalysisResult()
        
        # Use provided limits or defaults
        time_limit = time_limit or self.config.time_limit
        depth_limit = depth_limit or self.config.depth_limit
        
        try:
            start_time = time.time()
            
            # Analyze position
            limit = chess.engine.Limit(time=time_limit, depth=depth_limit)
            result = self.engine.play(self.board, limit)
            
            analysis_time = time.time() - start_time
            
            # Get detailed analysis
            info = self.engine.analyse(self.board, chess.engine.Limit(time=0.1))
            
            # Extract analysis data
            analysis = AnalysisResult()
            analysis.best_move = result.move.uci() if result.move else None
            analysis.analysis_time = analysis_time
            
            # Extract evaluation
            if "score" in info:
                score = info["score"].relative
                if score.is_mate():
                    analysis.mate_in = score.mate()
                    analysis.evaluation = 10000 if score.mate() > 0 else -10000
                else:
                    analysis.evaluation = float(score.score() or 0)
            
            # Extract other info
            analysis.nodes_searched = info.get("nodes", 0)
            analysis.depth_reached = info.get("depth", 0)
            
            # Extract principal variation
            if "pv" in info:
                analysis.principal_variation = [move.uci() for move in info["pv"]]
            
            return analysis
            
        except Exception as e:
            print(f"Analysis failed: {e}")
            return AnalysisResult()
    
    def evaluate_position(self, fen_string: str, analysis_time: float = 0.5) -> AnalysisResult:
        """Evaluate position without necessarily finding best move"""
        if not self._is_initialized:
            if not self.initialize():
                return AnalysisResult()
        
        if not self.set_position(fen_string):
            return AnalysisResult()
        
        try:
            start_time = time.time()
            
            # Analyze position
            info = self.engine.analyse(self.board, chess.engine.Limit(time=analysis_time))
            
            actual_time = time.time() - start_time
            
            analysis = AnalysisResult()
            analysis.analysis_time = actual_time
            
            # Extract evaluation
            if "score" in info:
                score = info["score"].relative
                if score.is_mate():
                    analysis.mate_in = score.mate()
                    analysis.evaluation = 10000 if score.mate() > 0 else -10000
                else:
                    analysis.evaluation = float(score.score() or 0)
            
            # Extract best move from PV
            if "pv" in info and info["pv"]:
                analysis.best_move = info["pv"][0].uci()
                analysis.principal_variation = [move.uci() for move in info["pv"]]
            
            analysis.nodes_searched = info.get("nodes", 0)
            analysis.depth_reached = info.get("depth", 0)
            
            return analysis
            
        except Exception as e:
            print(f"Position evaluation failed: {e}")
            return AnalysisResult()
    
    def validate_move(self, fen_string: str, move_uci: str) -> Tuple[bool, str, Optional[str]]:
        """
        Validate if a move is legal
        
        Returns:
            (is_legal, message, resulting_fen)
        """
        if not self.set_position(fen_string):
            return False, "Invalid FEN string", None
        
        try:
            # Parse move
            move = chess.Move.from_uci(move_uci)
            
            # Check if move is legal
            if move in self.board.legal_moves:
                # Make move to get resulting position
                temp_board = self.board.copy()
                temp_board.push(move)
                resulting_fen = temp_board.fen()
                
                # Check for special conditions
                if temp_board.is_checkmate():
                    return True, "Legal move - results in checkmate", resulting_fen
                elif temp_board.is_check():
                    return True, "Legal move - results in check", resulting_fen
                else:
                    return True, "Legal move", resulting_fen
            else:
                return False, "Illegal move", None
                
        except Exception as e:
            return False, f"Invalid move format: {e}", None
    
    def is_game_over(self, fen_string: str) -> Tuple[bool, str]:
        """Check if game is over and return result"""
        if not self.set_position(fen_string):
            return False, "*"
        
        if self.board.is_checkmate():
            if self.board.turn == chess.WHITE:
                return True, "0-1"  # Black wins
            else:
                return True, "1-0"  # White wins
        elif self.board.is_stalemate():
            return True, "1/2-1/2"  # Draw
        elif self.board.is_insufficient_material():
            return True, "1/2-1/2"  # Draw
        elif self.board.is_seventyfive_moves():
            return True, "1/2-1/2"  # Draw
        elif self.board.is_fivefold_repetition():
            return True, "1/2-1/2"  # Draw
        else:
            return False, "*"  # Game continues
    
    def set_skill_level(self, skill_level: int):
        """Set engine skill level (0-20)"""
        if not self._is_initialized:
            return False

        try:
            # Clamp skill level to valid range
            skill_level = max(0, min(20, skill_level))
            self.config.skill_level = skill_level

            # Update engine configuration
            self.engine.configure({"Skill Level": skill_level})
            return True
        except Exception as e:
            print(f"Failed to set skill level: {e}")
            return False

    def get_engine_info(self) -> str:
        """Get engine information"""
        if not self._is_initialized:
            return "Engine not initialized"

        try:
            # Get engine ID
            return f"Stockfish (Skill: {self.config.skill_level}/20, Threads: {self.config.threads})"
        except:
            return "Stockfish Engine"
    
    def __enter__(self):
        """Context manager entry"""
        self.initialize()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.shutdown()
