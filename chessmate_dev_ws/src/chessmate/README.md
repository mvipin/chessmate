# ChessMate ROS2 Package

This consolidated ROS2 package provides all components for the ChessMate autonomous chess robot system. It integrates chess engine capabilities, hardware interfaces, kinematics, robot description, and message definitions into a unified package.

## Overview

The package provides:
- **Chess Engine**: Stockfish integration for move calculation and position evaluation
- **Hardware Interfaces**: Arduino communication, GPIO abstraction, LCD display, rotary encoder
- **Kinematics**: SCARA robot inverse/forward kinematics and chess coordinate mapping
- **Robot Description**: URDF models, meshes, and visualization configurations
- **Messages/Services**: ROS2 message types, service definitions, and action interfaces

## Package Structure

```
chessmate/
├── chessmate/
│   ├── engine/                    # Chess engine components
│   │   ├── stockfish_interface.py
│   │   ├── chess_engine_server.py
│   │   ├── topic_chess_engine_server.py
│   │   ├── topic_game_management.py
│   │   └── message_converters.py
│   ├── hardware/                  # Hardware interface components
│   │   ├── arduino_communication_node.py
│   │   ├── topic_arduino_communication.py
│   │   ├── game_management_node.py
│   │   ├── gpio_abstraction.py
│   │   ├── lcd_display_node.py
│   │   └── rotary_encoder_node.py
│   ├── kinematics/                # Kinematics components
│   │   ├── scara_kinematics.py
│   │   └── chess_coordinate_mapper.py
│   └── nodes/                     # Additional nodes
├── action/                        # Action definitions
│   └── ExecuteChessMove.action
├── msg/                           # Message definitions
│   ├── BoardState.msg
│   ├── ChessMove.msg
│   ├── GameState.msg
│   ├── RobotStatus.msg
│   ├── LCDCommand.msg
│   ├── RotaryEncoderEvent.msg
│   └── ArduinoCommand.msg
├── srv/                           # Service definitions
│   ├── CalculateMove.srv
│   ├── ExecuteMove.srv
│   ├── ValidateMove.srv
│   ├── ForwardKinematics.srv
│   ├── InverseKinematics.srv
│   └── SetBoardMode.srv
├── config/                        # Configuration files
│   ├── scara_config.yaml
│   └── unified_hardware_config.yaml
├── description/                   # Robot description
│   ├── urdf/
│   ├── meshes/
│   └── rviz/
├── launch/                        # Launch files
├── scripts/                       # Utility scripts
├── test/                          # Test files
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

### System Dependencies
```bash
sudo apt update
sudo apt install stockfish ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 ros-jazzy-xacro
```

### Python Dependencies
```bash
pip3 install python-chess numpy pyserial pillow
```

## Installation

```bash
cd chessmate_dev_ws
colcon build --packages-select chessmate --symlink-install
source install/setup.bash
```

## Quick Start

### Start Complete System (Mock Mode)
```bash
# Terminal 1: Chess Engine
ros2 run chessmate topic_chess_engine_server

# Terminal 2: Arduino Communication
ros2 run chessmate topic_arduino_communication --ros-args -p hardware_mode:=mock

# Terminal 3: Game Management
ros2 run chessmate topic_game_management --ros-args -p hardware_mode:=mock
```

### Start with Launch File
```bash
ros2 launch chessmate unified_hardware.launch.py hardware_mode:=mock
```

### Robot Visualization
```bash
ros2 launch chessmate display.launch.py
```

## Component Documentation

### Chess Engine

The chess engine integrates Stockfish for computer opponent functionality.

#### Key Nodes
- `topic_chess_engine_server`: Handles move calculation requests via topics
- `chess_engine_server`: Service-based engine interface

#### Configuration Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `stockfish_path` | `/usr/games/stockfish` | Path to Stockfish binary |
| `skill_level` | `10` | Engine skill level (1-20) |
| `time_limit` | `5.0` | Thinking time in seconds |
| `hash_size_mb` | `128` | Hash table size in MB |

#### Topics
| Topic | Type | Description |
|-------|------|-------------|
| `engine/request` | `String` | Move calculation requests (JSON) |
| `engine/response` | `String` | Move calculation responses (JSON) |

#### Example Usage
```python
from chessmate.engine.stockfish_interface import StockfishInterface

engine = StockfishInterface(stockfish_path='/usr/games/stockfish')
engine.set_position('rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1')
best_move = engine.get_best_move(time_limit=5.0)
```

---

### Hardware Interfaces

Hardware communication with Arduino controllers, GPIO, LCD display, and rotary encoder.

#### Key Nodes
- `topic_arduino_communication`: Topic-based Arduino bridge for robot and chessboard
- `arduino_communication_node`: Service-based Arduino interface
- `lcd_display_node`: SSD1306 OLED display control
- `rotary_encoder_node`: Menu navigation input

#### Configuration Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `hardware_mode` | `mock` | `mock` or `real` hardware |
| `board_controller_port` | `/dev/ttyACM0` | ChessBoard serial port |
| `robot_controller_port` | `/dev/ttyACM1` | Robot arm serial port |
| `baud_rate` | `9600` | Serial communication baud rate |

#### Topics
| Topic | Type | Description |
|-------|------|-------------|
| `game/human_move` | `String` | Human move input from chessboard |
| `game/state` | `String` | Current game state (JSON) |
| `robot/request` | `String` | Robot move execution requests |
| `robot/response` | `String` | Robot execution responses |
| `chessboard/legal_moves` | `String` | Legal moves for current position |

#### Hardware Abstraction
The package includes GPIO abstraction for cross-platform development:
- **Linux Host**: Uses mock GPIO/serial for development
- **Raspberry Pi**: Uses real GPIO and hardware interfaces

---

### Kinematics

SCARA robot kinematics for chess piece manipulation.

#### Components
- `scara_kinematics.py`: Forward/inverse kinematics for 2-DOF SCARA arm
- `chess_coordinate_mapper.py`: Maps chess squares (a1-h8) to robot coordinates

#### Configuration (`config/scara_config.yaml`)
```yaml
scara:
  link1_length: 0.15    # First arm segment (meters)
  link2_length: 0.15    # Second arm segment (meters)
  base_offset_x: 0.0    # Base X offset from board origin
  base_offset_y: 0.0    # Base Y offset from board origin
  z_safe: 0.05          # Safe Z height for moves
  z_pick: 0.01          # Z height for piece pickup
```

#### Services
| Service | Description |
|---------|-------------|
| `kinematics/forward` | Joint angles → Cartesian position |
| `kinematics/inverse` | Cartesian position → Joint angles |
| `kinematics/chess_square` | Chess square → Robot coordinates |

#### Example Usage
```python
from chessmate.kinematics.scara_kinematics import SCARAKinematics
from chessmate.kinematics.chess_coordinate_mapper import ChessCoordinateMapper

kinematics = SCARAKinematics(link1=0.15, link2=0.15)
mapper = ChessCoordinateMapper(board_origin=(0.1, 0.1), square_size=0.04)

# Get position for square e4
x, y = mapper.square_to_xy('e4')
theta1, theta2 = kinematics.inverse_kinematics(x, y)
```

---

### Robot Description

URDF models and visualization for the SCARA chess robot.

#### Files
- `description/urdf/scara_robot.urdf.xacro`: Main robot model
- `description/urdf/chess_board.urdf.xacro`: Chess board model
- `description/urdf/chessmate_complete.urdf.xacro`: Combined system
- `description/meshes/`: STL mesh files (visual and collision)
- `description/rviz/`: RViz configuration files

#### Launch Visualization
```bash
# Basic robot visualization
ros2 launch chessmate display.launch.py

# With joint state publisher GUI
ros2 launch chessmate display.launch.py use_gui:=true

# Complete system with chess board
ros2 launch chessmate chessmate_complete.launch.py
```

#### Mesh Export
See `description/meshes/README.md` for Fusion 360 STL export instructions.

---

## Message Definitions

### Core Messages

#### ChessMove.msg
```
string from_square      # e.g., "e2"
string to_square        # e.g., "e4"
string piece_type       # "pawn", "rook", "knight", "bishop", "queen", "king"
string move_type        # "normal", "capture", "castling", "en_passant", "promotion"
bool is_capture
bool is_check
string uci_notation     # e.g., "e2e4"
```

#### BoardState.msg
```
string fen_position
uint8[64] square_occupancy
uint32 move_number
bool white_to_move
```

#### GameState.msg
```
BoardState board_state
string game_status      # "active", "check", "checkmate", "stalemate", "draw"
string current_player   # "human", "computer"
ChessMove[] move_history
```

### Service Definitions

#### CalculateMove.srv
```
# Request
string board_fen
float64 time_limit
uint8 skill_level
---
# Response
bool success
string best_move
float64 evaluation
```

#### ExecuteMove.srv
```
# Request
ChessMove move
float64 timeout
---
# Response
bool success
string message
float64 execution_time
```

---

## Testing

### Run Unit Tests
```bash
cd chessmate_dev_ws
python3 -m pytest src/chessmate/test/
```

### Integration Tests
```bash
# Full game simulation test
./test_step6_complete.sh

# Individual component tests
python3 test_step6_full_game.py
```

### Hardware Tests (Mock Mode)
```bash
ros2 launch chessmate unified_hardware.launch.py hardware_mode:=mock log_level:=DEBUG
```

---

## Troubleshooting

### Common Issues

**Stockfish Not Found**
```bash
sudo apt install stockfish
# Or specify custom path
ros2 run chessmate topic_chess_engine_server --ros-args -p stockfish_path:=/path/to/stockfish
```

**Serial Port Access Denied**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

**ROS2 Service Timeouts**
Use topic-based nodes (`topic_*`) instead of service-based nodes for better reliability.

**GPIO Permission Errors (Raspberry Pi)**
```bash
sudo usermod -a -G gpio $USER
```

### Debug Mode
```bash
ros2 run chessmate topic_game_management --ros-args --log-level DEBUG
```

---

## License

This package is part of the ChessMate project.

