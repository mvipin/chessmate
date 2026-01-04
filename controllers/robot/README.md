# ChessMate Robot Controller

This directory contains the Robot Controller implementation for the ChessMate project. The Robot Controller manages the robotic arm that executes computer chess moves.

## Architecture Overview

The Robot Controller is part of the new USB-based communication architecture:

```
Raspberry Pi Host → Robot Controller (USB Serial) → Robotic Arm Hardware
```

### Key Features

- **USB Serial Communication**: Direct communication with Raspberry Pi host via `/dev/ttyACM1`
- **Mock Mode**: Software simulation for testing without hardware
- **Move Execution**: Handles 6-character chess move commands (e.g., `e2pe4p`)
- **State Management**: Tracks robot state (IDLE, MOVING, HOMING, etc.)
- **Error Handling**: Robust error detection and recovery

## Hardware Setup

### Pi Pico Connections

The Robot Controller runs on a Raspberry Pi Pico with the following connections:

#### Stepper Motors
- **X-Axis Stepper**: 
  - Step Pin: GPIO 2
  - Direction Pin: GPIO 5
- **Y-Axis Stepper**:
  - Step Pin: GPIO 3
  - Direction Pin: GPIO 6
- **Z-Axis Stepper**:
  - Step Pin: GPIO 4
  - Direction Pin: GPIO 7
- **Enable Pin**: GPIO 8 (shared by all steppers)

#### Limit Switches
- **X-Axis Limit**: GPIO 9
- **Y-Axis Limit**: GPIO 10
- **Z-Axis Limit**: GPIO 11

#### Gripper Servo
- **Servo Control**: GPIO 13

#### Communication
- **USB Serial**: Built-in USB connection to Raspberry Pi
- **Legacy Serial**: GPIO 12 (RX), GPIO A3 (TX) - for legacy ChessBoard communication (not used in new architecture)

### Wiring Diagram

```
Pi Pico Robot Controller
┌─────────────────────────┐
│ GPIO 2  ──── X Step     │
│ GPIO 3  ──── Y Step     │
│ GPIO 4  ──── Z Step     │
│ GPIO 5  ──── X Dir      │
│ GPIO 6  ──── Y Dir      │
│ GPIO 7  ──── Z Dir      │
│ GPIO 8  ──── Enable     │
│ GPIO 9  ──── X Limit    │
│ GPIO 10 ──── Y Limit    │
│ GPIO 11 ──── Z Limit    │
│ GPIO 13 ──── Servo      │
│ USB     ──── Pi Host    │
└─────────────────────────┘
```

## Command Protocol

### Communication Format

All commands are sent as text strings terminated with newline (`\n`) via USB Serial at 9600 baud.

### Command Types

#### Mode Commands
```
mode:mock    - Enable mock simulation mode
mode:real    - Enable real hardware mode
```

#### System Commands
```
init         - Initialize robot systems
home         - Home robot to origin position
status       - Get current robot status
```

#### Move Commands
```
e2pe4p       - Execute chess move (6-character format)
```

**Move Format**: `<from><piece><to><piece>`
- `from`: Source square (e.g., `e2`)
- `piece`: Piece type at source (`p`=pawn, `r`=rook, `n`=knight, `b`=bishop, `q`=queen, `k`=king)
- `to`: Destination square (e.g., `e4`)
- `piece`: Piece type at destination (same as source, or `x` for capture)

**Examples**:
- `e2pe4p` - Pawn from e2 to e4
- `g1nf3n` - Knight from g1 to f3
- `d1qd8x` - Queen from d1 captures on d8

#### Single Character Commands
```
i            - Wake up robot (start animations)
s            - Sleep robot (idle animations)
j            - Home Z-axis only
z            - Home all axes
```

### Response Format

The Robot Controller sends responses in the format:
```
ROBOT: <message>
```

**Example Responses**:
```
ROBOT: ChessMate Robot Controller initialized
ROBOT: Mock simulation mode enabled
ROBOT: Executing move: e2pe4p
ROBOT: Move completed: e2pe4p
ROBOT: Ready for next command
```

## Mock vs Real Mode

### Mock Mode (Default)

Mock mode simulates robot operations without hardware:

- **Move Simulation**: Simulates 3-8 second move execution
- **Status Reporting**: Provides realistic status updates
- **No Hardware**: Safe for testing without physical robot
- **Timing**: Realistic timing for integration testing

### Real Mode

Real mode controls actual hardware:

- **Hardware Control**: Direct stepper motor and servo control
- **Sensor Feedback**: Uses limit switches for homing
- **Safety**: Includes collision detection and limits
- **Calibration**: Requires proper calibration for accurate moves

## Testing Instructions

### Prerequisites

1. **Hardware**: Pi Pico connected via USB to Raspberry Pi
2. **Software**: Robot Controller code uploaded to Pi Pico
3. **Permissions**: User in `dialout` group for USB access

### Running Tests

#### Individual Robot Tests

```bash
cd Robot
./test_robot.sh                    # Automated tests
./test_robot.sh interactive        # Interactive mode
```

#### Python Test Script

```bash
cd Robot
python3 test_robot_controller.py              # Automated tests
python3 test_robot_controller.py interactive  # Interactive mode
```

#### System Integration Tests

```bash
cd scripts
./test_chessmate.sh                # Main test launcher
python3 test_complete_game.py      # Full game simulation
```

### Test Sequence

1. **Connection Test**: Verify USB communication
2. **Mode Test**: Switch between mock and real modes
3. **System Commands**: Test init, home, status commands
4. **Move Commands**: Test various chess moves
5. **Single Character Commands**: Test legacy command support
6. **Integration Test**: Full game simulation with ChessBoard

### Expected Output

```bash
🤖 Robot Controller USB Test
==================================================
Connecting to Robot controller at /dev/ttyACM1...
✅ Connected to /dev/ttyACM1

🧪 Testing Basic Robot Commands
==================================================
📤 Sending: mode:mock
📥 [12:34:56.789] ROBOT: Mock simulation mode enabled
✅ Command 'mode:mock' sent

📤 Sending: init
📥 [12:34:56.890] ROBOT: Initializing robot systems
✅ Command 'init' sent

🎮 Testing Move Commands
==================================================
🎯 Testing move: e2pe4p
📤 Sending: e2pe4p
📥 [12:34:57.123] ROBOT: Executing move: e2pe4p
⏳ Waiting for move completion...
📥 [12:35:02.456] ROBOT: Move completed: e2pe4p
✅ Move 'e2pe4p' completed
```

## Integration with New Architecture

### Communication Flow

1. **Host Decision**: Raspberry Pi determines computer move
2. **Command Routing**: Host sends move command to Robot Controller
3. **Move Execution**: Robot Controller executes physical move
4. **Completion**: Robot reports move completion to Host
5. **State Update**: Host updates game state

### Differences from Legacy Architecture

**Old**: ChessBoard ↔ Robot (SoftwareSerial)
**New**: Host → Robot (USB Serial)

**Benefits**:
- Direct host control of robot
- Better error handling and recovery
- Simplified communication protocol
- Independent controller testing
- Improved reliability

## Troubleshooting

### Common Issues

#### Robot Controller Not Found
```
❌ Robot controller not found at /dev/ttyACM1
```

**Solutions**:
1. Check USB connection to Pi Pico
2. Verify Pi Pico is powered and running Robot Controller code
3. Check if device appears as different port (`/dev/ttyACM0`, `/dev/ttyACM2`, etc.)
4. Try unplugging and reconnecting USB cable

#### Permission Denied
```
⚠️ Permission issue with /dev/ttyACM1
```

**Solutions**:
1. Add user to dialout group: `sudo usermod -a -G dialout $USER`
2. Log out and log back in
3. Or run with sudo (not recommended for regular use)

#### No Response from Robot
```
❌ No response from Robot controller
```

**Solutions**:
1. Verify correct baud rate (9600)
2. Check if Robot Controller code is running
3. Try resetting Pi Pico (press BOOTSEL + RUN buttons)
4. Re-upload Robot Controller code

#### Move Execution Fails
```
❌ Robot move failed or timed out
```

**Solutions**:
1. Check if robot is in correct mode (mock/real)
2. Verify move format is correct (6 characters)
3. Ensure robot is not busy with previous command
4. Check hardware connections (in real mode)

### Debug Mode

Enable debug output by modifying `RobotController.ino`:

```cpp
#define DEBUG_ENABLED true
```

This provides additional diagnostic information for troubleshooting.

### Hardware Calibration

For real mode operation, the robot requires calibration:

1. **Home Position**: Ensure limit switches trigger correctly
2. **Square Positions**: Calibrate each chess square position
3. **Piece Heights**: Adjust Z-axis positions for different pieces
4. **Gripper**: Calibrate open/close positions

Refer to the legacy `Arm.ino` calibration functions for detailed procedures.

## File Structure

```
Robot/
├── README.md                 # This documentation
├── Robot.ino                 # Main Pi Pico controller (merged USB + hardware)
├── Arm.h                    # Arm hardware definitions
├── Arm.ino                  # Arm hardware implementation
├── Head.h                   # Head hardware definitions
├── Head.ino                 # Head hardware implementation
├── test_robot_controller.py  # Python test script
└── test_robot.sh            # Bash test wrapper
```

## Development Notes

- **Merged Architecture**: `Robot.ino` now includes both USB communication and hardware control
- **Hardware Integration**: Real hardware control uses existing `Arm.ino` and `Head.ino` functions
- **Backward Compatibility**: Maintains compatibility with existing hardware code
- **Testing**: Mock mode allows development and testing without hardware

For system-wide integration testing, see `../scripts/test_complete_game.py`.
