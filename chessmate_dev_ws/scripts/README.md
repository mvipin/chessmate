# ChessMate System Test Scripts

This directory contains system-wide integration test scripts for the ChessMate project.

## Scripts Overview

### `test_chessmate.sh`
Main test launcher script that provides a menu-driven interface for testing different components:

1. **ChessBoard Controller only** - Tests board sensing and human move detection
2. **Robot Controller only** - Tests robotic arm control and move execution  
3. **Complete game simulation** - Full integration test with both controllers
4. **Interactive modes** - Manual testing with real-time command input
5. **All tests (automated)** - Runs complete test suite

### `test_complete_game.py`
Python script for complete game simulation testing:

- **Dual Controller Integration**: Tests both ChessBoard and Robot controllers
- **Game Flow Simulation**: Simulates complete chess game with alternating turns
- **Mock Mode Support**: Works with mock controllers for testing without hardware
- **Error Handling**: Comprehensive error detection and reporting

## Usage

### Quick Start
```bash
cd scripts
./test_chessmate.sh
```

### Direct Script Execution
```bash
cd scripts

# Complete game simulation
python3 test_complete_game.py full

# Individual controller tests  
python3 test_complete_game.py individual

# Game initialization only
python3 test_complete_game.py init
```

## Prerequisites

### Hardware Requirements
- ChessBoard Pi Pico connected via USB (`/dev/ttyACM0`)
- Robot Pi Pico connected via USB (`/dev/ttyACM1`) - optional for ChessBoard-only tests

### Software Requirements
- Python 3 with `serial` module
- User in `dialout` group for USB device access
- Both controllers running appropriate firmware

### Permission Setup
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

## Project Structure Integration

The scripts directory works with the following project structure:

```
ChessMate/
├── ChessBoard/
│   ├── test_chessboard_controller.py    # ChessBoard-specific tests
│   └── test_chessboard.sh        # ChessBoard test wrapper
├── Robot/
│   ├── test_robot_controller.py  # Robot-specific tests  
│   └── test_robot.sh             # Robot test wrapper
└── scripts/
    ├── test_chessmate.sh         # Main system test launcher
    ├── test_complete_game.py     # Full game simulation
    └── README.md                 # This file
```

## Testing Workflow

### Development Testing
1. Test individual controllers first using their respective test scripts
2. Verify basic communication and command processing
3. Run integration tests using scripts in this directory

### Integration Testing  
1. Use `test_chessmate.sh` for comprehensive system testing
2. Start with individual controller tests (options 1-2)
3. Progress to complete game simulation (option 3)
4. Use interactive modes for debugging (options 4-5)

### Automated Testing
```bash
# Run all available tests automatically
./test_chessmate.sh
# Choose option 6 for automated test suite
```

## Error Handling

The scripts include comprehensive error checking:

- **Device Detection**: Verifies USB devices are present
- **Permission Validation**: Checks read/write access to devices
- **Script Validation**: Ensures all required test scripts exist
- **Communication Testing**: Validates controller responses
- **Timeout Handling**: Prevents hanging on unresponsive controllers

## Architecture Support

These scripts support the new USB-based architecture:

**Old Architecture**: ChessBoard ↔ Robot (SoftwareSerial) → Pi Host  
**New Architecture**: Pi Host → ChessBoard (USB) + Pi Host → Robot (USB)

Benefits of new architecture:
- Independent controller testing
- Better error isolation
- Simplified communication protocols
- Direct host control of both controllers
- Improved reliability and debugging

## Troubleshooting

### Common Issues

**Controllers not found**:
- Check USB connections
- Verify device paths (`/dev/ttyACM0`, `/dev/ttyACM1`)
- Try different USB ports

**Permission errors**:
- Add user to dialout group
- Check device permissions with `ls -la /dev/ttyACM*`

**Communication failures**:
- Verify correct baud rate (9600)
- Check controller firmware is running
- Try resetting controllers

**Test failures**:
- Run individual controller tests first
- Check controller-specific README files
- Enable debug mode in controller firmware

For controller-specific troubleshooting, see:
- `../ChessBoard/README.md`
- `../Robot/README.md`
