# ChessMate ChessBoard Pi Pico Controller

This Pi Pico sketch controls the chessboard hardware for the ChessMate chess robot system. It handles board sensing, LED matrix display, user interface components, and includes comprehensive mock mode for hardware-independent testing.

This Arduino sketch controls the chessboard hardware for the ChessMate chess robot system. It handles board sensing, LED matrix display, and user interface components.

## Features

- **Board Sensing**: Hall effect sensors detect piece positions
- **LED Matrix Display**: 8x8 RGB LED matrix for move highlighting and game status
- **User Interface**: Status LEDs and button controls
- **Serial Communication**: Communicates with Raspberry Pi via Serial2
- **Game State Management**: Tracks move states and validates player moves

## Hardware Requirements

- **Raspberry Pi Pico** (RP2040 microcontroller)
- 8x8 RGB LED matrix (WS2812B/NeoPixel)
- Hall effect sensors for each square (64 total)
- Status LEDs for human/computer turn indicators
- Buttons for move confirmation and hints
- Serial connection to Raspberry Pi (GPIO 0/1)

## Mock Mode Support

The controller includes comprehensive mock mode functionality that simulates all user interactions:
- **Runtime Mode Selection**: Switch between real/mock modes via protocol
- **Intelligent Move Selection**: Randomly selects from legal moves
- **Realistic Timing**: 2-8 second move simulation
- **Hint Requests**: Occasional hint requests (20% probability)
- **Board State Sync**: Maintains accurate occupancy tracking

## Communication Protocol

### Commands from Raspberry Pi (via Serial1):

**Mode Selection:**
- `mode:real` - Enable real hardware mode
- `mode:mock` - Enable mock simulation mode

**Game Commands:**

- `init` - Initialize board for new game
- `occupancy:pos1:pos2:...` - Set initial piece positions
- `legal:move1:move2:...` - Set legal moves for current position
- `hint:move` - Set hint move to display
- `check:square1:square2:...` - Highlight check squares
- `start` - Begin human turn
- `comp:move` - Execute computer move
- `override:move` - Override/undo move
- `checkmate:king_pos:move` - Display checkmate
- `reset` - Reset board state

### Responses to Raspberry Pi:

- `move` - 4-character move notation (e.g., "e2e4")
- `ffff` - Hint override signal
- Board state updates and confirmations

## Architecture Changes

**Note**: This version has been updated to remove direct robotic arm communication. The ChessBoard Arduino now focuses solely on:

1. **Board sensing and validation**
2. **LED display and user interface**
3. **Communication with Raspberry Pi**

The Raspberry Pi now handles all robotic arm coordination, making the system more modular and reliable.

## Pin Configuration

- **Serial1**: Communication with Raspberry Pi (GPIO 0/1)
- **USB Serial**: Debug output (Serial monitor)
- **LED Matrix**: Data pin for WS2812B strip
- **Sensors**: Hall effect sensors on GPIO pins
- **Status LEDs**: Human/computer turn indicators
- **Buttons**: Confirm and hint buttons

## Build and Upload

### Prerequisites

```bash
# Install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Install Pi Pico board support
arduino-cli core install rp2040:rp2040
```

### Compilation

```bash
# Navigate to ChessBoard directory
cd ChessMate/ChessBoard

# Compile for Pi Pico
arduino-cli compile --fqbn rp2040:rp2040:rpipico --output-dir ./build ChessBoard
```

### Upload to Pi Pico

#### Method 1: UF2 Bootloader (Recommended)

```bash
# 1. Put Pi Pico in bootloader mode (hold BOOTSEL while connecting USB)

# 2. Mount the Pi Pico drive
sudo mkdir -p /media/pi/RPI-RP2
sudo mount /dev/sda1 /media/pi/RPI-RP2

# 3. Copy firmware
sudo cp build/ChessBoard.ino.uf2 /media/pi/RPI-RP2/

# Pi Pico will automatically reboot and run the firmware
```

#### Method 2: Direct Upload

```bash
arduino-cli upload --fqbn rp2040:rp2040:rpipico --port /dev/ttyACM0 ChessBoard
```

## Hardware Connections

### Pi Pico to Raspberry Pi Serial Connection

```
Raspberry Pi    <-->    Pi Pico
GPIO 14 (TX)    <-->    GPIO 1 (RX) - Serial1 RX
GPIO 15 (RX)    <-->    GPIO 0 (TX) - Serial1 TX
GND             <-->    GND
```

### Serial Ports

- **Pi Pico Serial1**: GPIO 0/1 for Pi communication
- **Pi Pico USB**: Serial (USB) for debug output
- **Raspberry Pi**: /dev/serial0 or /dev/ttyAMA0 for hardware serial

## Testing

### Mock Mode Testing

```bash
# 1. Connect Pi Pico via USB for debug output
# 2. Connect Serial1 to Raspberry Pi
# 3. Enable mock mode
echo "mode:mock" > /dev/ttyAMA0

# 4. Initialize and start game
echo "init" > /dev/ttyAMA0
echo "occupancy:0:1:2:3:4:5:6:7:48:49:50:51:52:53:54:55" > /dev/ttyAMA0
echo "legal:e2e4:d2d4:Nf3:Nc3" > /dev/ttyAMA0
echo "start" > /dev/ttyAMA0

# Mock mode will automatically simulate moves
```

### Serial Port Monitoring

```bash
# Monitor debug output (USB connection)
screen /dev/ttyACM0 9600

# Monitor Pi communication (Serial1)
screen /dev/ttyAMA0 9600
```

## Integration

This ChessBoard controller integrates with the ChessMate ROS 2 system running on Raspberry Pi. The Pi coordinates between:

- ChessBoard Arduino (this controller)
- Robotic Arm Arduino (separate controller)
- Chess engine (Stockfish)
- User interface (LCD, rotary encoder)

This separation of concerns improves system reliability and makes debugging easier.
