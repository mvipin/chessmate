// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include "Arm.h"
#include "Head.h"

animation_id_t animation_id;

// New USB communication variables
bool mock_mode = true;  // Default to mock mode for testing
String pending_command = "";
unsigned long mock_move_start_time = 0;
unsigned long mock_move_duration = 0;
bool mock_move_in_progress = false;

// Robot state enumeration
typedef enum {
  ROBOT_IDLE,
  ROBOT_INITIALIZING,
  ROBOT_HOMING,
  ROBOT_MOVING,
  ROBOT_ERROR
} robot_state_t;

robot_state_t robot_state = ROBOT_IDLE;

void setup()
{
  Serial.begin(9600);

  // Wait for USB Serial connection
  for (uint16_t trial = 0; trial < 2000; trial++) {
    if (Serial) {
      break;
    }
  }

  head_init();
  arm_init();

  Serial.println("ROBOT: ChessMate Robot Controller initialized");
  Serial.print("ROBOT: Default mode is ");
  Serial.println(mock_mode ? "MOCK" : "REAL");
  Serial.println("ROBOT: Ready for commands");
}

void loop() {
  // Process USB commands first
  scan_usb_serial();

  // Process mock mode simulation
  if (mock_mode) {
    process_mock_mode();
  }

  // Original functionality
  arm_run();
  animate();
}

// USB Serial Communication Functions
void scan_usb_serial() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      process_usb_command(input);
    }
  }
}

void process_usb_command(String command) {
  Serial.print("ROBOT: Received command: ");
  Serial.println(command);

  // Parse command type
  if (command.startsWith("mode:")) {
    handle_mode_command(command);
  } else if (command == "init") {
    handle_init_command();
  } else if (command == "home") {
    handle_home_command();
  } else if (command == "status") {
    handle_status_command();
  } else if (command.length() == 6 && is_valid_move(command)) {
    handle_move_command(command);
  } else if (command.length() == 1) {
    handle_single_char_command(command.charAt(0));
  } else {
    Serial.print("ROBOT: Unknown command: ");
    Serial.println(command);
  }
}

void handle_mode_command(String command) {
  String mode = command.substring(5);  // Remove "mode:"

  if (mode == "mock") {
    mock_mode = true;
    Serial.println("ROBOT: Mock simulation mode enabled");
  } else if (mode == "real") {
    mock_mode = false;
    Serial.println("ROBOT: Real hardware mode enabled");
  } else {
    Serial.print("ROBOT: Unknown mode: ");
    Serial.println(mode);
    return;
  }

  Serial.print("ROBOT: Current mode is ");
  Serial.println(mock_mode ? "MOCK" : "REAL");
}

void handle_init_command() {
  robot_state = ROBOT_INITIALIZING;
  Serial.println("ROBOT: Initializing robot systems");

  if (mock_mode) {
    // Mock initialization
    Serial.println("ROBOT: Mock initialization complete");
    robot_state = ROBOT_IDLE;
  } else {
    // Real hardware initialization (already done in setup)
    Serial.println("ROBOT: Real hardware initialization complete");
    robot_state = ROBOT_IDLE;
  }
}

void handle_home_command() {
  robot_state = ROBOT_HOMING;
  Serial.println("ROBOT: Homing robot to origin position");

  if (mock_mode) {
    // Mock homing
    Serial.println("ROBOT: Mock homing complete");
    robot_state = ROBOT_IDLE;
  } else {
    // Real hardware homing
    home_all();
    curl_up();
    Serial.println("ROBOT: Real hardware homing complete");
    robot_state = ROBOT_IDLE;
  }
}

void handle_status_command() {
  Serial.print("ROBOT: Status - State: ");
  Serial.print(get_state_name(robot_state));
  Serial.print(", Mode: ");
  Serial.print(mock_mode ? "MOCK" : "REAL");

  if (mock_move_in_progress) {
    unsigned long elapsed = millis() - mock_move_start_time;
    Serial.print(", Move Progress: ");
    Serial.print((elapsed * 100) / mock_move_duration);
    Serial.print("%");
  }

  Serial.println();
}

void handle_move_command(String move) {
  if (robot_state != ROBOT_IDLE) {
    Serial.print("ROBOT: Cannot execute move, robot busy (state: ");
    Serial.print(get_state_name(robot_state));
    Serial.println(")");
    return;
  }

  pending_command = move;
  robot_state = ROBOT_MOVING;

  Serial.print("ROBOT: Executing move: ");
  Serial.println(move);

  if (mock_mode) {
    start_mock_move(move);
  } else {
    // Execute real hardware move using existing execute_move function
    execute_move(move);
    robot_state = ROBOT_IDLE;
    Serial.print("ROBOT: Move completed: ");
    Serial.println(move);
    Serial.println("ROBOT: Ready for next command");
  }
}

void handle_single_char_command(char cmd) {
  switch (cmd) {
    case 'i':
      Serial.println("ROBOT: Wake up command received");
      animation_id = WAKE_UP;
      break;
    case 's':
      Serial.println("ROBOT: Sleep command received");
      animation_id = DOZE_OFF;
      break;
    case 'j':
      handle_home_command();
      break;
    case 'z':
      handle_home_command();
      break;
    default:
      Serial.print("ROBOT: Unknown single character command: ");
      Serial.println(cmd);
      break;
  }
}

// Mock mode functions
void process_mock_mode() {
  if (!mock_mode || !mock_move_in_progress) {
    return;
  }

  unsigned long elapsed = millis() - mock_move_start_time;

  if (elapsed >= mock_move_duration) {
    // Mock move completed
    mock_move_in_progress = false;
    robot_state = ROBOT_IDLE;

    Serial.print("ROBOT: Move completed: ");
    Serial.println(pending_command);
    Serial.println("ROBOT: Ready for next command");

    pending_command = "";
  }
}

void start_mock_move(String move) {
  mock_move_start_time = millis();
  mock_move_duration = random(3000, 8000);  // 3-8 seconds
  mock_move_in_progress = true;

  Serial.print("ROBOT: Starting mock move, estimated duration: ");
  Serial.print(mock_move_duration);
  Serial.println("ms");
}

// Validation functions
bool is_valid_move(String move) {
  // Basic validation for 6-character move format: e2pe4p
  if (move.length() != 6) return false;

  // Check source square (chars 0-1: "e2")
  if (!is_valid_square(move.substring(0, 2))) return false;

  // Check destination square (chars 3-4: "e4")
  if (!is_valid_square(move.substring(3, 5))) return false;

  // Check piece characters (char 2: source piece, char 5: dest piece)
  if (!is_valid_piece(move.charAt(2))) return false;
  if (!is_valid_piece(move.charAt(5)) && move.charAt(5) != 'x') return false;

  return true;
}

bool is_valid_square(String square) {
  if (square.length() != 2) return false;

  char file = square.charAt(0);
  char rank = square.charAt(1);

  return (file >= 'a' && file <= 'h') && (rank >= '1' && rank <= '8');
}

bool is_valid_piece(char piece) {
  return (piece == 'p' || piece == 'r' || piece == 'n' ||
          piece == 'b' || piece == 'q' || piece == 'k');
}

// Utility function
const char* get_state_name(robot_state_t state) {
  switch (state) {
    case ROBOT_IDLE: return "IDLE";
    case ROBOT_INITIALIZING: return "INITIALIZING";
    case ROBOT_HOMING: return "HOMING";
    case ROBOT_MOVING: return "MOVING";
    case ROBOT_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}
