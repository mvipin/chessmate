// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#ifndef UTILS_H
#define UTILS_H

typedef enum {
  MOVE_NONE,
  MOVE_INIT,
  MOVE_RESET,
  MOVE_START,
  MOVE_STOP,
  MOVE_COMP,
  MOVE_OVERRIDE,
  MOVE_CHECKMATE,
} move_state_t;

typedef enum {
  BOARD_STATE_ERROR,
  BOARD_STATE_NONE_MOVED,
  BOARD_STATE_PIECE_MOVED,
  BOARD_STATE_PIECE_REMOVED,
} board_state_t;

enum {
  MOVE_TYPE_COMP,
  MOVE_TYPE_HINT,
  MOVE_TYPE_OVERRIDE,
  MOVE_TYPE_CHECKMATE,
  MOVE_TYPE_MAX,
};

enum {
  BOARD_1,
  BOARD_2,
  BOARD_3,
  BOARD_4,
  NUM_BOARDS,
};

#define COMPUTER 1
#define HUMAN 0

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define ORANGE   0xFC00
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF 

#define CHESS_ROWS 8
#define CHESS_COLS 8
#define LEGAL_MOVES_MAX 64
#define CHECK_SQUARES_MAX 4
#define CMD_LEN_MAX 128
#define HINT_OVERRIDE_CNT 3
#define HINT_OVERRIDE_INTERVAL 3000
#define CONFIRM_ZRESET_CNT 3
#define CONFIRM_ZRESET_INTERVAL 3000

// Mock mode configuration - determined at runtime via protocol
#define MOCK_MODE_DEFAULT true  // Default mode if not specified by host
#define MOCK_MOVE_MIN_TIME 2000  // Minimum time to make a move (ms)
#define MOCK_MOVE_MAX_TIME 8000  // Maximum time to make a move (ms)
#define MOCK_HINT_PROBABILITY 20 // 20% chance to request hint
#define MOCK_THINK_TIME 1000     // Time to "think" before move (ms)

// Debug configuration
#define DEBUG_ENABLED false      // Set to false to disable debug output for host communication

// Debug print function
void debug_print(const char* message) {
  if (DEBUG_ENABLED) {
    Serial.print("[DEBUG] ");
    Serial.println(message);
    Serial.flush();
  }
}

void debug_print(const String& message) {
  if (DEBUG_ENABLED) {
    Serial.print("[DEBUG] ");
    Serial.println(message);
    Serial.flush();
  }
}



void get_algebraic_notation(int row, int col, char *notation) {
  if (row >= 0 && row < CHESS_ROWS && col >= 0 && col < CHESS_COLS) {
    notation[0] = 'a' + col; // Columns map to letters
    notation[1] = '1' + row; // Rows map to numbers
    notation[2] = '\0'; // Null-terminate the string
  }
}

void xy_lookup(const char *notation, uint8_t &row, uint8_t &col) {
  if (notation[0] >= 'a' && notation[0] <= 'h' && notation[1] >= '1' && notation[1] <= '8') {
    col = notation[0] - 'a'; // Convert letter to column
    row = notation[1] - '1'; // Convert number to row
  }
}

void print_matrix(uint8_t matrix[CHESS_ROWS][CHESS_COLS]) {
  for (int i=0; i<CHESS_ROWS; i++) {
    for (int j=0; j<CHESS_COLS; j++) {
      Serial.print(matrix[i][j], HEX);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void print_matrix(bool matrix[CHESS_ROWS][CHESS_COLS]) {
  for (int i=0; i<CHESS_ROWS; i++) {
    for (int j=0; j<CHESS_COLS; j++) {
      Serial.print(matrix[i][j], HEX);
      Serial.print("\t");
    }
    Serial.println();
  }
}
#endif // UTILS_H
