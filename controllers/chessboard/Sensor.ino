#include <Arduino.h>
#include <Adafruit_MCP23X17.h>
// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include "Sensor.h"
#include "Display.h"

#define MOVEMENT_TYPE_ABSENT (1 << 0) // instantaneous state
#define MOVEMENT_TYPE_PRESENT (1 << 1) // instantaneous state
#define MOVEMENT_TYPE_REMOVE (1 << 2) // sticky bit, persistent for a single move
#define MOVEMENT_TYPE_ADD (1 << 3) // sticky bit, persistent for a single move

Adafruit_MCP23X17 mcp[NUM_BOARDS];

enum {
  X_START,
  Y_START,
  X_STOP,
  Y_STOP,
  XY_SIZE
};

const uint8_t sensor_coords[NUM_BOARDS][XY_SIZE] = {
  {0, 0, 3, 3},
  {0, 4, 3, 7},
  {4, 0, 7, 3},
  {4, 4, 7, 7},
};

const uint8_t sensor_pins[CHESS_ROWS][CHESS_COLS] PROGMEM = {
  { 3,  2,  1,  0, 15, 14, 13, 12},
  { 7,  6,  5,  4, 11, 10,  9,  8},
  {11, 10,  9,  8,  7,  6,  5,  4},
  {15, 14, 13, 12,  3,  2,  1,  0},
  { 3,  2,  1,  0, 15, 14, 13, 12},
  { 7,  6,  5,  4, 11, 10,  9,  8},
  {11, 10,  9,  8,  7,  6,  5,  4},
  {15, 14, 13, 12,  3,  2,  1,  0},
};

bool occupancy_init[CHESS_ROWS][CHESS_COLS];
uint8_t occupancy_delta[CHESS_ROWS][CHESS_COLS];
uint8_t i2c_addr[NUM_BOARDS] = {0x22, 0x23, 0x20, 0x21};

void reset_occupancy() {
  for (int i=0; i<CHESS_ROWS; i++) {
    for (int j=0; j<CHESS_COLS; j++) {
      occupancy_init[i][j] = false;
      occupancy_delta[i][j] = 0;
    }
  }
}

bool validate_occupancy() {
  bool ret = true;

  reset_display();
  for (uint8_t i=0; i<CHESS_ROWS; i++) {
    for (uint8_t j=0; j<CHESS_COLS; j++) {
      uint8_t id = board_id_lookup(i, j);
      bool present = !mcp[id].digitalRead(pgm_read_byte_near(&sensor_pins[i][j]));
      bool occupied = occupancy_init[i][j];
      if ((present && !occupied) || (!present && occupied)) {
        update_display(i, j, RED);
        ret = false;
      }
    }
  }
  lightup_display();

  return ret;
}

void show_valid_moves() {
  reset_display();
  // Highlight the possible moves for the piece(s) lifted
  for (uint8_t i=0; i<CHESS_ROWS; i++) {
    for (uint8_t j=0; j<CHESS_COLS; j++) {
      if (!(occupancy_delta[i][j] & MOVEMENT_TYPE_ABSENT)) continue;

      char src[3];
      get_algebraic_notation(i, j, src);
      update_display(i, j, YELLOW);
      for (uint8_t k=0; k<legal_moves_cnt; k++) {
        // Check if the first two characters of the move match the starting square
        if (strncmp(legal_moves[k], src, 2) == 0) {
          char *dst = legal_moves[k] + 2;
          uint8_t m, n;
          xy_lookup(dst, m, n);
          update_display(m, n, GREEN);
        }
      }
    }
  }
  lightup_display();
}

board_state_t occupancy_changed() {
  int8_t sum = 0;
  uint8_t changes = 0;
  for (uint8_t i=0; i<CHESS_ROWS; i++) {
    for (uint8_t j=0; j<CHESS_COLS; j++) {
      if (occupancy_delta[i][j] & MOVEMENT_TYPE_PRESENT) {
        sum += 1;
        changes++;
      } else if (occupancy_delta[i][j] & MOVEMENT_TYPE_ABSENT) {
        sum -= 1;
        changes++;
      }
    }
  }

  if ((sum > 0) || (sum < -1)) {
    Serial.println(sum);
    display_fatal_error();
    delay(5000); // TODO: Reboot the platform
    return BOARD_STATE_ERROR;
  }
  
  if (!changes) {
    return BOARD_STATE_NONE_MOVED;
  } else if (!sum) {
    return BOARD_STATE_PIECE_MOVED;
  }

  return BOARD_STATE_PIECE_REMOVED;
}

bool calculate_move_with_piece_moved(char move[]) {
  char dst[3];
  bool dst_found = false;
  // check for a valid destination
  for (uint8_t i=0; i<CHESS_ROWS; i++) {
    for (uint8_t j=0; j<CHESS_COLS; j++) {
      if (!(occupancy_delta[i][j] & MOVEMENT_TYPE_PRESENT)) continue;
      
      get_algebraic_notation(i, j, dst);
      for (uint8_t k=0; k<legal_moves_cnt; k++) {
        // Check if the last two characters of the move match the starting square
        if (strncmp(legal_moves[k]+2, dst, 2) == 0) {
          update_display(i, j, GREEN);
          dst_found = true;
          goto src;
        }
      }
    }
  }
  if (!dst_found) return false;

src:
  // check for a valid source
  char src[3];
  bool src_found = false;
  for (uint8_t i=0; i<CHESS_ROWS; i++) {
    for (uint8_t j=0; j<CHESS_COLS; j++) {
      if (!(occupancy_delta[i][j] & MOVEMENT_TYPE_ABSENT)) continue;
      
        get_algebraic_notation(i, j, src);
        for (uint8_t k=0; k<legal_moves_cnt; k++) {
          // Check if the last two characters of the move match the starting square
          if ((strncmp(legal_moves[k], src, 2) == 0) && (strncmp(legal_moves[k]+2, dst, 2) == 0)) {
            if (src_found) {
              Serial.print("Mul src: ");
              Serial.println(src);
              return false; // Multiple sources
            }
            update_display(i, j, GREEN);
            src_found = true;
          }
        }

    }
  }

  if (src_found) {
    strncpy(move, src, 2);
    strncpy(move+2, dst, 2);
    move[4] = '\0';
    return true;
  }

  return false;
}

bool calculate_move_with_piece_removed(char move[]) {
  char src[3];
  uint8_t i, j;
  bool found = false;
  for (i=0; (i<CHESS_ROWS) && !found; i++) {
    for (j=0; (j<CHESS_COLS) && !found; j++) {
      if (occupancy_delta[i][j] & MOVEMENT_TYPE_ABSENT) {
        get_algebraic_notation(i, j, src);
        Serial.print("src: ");
        Serial.println(src);
        found = true;
      }
    }
  }

  if (!found) return false;

  for (uint8_t k=0; k<legal_moves_cnt; k++) {
     // Check if the first two characters of the move match the starting square
     if (strncmp(legal_moves[k], src, 2) == 0) {
        char *dst = legal_moves[k] + 2;
        Serial.print("dst: ");
        Serial.println(dst);
        uint8_t m, n;
        xy_lookup(dst, m, n);
        uint8_t delta = occupancy_delta[m][n];
        if ((delta & MOVEMENT_TYPE_ADD) && (delta & MOVEMENT_TYPE_REMOVE)) {
          strncpy(move, legal_moves[k], 4);
          move[4] = '\0';
          update_display(i, j, GREEN); // source
          update_display(m, n, ORANGE); // destination
          return true;
        }
     }
  }

  return false;
}

bool compute_move(char move[]) {
  bool status = false;
  
  reset_display();
  board_state_t change = occupancy_changed();
  if (change == BOARD_STATE_NONE_MOVED) {
    Serial.println("none");
  } else if (change == BOARD_STATE_PIECE_MOVED) {
    status = calculate_move_with_piece_moved(move);
    Serial.println("moved");
  } else if (change == BOARD_STATE_PIECE_REMOVED) {
    status = calculate_move_with_piece_removed(move);
    Serial.println("removed");
  }
  lightup_display();
  //print_matrix(occupancy_delta);

  return status;
}

void compute_delta() {
  for (uint8_t i=0; i<CHESS_ROWS; i++) {
    for (uint8_t j=0; j<CHESS_COLS; j++) {
      uint8_t id = board_id_lookup(i, j);
      bool present = !mcp[id].digitalRead(pgm_read_byte_near(&sensor_pins[i][j]));
      bool occupied = occupancy_init[i][j];
      occupancy_delta[i][j] &= ~(MOVEMENT_TYPE_ABSENT | MOVEMENT_TYPE_PRESENT);
      if (occupied && !present) {
        occupancy_delta[i][j] |= MOVEMENT_TYPE_REMOVE;
        occupancy_delta[i][j] |= MOVEMENT_TYPE_ABSENT;
      } else if (!occupied && present) {
        occupancy_delta[i][j] |= MOVEMENT_TYPE_ADD;
        occupancy_delta[i][j] |= MOVEMENT_TYPE_PRESENT;
      }
      if ((occupancy_delta[i][j] & MOVEMENT_TYPE_REMOVE) && present) {
        occupancy_delta[i][j] |= MOVEMENT_TYPE_ADD;
      }
      if ((occupancy_delta[i][j] & MOVEMENT_TYPE_ADD) && !present) {
        occupancy_delta[i][j] |= MOVEMENT_TYPE_REMOVE;
      }
    }
  }
}

void scan_sensors() {
  if (state == MOVE_RESET) {
    while (!validate_occupancy());
    state = MOVE_START;
  }
  
  if (state == MOVE_START) {
    compute_delta();
    show_valid_moves();
  }
}

uint8_t board_id_lookup(uint8_t row, uint8_t col) {
  for (uint8_t id = 0; id < NUM_BOARDS; id++) {
    if (row >= sensor_coords[id][X_START] && row <= sensor_coords[id][X_STOP] &&
      col >= sensor_coords[id][Y_START] && col <= sensor_coords[id][Y_STOP]) {
      return id;
    }
  }

  return NUM_BOARDS; // invalid value. check against this for error
}

void sensor_init() {
  for (uint8_t id = 0; id < NUM_BOARDS; id++) {
    uint16_t color = RED;
    if (!mcp[id].begin_I2C(i2c_addr[id])) {
      Serial.print("Sensor board not responding, id: ");

    } else {
      Serial.print("Sensor board OK, id: ");
      color = GREEN;
    }
    Serial.println(id);
    display_board_status(sensor_coords[id][X_START], sensor_coords[id][Y_START], sensor_coords[id][X_STOP], sensor_coords[id][Y_STOP], color);
    delay(500);
    display_board_status(0, 0, 7, 7, BLACK);
  }

  // configure pin for input with pull up
  for (uint8_t id = 0; id < NUM_BOARDS; id++) {
    for (uint8_t pin = 0; pin < 16; pin++) {
      mcp[id].pinMode(pin, INPUT_PULLUP);
    }
  }
}
