#include "Utils.h"
// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include "Serial.h"
#include "Display.h"
#include "Sensor.h"
#include "Mock.h"

#define MAX_TOKENS (4 * CHESS_ROWS + 2)

char cmdstr[CMD_LEN_MAX];

void print_legal_moves() {
  Serial.print("LEGAL MOVES: ");
  for (int i=0; i<legal_moves_cnt; i++) {
    Serial.print(legal_moves[i]);
    Serial.print(" ");
  }
  Serial.println();
  Serial.flush();
}

int parse_command(char command[], char* tokens[]) {
    uint8_t token_count = 0;
    char* token = strtok(command, ":");

    while (token != NULL && token_count < MAX_TOKENS) {
        tokens[token_count] = token;
        token_count++;
        token = strtok(NULL, ":");
    }

    return token_count; // Return the number of tokens found
}

void process_cmd(char cmd[], uint8_t size) {
  // Extract the opcode and data
  char* tokens[MAX_TOKENS];
  uint8_t num_tokens = parse_command(cmd, tokens);

  // Process the opcode and data
  uint8_t idx = 0;
  if (strcmp(tokens[idx],"init") == 0) {
    state = MOVE_INIT;
    Serial.println("INIT: Host initialized");
    Serial.flush();
    reset_display();
    lightup_display();
    Serial.println("INIT: Board reset complete");
    Serial.flush();
  } else if (strcmp(tokens[idx],"mode") == 0) {
    // Set operating mode: mode:real or mode:mock
    if (++idx < num_tokens) {
      if (strcmp(tokens[idx], "real") == 0) {
        mock_mode = false;
        Serial.println("MODE: Real hardware mode enabled");
      } else if (strcmp(tokens[idx], "mock") == 0) {
        mock_mode = true;
        Serial.println("MODE: Mock simulation mode enabled");
      } else {
        Serial.print("MODE: Unknown mode '");
        Serial.print(tokens[idx]);
        Serial.println("', keeping current mode");
      }
    }
    Serial.print("MODE: Current mode is ");
    Serial.println(mock_mode ? "MOCK" : "REAL");
    Serial.flush();
  } else if (strcmp(tokens[idx],"occupancy") == 0) {
    reset_occupancy();
    while (++idx < num_tokens) {
      uint8_t row = atoi(tokens[idx]) / CHESS_COLS;
      uint8_t col = atoi(tokens[idx]) % CHESS_ROWS;
      occupancy_init[row][col] = true;
    }

    // Initialize mock occupancy if in mock mode
    if (mock_mode) {
      mock_init_occupancy();
    }
    Serial.print("OCCUPANCY: Set ");
    Serial.print(num_tokens - 1);
    Serial.println(" pieces");
    Serial.flush();
  } else if (strcmp(tokens[idx],"legal") == 0) {
    // Reset legal moves count before processing new legal moves
    legal_moves_cnt = 0;
    check_squares_cnt = 0;  // Also reset check squares for clean state

    // Parse comma-separated legal moves from tokens[1]
    if (num_tokens > 1 && tokens[1] != NULL) {
      char moves_copy[CMD_LEN_MAX];
      strncpy(moves_copy, tokens[1], CMD_LEN_MAX - 1);
      moves_copy[CMD_LEN_MAX - 1] = '\0';

      char* move_token = strtok(moves_copy, ",");
      while (move_token != NULL && legal_moves_cnt < LEGAL_MOVES_MAX) {
        // Trim whitespace and copy move
        while (*move_token == ' ') move_token++; // Skip leading spaces
        if (strlen(move_token) >= 4) {
          strncpy(legal_moves[legal_moves_cnt], move_token, 4);
          legal_moves[legal_moves_cnt][4] = '\0';
          legal_moves_cnt++;
        }
        move_token = strtok(NULL, ",");
      }
    }
    Serial.print("LEGAL: Set ");
    Serial.print(legal_moves_cnt);
    Serial.println(" legal moves");
    Serial.flush();
  } else if (strcmp(tokens[idx],"hint") == 0) {
    strncpy(special_moves[MOVE_TYPE_HINT], tokens[++idx], 4);
    special_moves[MOVE_TYPE_HINT][4] = '\0';
  } else if (strcmp(tokens[idx],"check") == 0) {
    while (++idx < num_tokens) {
      if (check_squares_cnt >= CHECK_SQUARES_MAX) {
        Serial.println("no mem sq");
        display_fatal_error();
        delay(5000); // TODO: Reboot the platform
        return;
      }
      strncpy(check_squares[check_squares_cnt], tokens[idx], 2);
      check_squares[check_squares_cnt++][3] = '\0';
    }
  } else if (strcmp(tokens[idx],"start") == 0) {
    Serial.println("START: Human turn beginning");
    set_control_pixel(HUMAN, GREEN);
    set_control_pixel(COMPUTER, BLACK);
    print_legal_moves();
    state = MOVE_RESET;

    // Reset mock mode state for new turn
    if (mock_mode) {
      mock_move_in_progress = false;
      mock_hint_requested = false;
      Serial.println("MOCK: Ready for human turn");
    } else {
      Serial.println("REAL: Waiting for physical input");
    }
    Serial.flush();
  } else if (strcmp(tokens[idx],"override") == 0) {
    strncpy(special_moves[MOVE_TYPE_OVERRIDE], tokens[++idx], 4);
    special_moves[MOVE_TYPE_OVERRIDE][4] = '\0';
    while (!validate_occupancy());
    uint8_t row, col;
    xy_lookup(special_moves[MOVE_TYPE_OVERRIDE]+2, row, col);
    uint16_t color = GREEN;
    if (occupancy_init[row][col]) {
      color = ORANGE;
    }
    highlight_move(special_moves[MOVE_TYPE_OVERRIDE], GREEN, color);
    state = MOVE_OVERRIDE;
    Serial.print("override: ");
    Serial.println(special_moves[MOVE_TYPE_OVERRIDE]);
  } else if (strcmp(tokens[idx],"comp") == 0) {
    strncpy(special_moves[MOVE_TYPE_COMP], tokens[++idx], 4);
    special_moves[MOVE_TYPE_COMP][4] = '\0';
    while (!validate_occupancy());
    uint8_t row, col;
    xy_lookup(special_moves[MOVE_TYPE_COMP]+2, row, col);
    uint16_t color = GREEN;
    if (occupancy_init[row][col]) {
      color = ORANGE;
    }
    highlight_move(special_moves[MOVE_TYPE_COMP], GREEN, color);
    state = MOVE_COMP;
    Serial.print("comp: ");
    Serial.println(special_moves[MOVE_TYPE_COMP]);
    confirm = true;  // Auto-confirm computer moves
  } else if (strcmp(tokens[idx],"checkmate") == 0) {
    char dst[3];
    strncpy(dst, tokens[++idx], 2);
    dst[2] = '\0';
    strncpy(special_moves[MOVE_TYPE_CHECKMATE], tokens[++idx], 4);
    special_moves[MOVE_TYPE_CHECKMATE][4] = '\0';
    state = MOVE_CHECKMATE;
    reset_display();
    uint8_t row, col;
    xy_lookup(dst, row, col);
    for (int i=-1; i<=1; i++) {
      for (int j=-1; j<=1; j++) {
        if ((i == 0) && (j == 0)) continue; // king square itself
        if ((row == 0) && (i == -1)) continue; // skip negative row index
        if ((col == 0) && (j == -1)) continue; // skip negative col index
        if ((row == 7) && (i == 1)) continue; // skip out-of-bounds row index
        if ((col == 7) && (j == 1)) continue; // skip out-of-bounds col index
        if (!occupancy_init[row+i][col+j]) {
          update_display(row+i, col+j, ORANGE);
        }
      }
    }
    lightup_display();
  } else if (strcmp(tokens[idx],"reset") == 0) {
    state = MOVE_INIT;
    display_count_up();
    reset_display();
    lightup_display();
  }
}

String check_for_cmd() {
  // Use USB Serial for Pi communication
  String input = Serial.readStringUntil('\n');
  if (input != NULL) {
    input.trim();
  }
  return input;
}

void scan_serial() {
  if (state == MOVE_NONE) {
    static int chess_squares_lit = 0;
    chess_squares_lit = loading_status(chess_squares_lit);
  }
  
  if ((state == MOVE_INIT) || (state == MOVE_NONE) || (state == MOVE_STOP)) {
    String cmd = check_for_cmd();
    if (cmd != NULL) {
      cmd.toCharArray(cmdstr, CMD_LEN_MAX);
      process_cmd(cmdstr, CMD_LEN_MAX);
    }
  }
}

// To the host
void send_response(const char resp[]) {
  // Send response to Pi via USB Serial
  Serial.println(resp);
  Serial.flush();
}

void serial_init() {
  // USB Serial is already initialized in setup()
  // No additional initialization needed for host communication
}
