// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include "Mock.h"

// Mock mode functionality for ChessBoard

void mock_init_occupancy() {
  // Initialize mock occupancy to match real board state
  for (int i = 0; i < CHESS_ROWS; i++) {
    for (int j = 0; j < CHESS_COLS; j++) {
      mock_occupancy[i][j] = occupancy_init[i][j];
    }
  }
  Serial.println("MOCK: Occupancy initialized");
}

void mock_update_occupancy(const char* move) {
  // Update mock occupancy based on move
  if (strlen(move) >= 4) {
    uint8_t from_row, from_col, to_row, to_col;
    xy_lookup(move, from_row, from_col);
    xy_lookup(move + 2, to_row, to_col);

    // Move piece from source to destination
    mock_occupancy[from_row][from_col] = false;
    mock_occupancy[to_row][to_col] = true;

    // Update real occupancy to match
    occupancy_init[from_row][from_col] = false;
    occupancy_init[to_row][to_col] = true;

    Serial.print("MOCK: Updated occupancy for move ");
    Serial.println(move);
  }
}

char* mock_select_random_move() {
  // Select random move from legal moves
  if (legal_moves_cnt > 0) {
    uint8_t selected_idx = random(legal_moves_cnt);
    strncpy(mock_selected_move, legal_moves[selected_idx], 4);
    mock_selected_move[4] = '\0';
    Serial.print("MOCK: Selected move ");
    Serial.print(mock_selected_move);
    Serial.print(" from ");
    Serial.print(legal_moves_cnt);
    Serial.println(" legal moves");
    return mock_selected_move;
  }
  return nullptr;
}

void mock_start_move() {
  if (!mock_mode || legal_moves_cnt == 0) return;

  // Decide if we want a hint first
  if (random(100) < MOCK_HINT_PROBABILITY && !mock_hint_requested) {
    Serial.println("MOCK: Requesting hint...");
    mock_hint_requested = true;
    hint = true;
    return;
  }

  // Start making a move
  mock_move_start_time = millis();
  mock_move_duration = random(MOCK_MOVE_MIN_TIME, MOCK_MOVE_MAX_TIME);
  mock_move_in_progress = true;
  mock_hint_requested = false;

  // Select the move we'll make
  mock_select_random_move();

  Serial.print("MOCK: Starting move, will take ");
  Serial.print(mock_move_duration);
  Serial.println("ms");
}

bool mock_check_move_complete() {
  if (!mock_mode || !mock_move_in_progress) return false;

  unsigned long elapsed = millis() - mock_move_start_time;
  if (elapsed >= mock_move_duration) {
    Serial.println("MOCK: Move completed, confirming...");
    mock_move_in_progress = false;

    // Update occupancy to simulate the move
    mock_update_occupancy(mock_selected_move);

    // Trigger confirmation
    confirm = true;
    return true;
  }
  return false;
}
