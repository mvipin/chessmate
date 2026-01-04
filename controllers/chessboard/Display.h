// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#ifndef DISPLAY_H
#define DISPLAY_H

#include "Utils.h"

// Display function prototypes
void display_init();
void reset_display();
void lightup_display();
void update_display(uint8_t row, uint8_t col, uint16_t color);
void set_control_pixel(uint8_t type, uint16_t color);
void highlight_move(char *move, uint16_t color_src, uint16_t color_dst);
void display_count_up();
void display_fatal_error();
void display_win(char *move);
int loading_status(int chess_squares_lit);

#endif // DISPLAY_H
