#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include "Display.h"

#define DISPLAY_LED_PIN 12
#define CONTROL_LED_PIN 19
#define DISPLAY_LED_CNT (CHESS_ROWS * CHESS_COLS)
#define CONTROL_LED_CNT 2

Adafruit_NeoMatrix display_pixels(CHESS_COLS, CHESS_ROWS, DISPLAY_LED_PIN, NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG + NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel control_pixel(CONTROL_LED_CNT, CONTROL_LED_PIN, NEO_GRB + NEO_KHZ800);

const uint8_t display_map[CHESS_ROWS][CHESS_COLS] PROGMEM = {
  { 0,  1,  2,  3,  4,  5,  6,  7},
  {15, 14, 13, 12, 11, 10,  9,  8},
  {16, 17, 18, 19, 20, 21, 22, 23},
  {31, 30, 29, 28, 27, 26, 25, 24},
  {32, 33, 34, 35, 36, 37, 38, 39},
  {47, 46, 45, 44, 43, 42, 41, 40},
  {48, 49, 50, 51, 52, 53, 54, 55},
  {63, 62, 61, 60, 59, 58, 57, 56},
};

uint16_t remap_fn(uint16_t x, uint16_t y) {
  if (x < CHESS_ROWS && y < CHESS_COLS) {
    // Read the byte from PROGMEM
    return pgm_read_byte_near(&display_map[x][y]);
  }
  return 0;
}

void display_count_up() {
  for (uint8_t i = 0; i < CHESS_ROWS; i++) {
    for (uint8_t j = 0; j < CHESS_COLS; j++) {
      display_pixels.drawPixel(i,j,ORANGE);
      display_pixels.show();
      delay(50);
      display_pixels.drawPixel(i,j,BLACK);
      display_pixels.show();
    }
  }
}

void display_win(char side[]) {
  uint16_t color = BLUE;
  if (strcmp(side,"whit") == 0) {
    color = WHITE;
  }
  for (uint8_t i = 0; i < CHESS_ROWS; i++) {
    for (uint8_t j = 0; j < CHESS_COLS; j++) {
      reset_display();
      update_display(i, j, color);
      lightup_display();
      delay(100);
    }
  }
}

void display_fatal_error() {
  display_pixels.fillScreen(BLACK);
  display_pixels.drawLine(0, 0, 7, 7, RED);
  display_pixels.drawLine(7, 0, 0, 7, RED);
  display_pixels.show();
}

void display_board_status(uint8_t x_start, uint8_t y_start, uint8_t x_stop, uint8_t y_stop, uint16_t color) {
  for (uint8_t i = 0; i < CHESS_ROWS; i++) {
    for (uint8_t j = 0; j < CHESS_COLS; j++) {
      if ((i >= x_start) && (j >= y_start) && (i <= x_stop) && (j <= y_stop)) {
        update_display(i, j, color);
      } else {
        update_display(i, j, BLACK);
      }
    }
  }
  
  lightup_display();
}

int loading_status(int chess_squares_already_lit) {
  int target_row = 0;
  int target_column = 0;
  if (chess_squares_already_lit < 8){
    target_row = 0;
    target_column = chess_squares_already_lit;
  } else if (chess_squares_already_lit > 7){
    target_row = chess_squares_already_lit / 8;
    target_column = chess_squares_already_lit % 8;
  }
  uint16_t color = random(0, 0xFFFF);
  display_pixels.drawPixel(target_row,target_column,color);
  display_pixels.show();
  if ((target_row == 7) && (target_column == 7)) {
    return 0;
  }
  
  return chess_squares_already_lit + 1;
}

void reset_display() {
  for (uint8_t i = 0; i < CHESS_ROWS; i=i+2) {
    for (uint8_t j = 0; j < CHESS_COLS; j++) {
      if (j % 2 == 0) {
        display_pixels.drawPixel(i,j,WHITE);
        display_pixels.drawPixel(i+1,j,BLUE);
      } else {
        display_pixels.drawPixel(i,j,BLUE);
        display_pixels.drawPixel(i+1,j,WHITE);
      }
    }
  }
}

void lightup_display() {
  display_pixels.show();
}

void update_display(uint8_t i, uint8_t j, uint16_t color) {
  display_pixels.drawPixel(i,j, color);
}

uint32_t convert_16to24(uint16_t color) {
    // Extract the red, green, and blue components from the 16-bit color
    uint8_t r = (color >> 11) & 0x1F;
    uint8_t g = (color >> 5) & 0x3F;
    uint8_t b = color & 0x1F;

    // Scale up to 8-bit values
    uint8_t red = (r * 255) / 31;
    uint8_t green = (g * 255) / 63;
    uint8_t blue = (b * 255) / 31;

    return control_pixel.Color(red, green, blue);
}

void set_control_pixel(uint8_t idx, uint16_t color) {
  control_pixel.setPixelColor(idx, convert_16to24(color));
  control_pixel.show();
}

void highlight_move(char *move, uint16_t color_src, uint16_t color_dst) {
  char part[3];
  uint8_t row, col;

  // Source
  reset_display();
  strncpy(part, move, 2);
  xy_lookup(part, row, col);
  update_display(row, col, color_src);
  // Destination
  strncpy(part, move+2, 2);
  xy_lookup(part, row, col);
  update_display(row, col, color_dst);
  lightup_display();
}

void display_init() {
  display_pixels.begin();
  display_pixels.setTextWrap(false);
  display_pixels.setBrightness(50);
  control_pixel.begin();
  control_pixel.clear();
  control_pixel.setBrightness(10);
  display_pixels.setRemapFunction(remap_fn);
  set_control_pixel(0, RED);
  set_control_pixel(1, RED);
}
