// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#ifndef HEAD_H
#define HEAD_H

typedef enum {
  EYE_LEFT,
  EYE_RIGHT,
} eye_t;

typedef enum {
  SERVO_RIGHT_EYELID,
  SERVO_RIGHT_EYEBALL,
  SERVO_RIGHT_EYELEVEL,
  SERVO_LEFT_EYELID,
  SERVO_LEFT_EYEBALL,
  SERVO_LEFT_EYELEVEL,
  SERVO_NUM_MAX,
} servo_id_t;

typedef enum {
  FULLY_RIGHT,
  PARTIALLY_RIGHT,
  SLIGHTLY_RIGHT,
  CENTER,
  SLIGHTLY_LEFT,
  PARTIALLY_LEFT,
  FULLY_LEFT,
} pos_clk_t;

typedef enum {
  FULLY_OPEN,
  PARTIALLY_OPEN,
  HALF_OPEN,
  PARTIALLY_CLOSED,
  FULLY_CLOSED,
} pos_gap_t;

typedef enum {
  FULLY_DOWN,
  LOWER_TILTED,
  SLIGHTLY_TILTED_DOWN,
  NEUTRAL,
  SLIGHTLY_TILTED_UP,
  RAISED_TILTED,
  FULLY_UP,
} pos_level_t;

typedef enum {
  DO_NOTHING,
  RESET_POSE,
  DOZE_OFF,
  WAKE_UP,
  THINK_HARD,
} animation_id_t;

typedef struct {
  uint16_t pulse_min;
  uint16_t pulse_max;
  uint16_t degree_low;
  uint16_t degree_high;
  unsigned long update_ts;
  uint16_t degree_cur;
} servo_cfg_t;

#endif // HEAD_H