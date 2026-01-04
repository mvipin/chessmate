// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

servo_cfg_t servo_cfg[SERVO_NUM_MAX] = {
  {80, 530, 35, 100, 0, 67}, // SERVO_RIGHT_EYELID (0)
  {80, 530, 40, 120, 0, 80}, // SERVO_RIGHT_EYEBALL (1)
  {100, 500, 0, 180, 0, 90}, // SERVO_RIGHT_EYELEVEL (2)
  {80, 530, 80, 135, 0, 107}, // SERVO_LEFT_EYELID (3)
  {80, 530, 55, 140, 0, 97}, // SERVO_LEFT_EYEBALL (4)
  {100, 500, 0, 180, 0, 90}, // SERVO_LEFT_EYELEVEL (5)
};

bool set_eyeball(eye_t eye, pos_clk_t clk, unsigned long interval)
{
  float factor;
  switch (clk) {
    case FULLY_RIGHT: {
      factor = 1.00f;
    } break;
    case PARTIALLY_RIGHT: {
      factor = 0.83f;
    } break;
    case SLIGHTLY_RIGHT: {
      factor = 0.66f;
    } break;
    case CENTER: {
      factor = 0.50f;
    } break;
    case SLIGHTLY_LEFT: {
      factor = 0.34f;
    } break;
    case PARTIALLY_LEFT: {
      factor = 0.17f;
    } break;
    case FULLY_LEFT: {
      factor = 0.00f;
    } break;
  }

  servo_id_t servo_id = (eye == EYE_RIGHT) ? SERVO_RIGHT_EYEBALL : SERVO_LEFT_EYEBALL;
  uint16_t degrees = (uint16_t)(servo_cfg[servo_id].degree_low + ((servo_cfg[servo_id].degree_high - servo_cfg[servo_id].degree_low) * factor));
  if (!interval) {
    servo_cfg[servo_id].degree_cur = degrees;
    uint16_t pulse_len = map(servo_cfg[servo_id].degree_cur, 0, 180, servo_cfg[servo_id].pulse_min, servo_cfg[servo_id].pulse_max);
    pwm.setPWM(servo_id, 0, pulse_len);
  }
  if (degrees == servo_cfg[servo_id].degree_cur) {
    return true;
  }
  unsigned long currentMillis = millis();
  if ((currentMillis - servo_cfg[servo_id].update_ts) > interval) {
    if (servo_cfg[servo_id].degree_cur < degrees) {
      servo_cfg[servo_id].degree_cur += 1;
    } else if (servo_cfg[servo_id].degree_cur > degrees) {
      servo_cfg[servo_id].degree_cur -= 1;
    }
    uint16_t pulse_len = map(servo_cfg[servo_id].degree_cur, 0, 180, servo_cfg[servo_id].pulse_min, servo_cfg[servo_id].pulse_max);
    pwm.setPWM(servo_id, 0, pulse_len);
    servo_cfg[servo_id].update_ts = currentMillis;
  }

  return false;
}

bool set_eyelid(eye_t eye, pos_gap_t gap, unsigned long interval)
{
  float factor;
  switch (gap) {
    case FULLY_OPEN: {
      factor = (eye == EYE_RIGHT) ? 1.00f : 0.00f;
    } break;
    case PARTIALLY_OPEN: {
      factor = (eye == EYE_RIGHT) ? 0.75f : 0.25f;
    } break;
    case HALF_OPEN: {
      factor = 0.50f;
    } break;
    case PARTIALLY_CLOSED: {
      factor = (eye == EYE_RIGHT) ? 0.25f : 0.75f;
    } break;
    case FULLY_CLOSED: {
      factor = (eye == EYE_RIGHT) ? 0.00f : 1.00f;
    } break;
  }

  servo_id_t servo_id = (eye == EYE_RIGHT) ? SERVO_RIGHT_EYELID : SERVO_LEFT_EYELID;
  uint16_t degrees = servo_cfg[servo_id].degree_low + ((servo_cfg[servo_id].degree_high - servo_cfg[servo_id].degree_low) * factor);
  if (!interval) {
    servo_cfg[servo_id].degree_cur = degrees;
    uint16_t pulse_len = map(servo_cfg[servo_id].degree_cur, 0, 180, servo_cfg[servo_id].pulse_min, servo_cfg[servo_id].pulse_max);
    pwm.setPWM(servo_id, 0, pulse_len);
  }
  if (degrees == servo_cfg[servo_id].degree_cur) {
    return true;
  }
  unsigned long currentMillis = millis();
  if ((currentMillis - servo_cfg[servo_id].update_ts) > interval) {
    if (servo_cfg[servo_id].degree_cur < degrees) {
      servo_cfg[servo_id].degree_cur += 1;
    } else if (servo_cfg[servo_id].degree_cur > degrees) {
      servo_cfg[servo_id].degree_cur -= 1;
    }
    uint16_t pulse_len = map(servo_cfg[servo_id].degree_cur, 0, 180, servo_cfg[servo_id].pulse_min, servo_cfg[servo_id].pulse_max);
    pwm.setPWM(servo_id, 0, pulse_len);
    servo_cfg[servo_id].update_ts = currentMillis;
  }

  return false;
}

bool set_eyelevel(eye_t eye, pos_level_t level, unsigned long interval)
{
  float factor;
  switch (level) {
    case FULLY_DOWN: {
      factor = (eye == EYE_RIGHT) ? 1.00f : 0.00f;
    } break;
    case LOWER_TILTED: {
      factor = (eye == EYE_RIGHT) ? 0.83f : 0.17f;
    } break;
    case SLIGHTLY_TILTED_DOWN: {
      factor = (eye == EYE_RIGHT) ? 0.66f : 0.34f;
    } break;
    case NEUTRAL: {
      factor = 0.50f;
    } break;
    case SLIGHTLY_TILTED_UP: {
      factor = (eye == EYE_RIGHT) ? 0.34f : 0.66f;
    } break;
    case RAISED_TILTED: {
      factor = (eye == EYE_RIGHT) ? 0.17f : 0.83f;
    } break;
    case FULLY_UP: {
      factor = (eye == EYE_RIGHT) ? 0.00f : 1.00f;
    } break;
  }

  servo_id_t servo_id = (eye == EYE_RIGHT) ? SERVO_RIGHT_EYELEVEL : SERVO_LEFT_EYELEVEL;
  uint16_t degrees = servo_cfg[servo_id].degree_low + ((servo_cfg[servo_id].degree_high - servo_cfg[servo_id].degree_low) * factor);
  if (!interval) {
    servo_cfg[servo_id].degree_cur = degrees;
    uint16_t pulse_len = map(servo_cfg[servo_id].degree_cur, 0, 180, servo_cfg[servo_id].pulse_min, servo_cfg[servo_id].pulse_max);
    pwm.setPWM(servo_id, 0, pulse_len);
  }
  if (degrees == servo_cfg[servo_id].degree_cur) {
    return true;
  }
  unsigned long currentMillis = millis();
  if ((currentMillis - servo_cfg[servo_id].update_ts) > interval) {
    if (servo_cfg[servo_id].degree_cur < degrees) {
      servo_cfg[servo_id].degree_cur += 1;
    } else if (servo_cfg[servo_id].degree_cur > degrees) {
      servo_cfg[servo_id].degree_cur -= 1;
    }
    uint16_t pulse_len = map(servo_cfg[servo_id].degree_cur, 0, 180, servo_cfg[servo_id].pulse_min, servo_cfg[servo_id].pulse_max);
    pwm.setPWM(servo_id, 0, pulse_len);
    servo_cfg[servo_id].update_ts = currentMillis;
  }

  return false;
}

void eyelids_test()
{
#define INTERVAL 20
  for (int gap = FULLY_OPEN; gap <= FULLY_CLOSED; gap++) {
    set_eyelid(EYE_LEFT, (pos_gap_t)gap, 0);
    set_eyelid(EYE_RIGHT, (pos_gap_t)gap, 0);
  }
  for (int gap = FULLY_CLOSED; gap >= FULLY_OPEN; gap--) {
    set_eyelid(EYE_LEFT, (pos_gap_t)gap, 0);
    set_eyelid(EYE_RIGHT, (pos_gap_t)gap, 0);
  }
}

void eyeballs_test()
{
#define INTERVAL 20
  set_eyelid(EYE_LEFT, FULLY_OPEN, 0);
  set_eyelid(EYE_RIGHT, FULLY_OPEN, 0);
  for (int clk = FULLY_RIGHT; clk <= FULLY_LEFT; clk++) {
    set_eyeball(EYE_LEFT, (pos_clk_t)clk, 0);
    set_eyeball(EYE_RIGHT, (pos_clk_t)clk, 0);
    delay(500);
  }
  for (int clk = FULLY_LEFT; clk >= FULLY_RIGHT; clk--) {
    set_eyeball(EYE_LEFT, (pos_clk_t)clk, 0);
    set_eyeball(EYE_RIGHT, (pos_clk_t)clk, 0);
    delay(500);
  }
}

void eyelevel_test()
{
#define INTERVAL 20
  set_eyelid(EYE_LEFT, HALF_OPEN, 0);
  set_eyelid(EYE_RIGHT, HALF_OPEN, 0);
  for (int level = FULLY_DOWN; level <= FULLY_UP; level++) {
    set_eyelevel(EYE_LEFT, (pos_level_t)level, 0);
    set_eyelevel(EYE_RIGHT, (pos_level_t)level, 0);
    delay(1000);
  }
  for (int level = FULLY_UP; level >= FULLY_DOWN; level--) {
    set_eyelevel(EYE_LEFT, (pos_level_t)level, 0);
    set_eyelevel(EYE_RIGHT, (pos_level_t)level, 0);
    delay(1000);
  }
}

void expression_1()
{
#define INTERVAL 20
  static uint32_t previousMillis = 0;
  static int step = 0;
  uint32_t currentMillis = millis();

  switch (step) {
    case 0: {
      if (currentMillis - previousMillis >= 700) {
        set_eyelid(EYE_LEFT, FULLY_OPEN, INTERVAL);
        set_eyelid(EYE_RIGHT, FULLY_OPEN, INTERVAL);
        set_eyeball(EYE_LEFT, CENTER, 5);
        set_eyeball(EYE_RIGHT, CENTER, 5);
        set_eyelevel(EYE_LEFT, SLIGHTLY_TILTED_DOWN, INTERVAL);
        set_eyelevel(EYE_RIGHT, SLIGHTLY_TILTED_UP, INTERVAL);
        previousMillis = currentMillis;
        step = 1;
      }
    } break;
    case 1: {
      if (currentMillis - previousMillis >= 700) {
        set_eyelevel(EYE_LEFT, NEUTRAL, INTERVAL);
        set_eyelevel(EYE_RIGHT, NEUTRAL, INTERVAL);
        previousMillis = currentMillis;
        step = 2;
      }
    }  break;
    case 2: {
      if (currentMillis - previousMillis >= 700) {
        set_eyelevel(EYE_LEFT, SLIGHTLY_TILTED_UP, INTERVAL);
        set_eyelevel(EYE_RIGHT, SLIGHTLY_TILTED_DOWN, INTERVAL);
        previousMillis = currentMillis;
        step = 3;
      }
    } break;
    case 3: {
      if (currentMillis - previousMillis >= 700) {
        set_eyelevel(EYE_RIGHT, NEUTRAL, INTERVAL);
        set_eyelevel(EYE_LEFT, NEUTRAL, INTERVAL);
        previousMillis = currentMillis;
        step = 4;
      }
    } break;
    case 4: {
      if (currentMillis - previousMillis >= 700) {
        set_eyeball(EYE_RIGHT, FULLY_RIGHT, 5);
        set_eyeball(EYE_LEFT, FULLY_RIGHT, 5);
        previousMillis = currentMillis;
        step = 5;
      }
    } break;
    case 5: {
      if (currentMillis - previousMillis >= 700) {
        set_eyeball(EYE_RIGHT, CENTER, 5);
        set_eyeball(EYE_LEFT, CENTER, 5);
        previousMillis = currentMillis;
        step = 6;
      }
    } break;
    case 6: {
      if (currentMillis - previousMillis >= 700) {
        set_eyeball(EYE_RIGHT, FULLY_LEFT, 5);
        set_eyeball(EYE_LEFT, FULLY_LEFT, 5);
        previousMillis = currentMillis;
        step = 7;
      }
    } break;
    case 7: {
      if (currentMillis - previousMillis >= 700) {
        set_eyeball(EYE_RIGHT, CENTER, 5);
        set_eyeball(EYE_LEFT, CENTER, 5);
        previousMillis = currentMillis;
        step = 0; // Reset state to 0 or remove this to stop repeating
      }
    } break;
  }
}

void animate_reset_pose()
{
  set_eyelevel(EYE_RIGHT, NEUTRAL, 0);
  set_eyelevel(EYE_LEFT, NEUTRAL, 0);
  set_eyelid(EYE_RIGHT, HALF_OPEN, 0);
  set_eyelid(EYE_LEFT, HALF_OPEN, 0);
  set_eyeball(EYE_RIGHT, CENTER, 0);
  set_eyeball(EYE_LEFT, CENTER, 0);
  animation_id = DO_NOTHING;
}

void animate_think_hard()
{
#define INTERVAL 20
  static int step = 0;

  switch (step) {
    case 0: {
      bool done1 = set_eyelevel(EYE_RIGHT, FULLY_DOWN, INTERVAL);
      bool done2 = set_eyelevel(EYE_LEFT, FULLY_DOWN, INTERVAL);
      bool done3 = set_eyelid(EYE_RIGHT, PARTIALLY_CLOSED, INTERVAL);
      bool done4 = set_eyelid(EYE_LEFT, PARTIALLY_CLOSED, INTERVAL);
      bool done5 = set_eyeball(EYE_RIGHT, CENTER, INTERVAL);
      bool done6 = set_eyeball(EYE_LEFT, CENTER, INTERVAL);
      if (done1 && done2 && done3 && done4 && done5 && done6) {
        step = 1;
      }
    } break;
    case 1: {
      bool done1 = set_eyeball(EYE_RIGHT, PARTIALLY_LEFT, INTERVAL);
      bool done2 = set_eyeball(EYE_LEFT, PARTIALLY_LEFT, INTERVAL);
      if (done1 && done2) {
        step = 2;
      }
    } break;
    case 2: {
      bool done1 = set_eyeball(EYE_RIGHT, PARTIALLY_RIGHT, INTERVAL);
      bool done2 = set_eyeball(EYE_LEFT, PARTIALLY_RIGHT, INTERVAL);
      if (done1 && done2) {
        step = 0;
      }
    } break;
  }
}

void animate_doze_off()
{
#define INTERVAL 20
  static int step = 0;
  static unsigned long previousMillis = 0;
  static unsigned long lastvisitMillis = 0;
  unsigned long currentMillis = millis();

  // If last time we got interrupted without actually going to step 3, it will reset the state to 0
  if ((currentMillis - lastvisitMillis) >= 15000) {
    step = 0;
  }

  switch (step) {
    case 0: {
      previousMillis = currentMillis;
      step = 1;
    } break;
    case 1: {
      if (currentMillis - previousMillis >= 10000) {
        bool done1 = set_eyelevel(EYE_RIGHT, LOWER_TILTED, INTERVAL);
        bool done2 = set_eyelevel(EYE_LEFT, SLIGHTLY_TILTED_UP, INTERVAL);
        bool done3 = set_eyelid(EYE_RIGHT, FULLY_CLOSED, INTERVAL);
        bool done4 = set_eyelid(EYE_LEFT, FULLY_CLOSED, INTERVAL);
        if (done1 && done2 && done3 && done4) {
          step = 2;
        }
      }
    } break;
    case 2: {
      bool done1 = set_eyelid(EYE_RIGHT, PARTIALLY_CLOSED, INTERVAL);
      bool done2 = set_eyelid(EYE_LEFT, PARTIALLY_CLOSED, INTERVAL);
      if (done1 && done2) {
        step = 3;
      }
    } break;
    case 3: {
      bool done1 = set_eyelid(EYE_RIGHT, FULLY_CLOSED, INTERVAL);
      bool done2 = set_eyelid(EYE_LEFT, FULLY_CLOSED, INTERVAL);
      if (done1 && done2) {
        step = 0;
        animation_id = DO_NOTHING;
      }
    } break;
  }
  lastvisitMillis = currentMillis;
}

void animate_wake_up()
{
  set_eyelevel(EYE_RIGHT, RAISED_TILTED, 0);
  set_eyelevel(EYE_LEFT, RAISED_TILTED, 0);
  set_eyelid(EYE_RIGHT, FULLY_OPEN, 0);
  set_eyelid(EYE_LEFT, FULLY_OPEN, 0);
  animation_id = THINK_HARD;
}

void simple_test()
{
  set_eyelevel(EYE_LEFT, FULLY_UP, 20);
  set_eyelevel(EYE_RIGHT, FULLY_UP, 20);
}

void animate()
{
  switch (animation_id) {
    case RESET_POSE: {
      animate_reset_pose();
    } break;

    case DOZE_OFF: {
      animate_doze_off();
    } break;

    case WAKE_UP: {
      animate_wake_up();
    } break;

    case THINK_HARD: {
      animate_think_hard();
    } break;
  }
}

void head_init()
{
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}