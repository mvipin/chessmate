// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include "Button.h"

#define BUTTON_CONFIRM_PIN 22
#define BUTTON_HINT_PIN 28
#define DEBOUNCE_DELAY 50

enum {
  BUTTON_CONFIRM,
  BUTTON_HINT,
  BUTTON_COUNT,
};

unsigned long last_debounce_time[BUTTON_COUNT] = {0, 0};  // the last time the output pin was toggled
uint8_t button_state[BUTTON_COUNT] = {LOW, LOW};             // the current reading from the input pin
uint8_t last_button_state[BUTTON_COUNT] = {LOW, LOW};   // the previous reading from the input pin
uint8_t button_pin[BUTTON_COUNT] = {BUTTON_CONFIRM_PIN, BUTTON_HINT_PIN};

void button_init() {
  pinMode(BUTTON_HINT_PIN, INPUT);
  pinMode(BUTTON_CONFIRM_PIN, INPUT);
}

void scan_buttons() {
  if ((state == MOVE_INIT) || (state == MOVE_NONE) || (state == MOVE_STOP)) {
    return;
  }

  for (uint8_t i=0; i<BUTTON_COUNT; i++) {
    // read the state of the switch into a local variable:
    int reading = digitalRead(button_pin[i]);

    // If the switch changed, due to noise or pressing:
    if (reading != last_button_state[i]) {
      // reset the debouncing timer
      last_debounce_time[i] = millis();
    }

    if ((millis() - last_debounce_time[i]) > DEBOUNCE_DELAY) {
      // whatever the reading is at, it's been there for longer than the debounce
      // delay, so take it as the actual current state:

      // if the button state has changed:
      if (reading != button_state[i]) {
        button_state[i] = reading;

        // only toggle the LED if the new button state is HIGH
        static bool skip_first[2] = {true, true}; // TODO: This is a WAR. FIXME.
        if (button_state[i] == HIGH) {
          if (skip_first[i]) {
            skip_first[i] = false;
            continue;
          }
          if (i == BUTTON_CONFIRM) {
            confirm = true;
          } else {
            hint = true;
          }
        }
      }
    }
    
    // save the reading. Next time through the loop, it'll be the last_button_state:
    last_button_state[i] = reading;
  }
}
