// Copyright (c) 2025 Vipin M
// Licensed under the MIT License. See LICENSE file in the project root for full license information.

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <assert.h>
#include <math.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

#undef MULTISTEPPER_ENABLED //change to #undef if we want the motors to accelerate/decelerate

#define STEPPER_X_STP_PIN 2
#define STEPPER_Y_STP_PIN 3 
#define STEPPER_Z_STP_PIN 4
#define STEPPER_X_DIR_PIN 5 
#define STEPPER_Y_DIR_PIN 6
#define STEPPER_Z_DIR_PIN 7
#define STEPPER_ENABLE_PIN 8
#define LIMIT_SWITCH_X_PIN 9
#define LIMIT_SWITCH_Y_PIN 10
#define LIMIT_SWITCH_Z_PIN 11
#define SERIAL_RX_PIN 12
#define SERIAL_TX_PIN A3

#define GRIPPER_SERVO_PIN 13
#define GRIPPER_OPEN_ANGLE 20
#define GRIPPER_CLOSE_ANGLE 100

#define MOTOR_INTERFACE_TYPE 1

#define ARM_1_SIZE 202.0f // mm
#define ARM_2_SIZE 190.0f // mm
#define LOWER_LEFT_CORNER_X -116.65 // origin at base
#define LOWER_LEFT_CORNER_Y 137.0
#define UPPER_LEFT_CORNER_X -116.65
#define UPPER_LEFT_CORNER_Y 370.33 // 137 + 233.33
#define UPPER_RIGHT_CORNER_X 116.65
#define UPPER_RIGHT_CORNER_Y 370.33
#define LOWER_RIGHT_CORNER_X 116.65
#define LOWER_RIGHT_CORNER_Y 137.0
#define CHESS_ROWS 8
#define CHESS_COLS 8

#define EEPROM_START_ADDRESS 0
#define ANGLE_DATA_SIZE 8 // bytes per square

#define MAX_SPEED_X 2500
#define REDUCED_SPEED_X (MAX_SPEED_X >> 1)
#define HOMING_SPEED_X (-MAX_SPEED_X)
#define HOMING_REDUCED_SPEED_X (-(MAX_SPEED_X >> 3))
#define ACCELERATION_X 2000

#define MAX_SPEED_Y 3500
#define REDUCED_SPEED_Y (MAX_SPEED_Y >> 1)
#define HOMING_SPEED_Y (-MAX_SPEED_Y)
#define HOMING_REDUCED_SPEED_Y (-(MAX_SPEED_Y >> 3))
#define ACCELERATION_Y 2000

#define MAX_SPEED_Z 1200
#define REDUCED_SPEED_Z (MAX_SPEED_Z >> 1)
#define HOMING_SPEED_Z (-REDUCED_SPEED_Z)
#define HOMING_REDUCED_SPEED_Z (-(MAX_SPEED_Z >> 1))
#define ACCELERATION_Z 1500
#define Z_MIN 200
#define Z_LOW 500
#define Z_MAX 2500

#define Z_MIN_PAWN_OFFSET 0
#define Z_MIN_ROOK_OFFSET 25
#define Z_MIN_KNIGHT_OFFSET -100
#define Z_MIN_BISHOP_OFFSET 150
#define Z_MIN_QUEEN_OFFSET 300
#define Z_MIN_KING_OFFSET 300

#define PULLEY_RATIO 1.00 //1.879
#define STEPS_REF_X 820
#define STEPS_REF_Y 7720
#define STEPS_PER_DEGREE_X 64.0f // 72/20 (gear down ratio) * 200 (steps per revolution) * 32 (microstepping) / 360 (degrees)
#define STEPS_PER_DEGREE_Y 55.11f // 62/20 (gear down ratio 1) * 62/62 (gear down ratio 2) * 200 * 32 / 360
#define STEPS_CNT(_deg, _dir) \
  ((_deg * STEPS_PER_DEGREE_##_dir) + STEPS_REF_##_dir)
#define STEP_ZERO_ANGLE(_dir) \
  (-STEPS_REF_##_dir / STEPS_PER_DEGREE_##_dir)

#define CAL_LARGE_STEP 5.0
#define CAL_SMALL_STEP 0.5

SoftwareSerial chessboard(SERIAL_RX_PIN, SERIAL_TX_PIN);

uint32_t z_min[PIECE_TYPE_NUM] = {
  Z_MIN + Z_MIN_PAWN_OFFSET,
  Z_MIN + Z_MIN_ROOK_OFFSET,
  Z_MIN + Z_MIN_KNIGHT_OFFSET,
  Z_MIN + Z_MIN_BISHOP_OFFSET,
  Z_MIN + Z_MIN_QUEEN_OFFSET,
  Z_MIN + Z_MIN_KING_OFFSET,
};

AccelStepper stepperX = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_X_STP_PIN, STEPPER_X_DIR_PIN);  
AccelStepper stepperY = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_Y_STP_PIN, STEPPER_Y_DIR_PIN);  
AccelStepper stepperZ = AccelStepper(MOTOR_INTERFACE_TYPE, STEPPER_Z_STP_PIN, STEPPER_Z_DIR_PIN);  

MultiStepper multistepper;
long multistepper_positions[2];
Servo gripper;
double angle1 = 0.0;
double angle2 = 0.0;
int current_square_index; // From 0 to 63 for the 64 squares

struct grid {
  double x;
  double y;
} corner[] = {
  {LOWER_LEFT_CORNER_X,LOWER_LEFT_CORNER_Y},
  {UPPER_LEFT_CORNER_X,UPPER_LEFT_CORNER_Y},
  {UPPER_RIGHT_CORNER_X,UPPER_RIGHT_CORNER_Y},
  {LOWER_RIGHT_CORNER_X,LOWER_RIGHT_CORNER_Y},
};

void gripper_close()
{
  gripper.write(GRIPPER_CLOSE_ANGLE); 
}

void gripper_open()
{
  gripper.write(GRIPPER_OPEN_ANGLE);
}

void move_xy_to(double x_deg, double y_deg)
{
  long x_steps = STEPS_CNT(x_deg, X);
  long y_steps = STEPS_CNT(y_deg, Y);
  //Serial.print(x_steps);
  //Serial.print(", ");
  //Serial.println(y_steps);

#ifdef MULTISTEPPER_ENABLED
  multistepper_positions[0] = x_steps;
  multistepper_positions[1] = y_steps;
  multistepper.moveTo(multistepper_positions);
  multistepper.runSpeedToPosition(); // Blocks until all are in position
#else
  stepperX.moveTo(x_steps);
  stepperY.moveTo(y_steps);
  while ((stepperX.currentPosition() != x_steps) || (stepperY.currentPosition() != y_steps)) {
    stepperX.run();
    stepperY.run();
    animate();
  }
#endif
}

void move_x_to(double degree)
{
  long steps = STEPS_CNT(degree, X);
  stepperX.moveTo(steps);
  while (stepperX.currentPosition() != steps) {
    stepperX.run();
    animate();
  }
}

void move_y_to(double degree)
{
  long steps = STEPS_CNT(degree, Y);
  stepperY.moveTo(steps);
  while (stepperY.currentPosition() != steps) {
    stepperY.run();
    animate();
  }
}

void move_z_to(long steps)
{
  stepperZ.moveTo(steps);
  while (stepperZ.currentPosition() != steps) {
    stepperZ.run();
  }
}

void home_x()
{
  // Search mode - move quickly until the limit switch is triggered
  stepperX.setSpeed(HOMING_SPEED_X);
  while (!digitalRead(LIMIT_SWITCH_X_PIN)) {
    stepperX.runSpeed();
  }
  stepperX.stop();

  // Locate mode - move slowly away from the switch and then back to engage it slowly
  stepperX.move(200); // Move 100 steps away
  while (stepperX.distanceToGo() > 0) {
    stepperX.run();
  }
  delay(100);
  stepperX.setSpeed(HOMING_REDUCED_SPEED_X); // Move back towards the switch slowly
  while (!digitalRead(LIMIT_SWITCH_X_PIN)) {
    stepperX.runSpeed();
  }
  stepperX.stop();

  // Pull-off motion - move away from the limit switch to disengage it
  stepperX.move(50); // Move away from the switch to disengage
  while (stepperX.distanceToGo() > 0) {
    stepperX.run();
  }

  stepperX.setCurrentPosition(0); // When limit switch pressed set position to 0 steps
}

void home_y()
{
 // Search mode - move quickly until the limit switch is triggered
  stepperY.setSpeed(HOMING_SPEED_Y);
  while (!digitalRead(LIMIT_SWITCH_Y_PIN)) {
    stepperY.runSpeed();
  }
  stepperY.stop();

  // Locate mode - move slowly away from the switch and then back to engage it slowly
  stepperY.move(200); // Move 100 steps away
  while (stepperY.distanceToGo() > 0) {
    stepperY.run();
  }
  delay(100);
  stepperY.setSpeed(HOMING_REDUCED_SPEED_Y); // Move back towards the switch slowly
  while (!digitalRead(LIMIT_SWITCH_Y_PIN)) {
    stepperY.runSpeed();
  }
  stepperY.stop();

  // Pull-off motion - move away from the limit switch to disengage it
  stepperY.move(50); // Move away from the switch to disengage
  while (stepperY.distanceToGo() > 0) {
    stepperY.run();
  }

  stepperY.setCurrentPosition(0); // When limit switch pressed set position to 0 steps
}

void home_z()
{
  stepperZ.setSpeed(HOMING_SPEED_Z);
  while (!digitalRead(LIMIT_SWITCH_Z_PIN)) {
    stepperZ.runSpeed();
  }
  stepperZ.stop();
  stepperZ.setCurrentPosition(-50); // When limit switch pressed set position to 0 steps
}

void home_all()
{
  home_z();
  move_z_to(Z_MAX);
  home_x();
  move_x_to(90);
  home_y();
  move_y_to(0);
}

void inverse_kinematics(double x, double y, double *theta1, double *theta2)
{
  double c = sqrt(x * x + y * y);
  //Serial.print("c: "); Serial.println(c);
  double b2 = atan2(x, y) * (180/3.1416);
  //Serial.print("B2: "); Serial.println(b2);
  double B = acos(((ARM_1_SIZE * ARM_1_SIZE) + (x * x) + (y * y) - (ARM_2_SIZE * ARM_2_SIZE)) / (2 * c * ARM_1_SIZE)) * (180/3.1416);
  //Serial.print("B: "); Serial.println(B);
  double b1 = B + b2;
  //Serial.print("B1: "); Serial.println(b1);
  *theta1 = 90 - b1;

  double C = acos(((ARM_1_SIZE * ARM_1_SIZE) + (ARM_2_SIZE * ARM_2_SIZE) - (c * c))/(2 * ARM_1_SIZE * ARM_2_SIZE)) * (180/3.1416);
  *theta2 = 180 - C;
  Serial.print("theta1: "); Serial.println(*theta1);
  Serial.print("theta2: "); Serial.println(*theta2);
  Serial.println();
}

void four_corner_test()
{
  double theta1, theta2;

  for (int i=0; i<4; i++) {
    inverse_kinematics(corner[i].x, corner[i].y, &theta1, &theta2);
    move_xy_to(theta1, theta2 - ((90-theta1)/PULLEY_RATIO));
    gripper_close();
    move_z_to(Z_MIN);
    gripper_open();
    delay(2000);
    gripper_close();
    move_z_to(Z_MAX);
  }
}

void up_down_test()
{
  stepperZ.setSpeed(MAX_SPEED_Z);
  long t0 = millis();
  move_z_to(Z_MAX);
  long t1 = millis();
  Serial.print("Up: ");
  Serial.println(t1-t0);
  delay(500);
  t0 = millis();
  move_z_to(Z_MIN);
  t1 = millis();
  Serial.print("Down: ");
  Serial.println(t1-t0);
  delay(500);
}

void curl_up()
{
  move_y_to(-10);
  move_x_to(STEP_ZERO_ANGLE(X));
}

void dump_eeprom()
{
  float angles[2];
  int address = EEPROM_START_ADDRESS;
  
  for (int i = 0; i < 64; i++) {
    // Read each angle for the square
    for (int j = 0; j < 2; j++) {
      EEPROM.get(address, angles[j]);
      address += sizeof(float);
    }
    // Print the angles
    Serial.print("Square ");
    Serial.print(i);
    Serial.print(": Angle 1 = ");
    Serial.print(angles[0], 6); // Print with precision of 6 digits
    Serial.print(", Angle 2 = ");
    Serial.println(angles[1], 6); // Print with precision of 6 digits
  }
}

void store_angles_for_square()
{
  int eeprom_address = EEPROM_START_ADDRESS + current_square_index * ANGLE_DATA_SIZE;
  EEPROM.put(eeprom_address, angle1);
  EEPROM.put(eeprom_address + sizeof(float), angle2);
  Serial.print("Stored angles for square ");
  Serial.println(current_square_index);
}

void adjust_angle(double* angle, double step)
{
  *angle += step;
  // Add bounds checking for angles if needed
}

int chess_notation_to_index(const String& notation)
{
  if (notation.length() != 2) return -1;

  // These arrays match the upside-down layout a.k.a robot's perspective
  const char* columns = "hgfedcba"; // Column identifiers, inverted
  const char* rows = "87654321"; // Row identifiers, standard but in reverse order

  char file = notation.charAt(0); // 'a' to 'h', but in reverse
  char rank = notation.charAt(1); // '1' to '8', but in reverse

  int file_index = -1;
  int rank_index = -1;

  // Find index in the reversed identifiers
  for (int i = 0; i < 8; i++) {
    if (columns[i] == file) file_index = i;
    if (rows[i] == rank) rank_index = i;
  }

  if (file_index == -1 || rank_index == -1) {
    return -1; // Notation is out of bounds
  }

  return rank_index * 8 + file_index;
}

String chess_index_to_notation(int index)
{
  // Handling it this way because the board is upside down from robot's perspective
  const char* columns = "hgfedcba"; // Inverted order
  const char* rows = "12345678"; // Standard order, but will be accessed in reverse

  // Calculate file (column) and rank (row) from the currentSquareIndex
  int file_index = index % 8; // Column
  int rank_index = 7 - (index / 8); // Row, inverted

  // Construct the square notation
  char notation[3] = {columns[file_index], rows[rank_index], '\0'}; // Null-terminated string
  return notation;
}

void xy_lookup(uint8_t index, double *x, double *y)
{
  // Calculate row and column from index
  int row = current_square_index / CHESS_COLS;
  int col = current_square_index % CHESS_COLS;

  // Calculate fractions along each axis
  double x_fraction = (double)col / (CHESS_COLS - 1);
  double y_fraction = (double)row / (CHESS_ROWS - 1);

  // Interpolate x and y positions
  double x_left = LOWER_LEFT_CORNER_X * (1 - y_fraction) + UPPER_LEFT_CORNER_X * y_fraction;
  double x_right = LOWER_RIGHT_CORNER_X * (1 - y_fraction) + UPPER_RIGHT_CORNER_X * y_fraction;
  *x = x_left * (1 - x_fraction) + x_right * x_fraction;

  double y_lower = LOWER_LEFT_CORNER_Y * (1 - x_fraction) + LOWER_RIGHT_CORNER_Y * x_fraction;
  double y_upper = UPPER_LEFT_CORNER_Y * (1 - x_fraction) + UPPER_RIGHT_CORNER_Y * x_fraction;
  *y = y_lower * (1 - y_fraction) + y_upper * y_fraction;
}

uint32_t adjusted_z_min(piece_type_t piece, uint8_t row)
{
  // TODO The nomenclature for 'row' is reverse in robotic arm. Need to make it consistent with chess board.
  uint32_t zmin = z_min[piece] - ((CHESS_ROWS - 1 - row) * 25);
  if (piece == PIECE_TYPE_KNIGHT) {
    zmin = z_min[piece] - ((CHESS_ROWS - 1 - row) * 14);
  }

  return zmin;
}

piece_type_t piece_type(char p)
{
  piece_type_t piece = PIECE_TYPE_PAWN;  // Default to pawn
  switch (p) {
    case 'p': {
      piece = PIECE_TYPE_PAWN;
    } break;
    case 'r': {
      piece = PIECE_TYPE_ROOK;
    } break;
    case 'n': {
      piece = PIECE_TYPE_KNIGHT;
    } break;
    case 'b': {
      piece = PIECE_TYPE_BISHOP;
    } break;
    case 'q': {
      piece = PIECE_TYPE_QUEEN;
    } break;
    case 'k': {
      piece = PIECE_TYPE_KING;
    } break;
  }

  return piece;
}

void prompt_next_square(bool eeprom)
{
  String notation = chess_index_to_notation(current_square_index);
  Serial.print("Please jog to the square ");
  Serial.print(notation);
  Serial.println(" for adjustment. Press 's' to save when done.");
  calibrate_square(notation, eeprom);
}

bool handle_input(char input, bool eeprom)
{
  bool adjust = false;
  switch(input) {
    case 'L': {
      adjust_angle(&angle1, CAL_LARGE_STEP);
      adjust = true;
    } break;
    case 'R': {
      adjust_angle(&angle1, -CAL_LARGE_STEP);
      adjust = true;
    } break;
    case 'l': {
      adjust_angle(&angle1, CAL_SMALL_STEP);
      adjust = true;
    } break;
    case 'r': {
      adjust_angle(&angle1, -CAL_SMALL_STEP);
      adjust = true;
    } break;
    case 'D': {
      adjust_angle(&angle2, CAL_LARGE_STEP);
      adjust = true;
    } break;
    case 'U': {
      adjust_angle(&angle2, -CAL_LARGE_STEP);
      adjust = true;
    } break;
    case 'd': {
      adjust_angle(&angle2, CAL_SMALL_STEP);
      adjust = true;
    } break;
    case 'u': {
      adjust_angle(&angle2, -CAL_SMALL_STEP);
      adjust = true;
    } break;
    case 's': {
      store_angles_for_square();
    } break;
    case 'n': {
      current_square_index++;
      if (current_square_index >= 64) {
        Serial.println("Calibration complete!");
        current_square_index = 0; // Reset or end calibration
      } else {
        prompt_next_square(eeprom);
      }
    } break;
    case 'q': {
      Serial.println("Exiting calibration");
      dump_eeprom();
      return false;
    } break;
    case 'z': {
      home_z();
      move_z_to(Z_MIN);
    } break;
    case 'g': {
      gripper_open();
      delay(2000);
      gripper_close();
    }
  }
  
  if (adjust) {
    move_xy_to(angle1, angle2 - ((90-angle1)/PULLEY_RATIO));
    // Output current angles
    Serial.print("Angle1: ");
    Serial.print(angle1);
    Serial.print(", Angle2: ");
    Serial.println(angle2);
  }

  return true;
}

void calibrate_square(String sq, bool eeprom)
{
  current_square_index = chess_notation_to_index(sq);

  if (current_square_index == -1) {
    Serial.println(F("Invalid move notation."));
    return;
  }

  // Get the angles from EEPROM or IK
  if (eeprom) {
    double angles[2];
    int address = EEPROM_START_ADDRESS + current_square_index * ANGLE_DATA_SIZE;

    EEPROM.get(address, angles);
    Serial.print(F("Square index: "));
    Serial.println(current_square_index);
    Serial.print(F("Square angles: Theta1 = "));
    Serial.print(angles[0], 6);
    Serial.print(F(", Theta2 = "));
    Serial.println(angles[1], 6);
    angle1 = angles[0];
    angle2 = angles[1];
  } else {
    double x, y, theta1, theta2;
    xy_lookup(current_square_index, &x, &y);
    inverse_kinematics(x, y, &theta1, &theta2);
    angle1 = theta1;
    angle2 = theta2;
  }

  stepperX.setMaxSpeed(REDUCED_SPEED_X);
  stepperY.setMaxSpeed(REDUCED_SPEED_Y);
  move_xy_to(angle1, angle2 - ((90-angle1)/PULLEY_RATIO));
  move_z_to(adjusted_z_min(PIECE_TYPE_QUEEN, current_square_index / CHESS_ROWS));
  while (true) {
    if (Serial.available() > 0) {
      char input = Serial.read();
      if (!(handle_input(input, eeprom))) {
        break;
      }
    }
  }
  stepperX.setMaxSpeed(MAX_SPEED_X);
  stepperY.setMaxSpeed(MAX_SPEED_Y);
}

void calibrate_board(bool eeprom)
{
  gripper_open();
  delay(2000);
  gripper_close();

  current_square_index = 0;
  prompt_next_square(eeprom);
}

String get_square_input()
{
  Serial.println(F("Enter chess square (e.g., e2):"));
  String sq = Serial.readStringUntil('\n');
  while (sq.length() != 2) {
    sq = Serial.readStringUntil('\n');
  }
  sq.trim(); // Remove any whitespace
  Serial.println(sq);

  return sq;
}

void move_to_square(String sq)
{
  int index = chess_notation_to_index(sq);
  if (index == -1) {
    Serial.println(F("Invalid move notation."));
    return;
  }

  double angles[2];
  int address = EEPROM_START_ADDRESS + index * ANGLE_DATA_SIZE;
  EEPROM.get(address, angles);
  //Serial.print(F("Source square angles: Theta1 = "));
  //Serial.print(angles[0], 6);
  //Serial.print(F(", Theta2 = "));
  //Serial.println(angles[1], 6);
  move_xy_to(angles[0], angles[1] - ((90-angles[0])/PULLEY_RATIO));
}

void test_square(String sq)
{
  Serial.print("Square: "); Serial.println(sq);
  move_to_square(sq);
  move_z_to(Z_MIN);
  delay(2000);
  move_z_to(Z_MAX);
}

String get_move_input()
{
  // move format: <start sq><piece1><end sq><piece2>
  // If piece2 is 'x' or differs from piece1, then it is taking opponent's piece
  // else it is just a movement
  Serial.println(F("Enter chess move (e.g., e2pe4p):"));

  String move = Serial.readStringUntil('\n');
  while (move.length() != 6) {
    move = Serial.readStringUntil('\n');
  }
  move.trim(); // Remove any whitespace
  Serial.println(move);
  return move;
}

void idle_move()
{
    static uint8_t i = 0;
    switch (i) {
        case 0: {
            move_to_square("d4");
        } break;
        case 1: {
            move_to_square("e4");
        } break;
        case 2: {
            move_to_square("e5");
        } break;
        case 3: {
            move_to_square("d5");
        } break;
    }
    i = (i + 1) % 4;
}

bool are_squares_adjacent(int start_index, int end_index) {
  // Calculate row and column for start and end indices
  int start_row = start_index / CHESS_COLS;
  int start_col = start_index % CHESS_COLS;
  int end_row = end_index / CHESS_COLS;
  int end_col = end_index % CHESS_COLS;

  // Check if the squares are adjacent
  if ((abs(start_row - end_row) <= 1) && (abs(start_col - end_col) <= 1)) {
    return true; // Squares are adjacent
  }
  return false; // Squares are not adjacent
}

void execute_move(String move)
{
  // Move type?
  if ((move[4] != move[5]) || (move[5] == 'x')) {
    // Piece being taken
    String sq = move.substring(2, 4);
    move_to_square(sq);
    gripper_open();
    if (move[5] == 'x') {
      move_z_to(adjusted_z_min(piece_type(move[4]), chess_notation_to_index(sq) / CHESS_ROWS));
    } else {
      move_z_to(adjusted_z_min(piece_type(move[5]), chess_notation_to_index(sq) / CHESS_ROWS));
    }
    gripper_close();
    delay(500);
    move_z_to(Z_MAX);
    curl_up();
    gripper_open();
    delay(500);
    gripper_close();
  }

  // Simple piece movement
  int start_index = chess_notation_to_index(move.substring(0, 2));
  int end_index = chess_notation_to_index(move.substring(2, 4));
  bool is_adjacent = are_squares_adjacent(start_index, end_index);
  String sq = move.substring(0, 2);
  move_to_square(sq);
  gripper_open();
  move_z_to(adjusted_z_min(piece_type(move[4]), start_index / CHESS_ROWS));
  gripper_close();
  delay(500);
  if (is_adjacent) {
    move_z_to(adjusted_z_min(piece_type(move[4]), start_index / CHESS_ROWS) + Z_LOW);
  } else {
    move_z_to(Z_MAX);
  }
  sq = move.substring(2, 4);
  move_to_square(sq);
  move_z_to(adjusted_z_min(piece_type(move[4]), end_index / CHESS_ROWS));
  gripper_open();
  delay(500);
  move_z_to(Z_MAX);
  gripper_close();
  chessboard.println("done");
  curl_up();
}

void test_pawns_march()
{
  for (int row = 7; row > 0; row--) { // Start from row closest to the arm (a8 to h8) and move towards a1 to h1
    for (int col = 0; col < 8; col++) { // Iterate through all columns (pawns) in the row
      String sq = chess_index_to_notation(col + row * 8); // Get the starting square notation
      move_to_square(sq);
      gripper_open();
      move_z_to(adjusted_z_min(PIECE_TYPE_PAWN, row));
      gripper_close();
      delay(500);
      move_z_to(Z_MAX);

      sq = chess_index_to_notation(col + (row - 1) * 8); // Get the ending square notation
      move_to_square(sq);
      move_z_to(adjusted_z_min(PIECE_TYPE_PAWN, row - 1));
      gripper_open();
      delay(500);
      move_z_to(Z_MAX);
      gripper_close();
    }
  }
}

void arm_run()
{
  static String inputBuffer = ""; // Buffer to hold incoming characters
  static bool idle_move_start = false;
  static bool skip_first = true;

  // Check if data is available to read from chessboard
  while (chessboard.available()) {
    char c = chessboard.read(); // Read a character
    Serial.write(c); // Echo the character to the main serial port for debugging

    if (c == 'i') {
      // Player done with their move. Wake up!
      idle_move_start = true;
      animation_id = WAKE_UP;
    } else if (c == 'j') {
      home_z();
      move_z_to(Z_MAX);
    } else if (c == 'z') {
      home_all();
      curl_up();
      skip_first = true;
    } else if (c == 's') {
      // Start of player's turn. Doze off if not received 'i' in 10 sec
      if (!skip_first) {
        animation_id = DOZE_OFF;
      }
      skip_first = false;
    } else if (c == '\n' || c == '\r') {
      // End of line character, ignore it but reset if in buffer
      if (inputBuffer.length() != 0) {
        Serial.println(F("Incomplete move, resetting buffer."));
        inputBuffer = ""; // Clear the buffer if we had partial data
      }
    } else {
      inputBuffer += c; // Add character to buffer

      // Check if we have a full move in the buffer
      if (inputBuffer.length() == 6) {
        idle_move_start = false;
        animation_id = RESET_POSE;
        Serial.println("\nInitiating move"); // Move to a new line
        execute_move(inputBuffer); // Process the move
        inputBuffer = ""; // Clear the buffer for the next move
      }
    }
  }

  if (idle_move_start) {
    idle_move();
  }
}

void arm_init()
{
  chessboard.begin(9600);
  static_assert(GRIPPER_OPEN_ANGLE < GRIPPER_CLOSE_ANGLE, "Invalid gripper angle range");

  gripper.attach(GRIPPER_SERVO_PIN);

  stepperX.setMaxSpeed(MAX_SPEED_X);
  stepperX.setAcceleration(ACCELERATION_X);
  stepperX.setPinsInverted(false, false, true);
  stepperX.setEnablePin(STEPPER_ENABLE_PIN);

  stepperY.setMaxSpeed(MAX_SPEED_Y);
  stepperY.setAcceleration(ACCELERATION_Y);
  stepperY.setPinsInverted(false, false, true);
  stepperY.setEnablePin(STEPPER_ENABLE_PIN);

  stepperZ.setMaxSpeed(MAX_SPEED_Z);
  stepperZ.setAcceleration(ACCELERATION_Z);
  stepperZ.setPinsInverted(false, false, true);
  stepperZ.setEnablePin(STEPPER_ENABLE_PIN);

  multistepper.addStepper(stepperX);
  multistepper.addStepper(stepperY);

  pinMode(LIMIT_SWITCH_X_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Y_PIN, INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_Z_PIN, INPUT_PULLUP);

  gripper_open();
  delay(1000);
  gripper_close();

  home_all();
  curl_up();
  animation_id = RESET_POSE;
  Serial.println("Robotic Arm initialized");
}