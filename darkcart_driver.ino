/*
 * Copyright (c) Clinton Freeman 2014
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
 * associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "darkcart_driver.h"
                                           // of the robot.
unsigned long t = 0;

State state;                               // The current state of the darkcart robot.

static const int POS_1 = 9;
static const int POS_2 = 10;
static const int POS_3 = 11;
static const int POS_4 = 12;
static const int POS_5 = 13;

static const int POS_ENDSTOP = 2;         // Endstop which acts as our reset button.
static const int POS_FARSTOP = 3;          // Farstop saftey kill - just in case everything goes crazy.
static const int POS_TICKER = 3;           // Reed switch which acts our a low res rotary encoder.
static const int STEPPER_ON = 8;           // The enable pin for the stepper motor.
static const int STEPPER_DIR = 4;          // The pin for controlling the direction of the stepper.
static const int STEPPER_STEP = 5;         // The pin for controlling the steps on the stepper.
static const int ACTUATOR_UP = 6;          // The pin for controlling the direction of the actuator.
static const int ACTUATOR_DOWN = 7;        // The on/off pin for turning the actuator on or off.
static const int ACTUATOR_POT = 0;         // The pin for measuring the current position of the actuator.
static const int STEP_DELAY_INIT = 80;
static const long STEP_DELAY_SHOW = 250;
static const int ACTUATOR_LOWER = 27;      // Minimum value read from the actutor pot
static const int ACTUATOR_UPPER = 937;     // Maximum value read from the actuator pot.
static const float ACTUATOR_THRESH = 0.1;  // Movement threshold for the actuator pot.

/**
 * ReadCommand sucks down the lastest command from the serial port, returns
 * {'*', 0.0} if no new command is available.
 */
Command ReadCommand() {
  // Not enough bytes for a command, return an empty command.
  if (Serial.available() < 5) {
    return (Command) {'*', 0.0};
  }

  union {
    char b[4];
    float f;
  } ufloat;

  // Read the command identifier and argument from the serial port.
  char c = Serial.read();
  Serial.readBytes(ufloat.b, 4);

  return (Command) {c, ufloat.f};
}

/**
 * Moves the robot backwards along the linear rail (i.e. alters position).
 */
void stepBackward(long stepDelay) {
  digitalWrite(STEPPER_DIR, HIGH);
  step(stepDelay);
}

/**
 * Moves the robot forwards along the linear rail (i.e. alters position).
 */
void stepForward(long stepDelay) {
  digitalWrite(STEPPER_DIR, LOW);
  step(stepDelay);
}

void step(long stepDelay) {
  digitalWrite(STEPPER_STEP, LOW);
  delayMicroseconds(stepDelay);
  digitalWrite(STEPPER_STEP, HIGH);

  unsigned long dt = micros() - t;
  if (dt < STEP_DELAY_SHOW) {
    delayMicroseconds(STEP_DELAY_SHOW - dt);
  }

  t = micros();
}

float currentHeight() {
  return ((analogRead(ACTUATOR_POT) - ACTUATOR_LOWER) / (float) (ACTUATOR_UPPER - ACTUATOR_LOWER));
}

int currentPosition(State current_state) {
  if (digitalRead(POS_1) == HIGH) {
    return 1;
  } else if (digitalRead(POS_2) == HIGH) {
    return 2;
  } else if (digitalRead(POS_3) == HIGH) {
    return 3;
  } else if (digitalRead(POS_4) == HIGH) {
    return 4;
  }  else if (digitalRead(POS_5) == HIGH) {
    return 5;
  } else {
    // Currently travelling between positions -- use the last known position.
    return current_state.current_position;
  }
}

/**
 * Updates the current state of the robot based on incoming commands and what it senses in the environment.
 *
 * @param current_state The current state of the robot that is to be altered.
 * @param command any incoming commands that instruct the robot.
 */
State update(State current_state, Command command) {
  switch (command.instruction) {
    case 'p':
      // Make sure the position is valid (between 1 and 5).
      if ((int) command.argument < 1) {
        current_state.target_position = 1;
      } else if ((int) command.argument > 5) {
        current_state.target_position = 5;
      } else {
        current_state.target_position = (int) command.argument;
      }
      break;

    case 'h':
      current_state.target_height = command.argument;
      break;

    default:
      break;      // No new command - Don't update anything.
  }

  current_state.current_position = currentPosition(current_state);
  if (current_state.target_position - current_state.current_position > 0 &&
      digitalRead(POS_FARSTOP) == HIGH) {
    stepForward(STEP_DELAY_SHOW);

  } else if (current_state.target_position - current_state.current_position < 0 &&
             digitalRead(POS_ENDSTOP) == HIGH) {
    stepBackward(STEP_DELAY_SHOW);

  }

  // Work out the different between the target height and the current height and apply any
  // movement co compensate.
  float dh = currentHeight() - current_state.target_height;

  if (dh < ACTUATOR_THRESH && dh > -ACTUATOR_THRESH ) {
      digitalWrite(ACTUATOR_UP, LOW);
      digitalWrite(ACTUATOR_DOWN, LOW);

  } else if (dh < 0.0) {
    digitalWrite(ACTUATOR_UP, LOW);
    digitalWrite(ACTUATOR_DOWN, HIGH);

  } else if (dh > 0.0) {
    digitalWrite(ACTUATOR_DOWN, LOW);
    digitalWrite(ACTUATOR_UP, HIGH);
  }

  return current_state;
}

/**
 * Arduino initalisation.
 */
void setup() {
  // Move the robot to its starting position when we are powered on.
  pinMode(POS_ENDSTOP, INPUT);
  pinMode(POS_TICKER, INPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(ACTUATOR_UP, OUTPUT);
  pinMode(ACTUATOR_DOWN, OUTPUT);
  pinMode(POS_1, INPUT);
  pinMode(POS_2, INPUT);
  pinMode(POS_3, INPUT);
  pinMode(POS_4, INPUT);
  pinMode(POS_5, INPUT);

  // Wind the stepper to the starting position.
  while (digitalRead(POS_ENDSTOP) == HIGH) {
    stepBackward(STEP_DELAY_INIT);
    delayMicroseconds(STEP_DELAY_INIT);
  }

  // Raise the actuator to the starting position.
  digitalWrite(ACTUATOR_UP, HIGH);
  while (currentHeight() > 0.0) {
  }
  digitalWrite(ACTUATOR_UP, LOW);

  // Initalise the state of the robot.
  state.current_position = 0;
  state.target_position = 0;
  state.target_height = 0.0;
  Serial.begin(9600);
}

/**
 * Main Arduino loop.
 */
void loop() {
  state = update(state, ReadCommand());
}
