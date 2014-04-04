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

volatile long p = 0;                       // The current position of the robot. DO NOT ACCESS DIRECTLY
                                           // use getPosTicks intead.
volatile long pt = 0;                      // The last time the we got a reed switch tick for the position
                                           // of the robot.

State state;                               // The current state of the darkcart robot.

static const int POS_ENDSTOP = 2;          // Endstop which acts as our reset button.
static const int POS_TICKER = 3;           // Reed switch which acts our a low res rotary encoder.
static const int STEPPER_ON = 8;           // The enable pin for the stepper motor.
static const int STEPPER_DIR = 4;          // The pin for controlling the direction of the stepper.
static const int STEPPER_STEP = 5;         // The pin for controlling the steps on the stepper.
static const int ACTUATOR_UP = 6;          // The pin for controlling the direction of the actuator.
static const int ACTUATOR_DOWN = 7;        // The on/off pin for turning the actuator on or off.
static const int ACTUATOR_POT = 0;         // The pin for measuring the current position of the actuator.
static const long STEP_DELAY_SHOW = 80;
static const long STEP_DELAY_INIT = 80;
static const long MIN_TICK_TIME = 800;     // The minimum time to allow between reed switch ticks. Increase if switch bounding is a problem.
static const int ACTUATOR_LOWER = 27;      // Minimum value read from the actutor pot
static const int ACTUATOR_UPPER = 937;     // Maximum value read from the actuator pot.
static const float ACTUATOR_THRESH = 0.02; // Movement threshold for the actuator pot.

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
 * Callback method for POS_TICKER interrupt. Updates the total number of magnets the robot has
 * passed over since it was turned on.
 */
void updatePosTicks() {
  if ((millis() - pt) > MIN_TICK_TIME) {
    p++;
    pt = millis();
  }
}

/**
 * Returns the total number of position ticks that have counted since the robot was turned on.
 */
long getPosTicks() {
  cli();            // Temporarily disable interrupts so that we can read from p.
  long result = p;
  sei();            // Renable interrupts so that p gets updated in the background.
  
  return result;
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
  delayMicroseconds(stepDelay);
}

float currentHeight() {
  return ((analogRead(ACTUATOR_POT) - ACTUATOR_LOWER) / (float) (ACTUATOR_UPPER - ACTUATOR_LOWER));
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
      current_state.target_position = (int) command.argument;
      break;

    case 'h':
      current_state.target_height = (int) command.argument;
      break;

    default:
      break;      // No new command - Don't update anything.
  }

  // TODO: endstop protection. If we go crazy and miss a position and hit an endstop. Wind back to the start.
  if (current_state.target_position - current_state.current_position > 0) {
    while (current_state.target_position - current_state.current_position > 0) {
        long new_tick_count = getPosTicks();
        long dp = new_tick_count - current_state.tick_count;
        current_state.current_position += dp;
        current_state.tick_count = new_tick_count;
        stepForward(STEP_DELAY_SHOW);
    }

  } else if (current_state.target_position - current_state.current_position < 0) {
    while (current_state.target_position - current_state.current_position < 0) {
        long new_tick_count = getPosTicks();
        long dp = new_tick_count - current_state.tick_count;
        current_state.current_position -= dp;
        current_state.tick_count = new_tick_count;
        stepBackward(STEP_DELAY_SHOW);
    }

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
  Serial.begin(9600);

  // Move the robot to its starting position when we are powered on.
  pinMode(POS_ENDSTOP, INPUT);
  pinMode(POS_TICKER, INPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(ACTUATOR_UP, OUTPUT);
  pinMode(ACTUATOR_DOWN, OUTPUT);
  pinMode(13, OUTPUT);

  // Wind the stepper to the starting position.
  while (digitalRead(POS_ENDSTOP) == HIGH) {
    stepBackward(STEP_DELAY_INIT);
  }

  // Raise the actuator to the starting position.
  digitalWrite(ACTUATOR_UP, HIGH);
  while (currentHeight() > 0.0) {
  }
  digitalWrite(ACTUATOR_UP, LOW);

  attachInterrupt(1, updatePosTicks, RISING);  // Sets pin 2 on the Arduino as a RISING interrupt.

  // Initalise the state of the robot.
  state.tick_count = getPosTicks();
  state.current_position = 0;
  state.target_position = 0;
  state.target_height = 0;
}

/**
 * Main Arduino loop.
 */
void loop() {
  state = update(state, ReadCommand());
}
