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

volatile long p = 0;              // The current position of the robot. DO NOT ACCESS DIRECTLY
                                  // use getPosTicks intead.

State state;                      // The current state of the darkcart robot.

static const int POS_ENDSTOP = 2; // Endstop which acts as our reset button.
static const int POS_TICKER = 3;  // Reed switch which acts our a low res rotary encoder. 
static const int STEPPER_DIR = 4; // The pin for controlling the direction of the stepper.
static const int STEPPER_STEP = 5;// The pin for controlling the steps on the stepper.
static const int ACTUATOR_DIR = 6;// The pin for controlling the direction of the actuator.
static const int ACTUATOR_ON = 7; // The on/off pin for turning the actuator on or off.
static const int ACTUATOR_POT = 0;// The pin for measuring the current position of the actuator.

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
  p++;
}

/**
 * Returns the total number of position ticks that have counted since the robot was turned on.
 */
long getPosTicks() {
  cli();
  long result = p;
  sei();
  
  return result;
}

/**
 * Moves the robot backwards along the linear rail (i.e. alters position).
 */
void moveBackward() {
  // TODO: Hook into the easy driver for the stepper.
}

/**
 * Moves the robot forwards along the linear rail (i.e. alters position).
 */
void moveForward() {
  // TODO: Hook into the easy driver for the stepper.
}

State update(State current_state, Command command) {
  switch (command.instruction) {
    case 'p':
      current_state.target_position = (int) command.argument;
      break;

    case 'h':
      current_state.target_height = (int) command.argument;
      break;

    default:
      break;      // No new command - don't update anything.
  }

  // Work out the difference between the target position and the current position and apply 
  // any movement to compensate.
  long new_tick_count = getPosTicks();
  long dt = new_tick_count - current_state.tick_count;
  if (current_state.target_position - current_state.current_position > 0) {
    moveForward();
    current_state.current_position += dt;    

  } else if (current_state.target_position - current_state.current_position < 0) {
    moveBackward();
    current_state.current_position -= dt;

  }  
  current_state.tick_count = new_tick_count;
  

  // Work out the different between the target height and the current height and apply any
  // movement co compensate.
  float current_height = (analogRead(ACTUATOR_POT) / 255.0);
  float dh = current_height - current_state.target_height;
  if (dh < 0.001) {
    // TODO: Set the actuator direction and turn it on.
  } else if (dh > 0.001) {
    // TODO: Set the actuator in the oposite direction and turn it on.
  } else {
    // TODO: Turn the actuator off. 
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
  while (digitalRead(POS_ENDSTOP) == LOW) {
    moveBackward();
  }

  attachInterrupt(1, updatePosTicks, RISING);  // Sets pin 2 on the Arduino as a RISING interrupt.

  // Initalise the state of the robot.
  state.tick_count = 0;
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
