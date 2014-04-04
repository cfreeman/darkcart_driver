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
static const int STEPPER_ON = 8;  // The enable pin for the stepper motor.
static const int STEPPER_DIR = 4; // The pin for controlling the direction of the stepper.
static const int STEPPER_STEP = 5;// The pin for controlling the steps on the stepper.
static const int ACTUATOR_UP = 6;// The pin for controlling the direction of the actuator.
static const int ACTUATOR_DOWN = 7; // The on/off pin for turning the actuator on or off.
static const int ACTUATOR_POT = 0;// The pin for measuring the current position of the actuator.
static const long STEP_DELAY_SHOW = 80;
static const long STEP_DELAY_INIT = 80;

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
  // TODO: Debounce ticks on the reed switch. - Don't allow ticks to occur faster than half a second.
  p++;
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
      break;      // No new command - don't update anything.
  }

  // Work out the difference between the target position and the current position and apply 
  // any movement to compensate.
  long new_tick_count = getPosTicks();
  long dp = new_tick_count - current_state.tick_count;
  if (current_state.target_position - current_state.current_position > 0) {
    stepForward(STEP_DELAY_SHOW);
    current_state.current_position += dp;    

  } else if (current_state.target_position - current_state.current_position < 0) {
    stepBackward(STEP_DELAY_SHOW);
    current_state.current_position -= dp;

  }
  // TODO: endstop protection. If we go crazy and miss a position and hit an endstop. Wind back to the start.
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
  pinMode(POS_TICKER, INPUT);
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(ACTUATOR_UP, OUTPUT);
  pinMode(ACTUATOR_DOWN, OUTPUT);
  pinMode(13, OUTPUT);

//  Enable stepper initalisation.
//  while (digitalRead(POS_ENDSTOP) == HIGH) {
//    stepBackward(STEP_DELAY_INIT);
//  }

//  digitalWrite(ACTUATOR_DOWN, HIGH); // Left switch
  digitalWrite(ACTUATOR_UP, HIGH);

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
