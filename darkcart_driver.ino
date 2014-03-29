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
typedef struct {
  char instruction;          // The instruction that came in over the serial connection.
  float argument;            // The argument that was supplied with the instruction.
} Command;

typedef struct State_struct {
  int target_position;      // The desired position on the linear rail for the robot.  
  float target_height;      // The desired height of the curtain for the robot.
} State;

State state;                // The current state of the darkcart robot.

static const int ACTUATOR_POT = 0

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



  // Work out the different between the target height and the current height and apply any
  // movement co compensate.
  float current_height = (analogRead(ACTUATOR_POT) / 255.0);
  float dh = current_height - current_state.target_height;
  if (dh < 0.001) {
    // TODO: Set the actuator direction and turn it on.
  } else if (dh > 0.001) {
    // TODO: Set the actuator in the oposite direction and turn it on.
  }

  return current_state;
}

/**
 * Arduino initalisation.
 */
void setup() {
  Serial.begin(9600);
}

/**
 * Main Arduino loop.
 */
void loop() {
  state = update(state, ReadCommand());
}




