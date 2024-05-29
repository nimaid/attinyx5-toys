/*
 * Capacitive touch example, adapted by Eva Snyder from
 * http://www.instructables.com/id/Turn-a-pencil-drawing-into-a-capacitive-sensor-for/
 *
 * Found on Olena Kokhan's blog: https://kokhana89.github.io/Fablab/touchsensor.html
 * 
 * Futher adapted by Ella Jameson to make the ATtinyX5 a general purpose cap sense chip.
 * More info on the method: https://github.com/wagiminator/ATtiny13-TinyTouchLight
 *
 * Requires the "CapacitiveSensor" library by Paul Bagder to be installed.
 * 
 * Standard Application Schematic:
 *       ┌───U───┐             ┌─────┐
 *     ──┤ ○     ├── VCC       │Touch│
 *  Tr ─3┤       ├2──────────┬─┤ Pad │
 *   P ─4┤       ├1───────~~~┘ └─────┘
 * GND ──┤       ├0─ Tp   1MΩ (approx.)
 *       └───────┘
 * 
 * Schematic Key:
 * ┌──────┬───────────────────┐
 * │ Name │      Meaning      │
 * ├──────┼───────────────────┤
 * │  P   │      Pressed      │
 * │  Tp  │ Toggle (pressed)  │
 * │  Tr  │ Toggle (released) │
 * └──────┴───────────────────┘
 */

#include <CapacitiveSensor.h>

// This is how high the sensor needs to read in order
//  to trigger a touch.  You'll find this number
//  by trial and error, or you could take readings at 
//  the start of the program to dynamically calculate this.
// If this is not sensitive enough, try a resistor with more ohms.
// Range: 0 - 1023
#define TOUCH_SENSITIVITY 600

// The time to delay() between each update loop
#define LOOP_DELAY 10

// Pins for outputs
#define PRESSED_PIN 4
#define TOGGLE_PRESS_PIN 0
#define TOGGLE_RELEASE_PIN 3

// Pin to connect to your conductive sensor 
// (paperclip, conductive paint/fabric/thread, wire)
#define CS_SENSE_PIN 2

// Pin on the other side of the sense resistor
#define CS_DRIVE_PIN 1


CapacitiveSensor cs = CapacitiveSensor(CS_DRIVE_PIN, CS_SENSE_PIN);

bool isPressed = false;
bool toggleRisingState = LOW;
bool toggleFallingState = LOW;

long csValue;


void setup(){
  // Set up the output pins
  pinMode(PRESSED_PIN, OUTPUT);
  digitalWrite(PRESSED_PIN, LOW);

  pinMode(TOGGLE_PRESS_PIN, OUTPUT);
  digitalWrite(TOGGLE_PRESS_PIN, LOW);

  pinMode(TOGGLE_RELEASE_PIN, OUTPUT);
  digitalWrite(TOGGLE_RELEASE_PIN, LOW);
  
  // Setup the capacitive touch sensor
  cs.set_CS_AutocaL_Millis(0xFFFFFFFF);  // turn off autocalibrate on channel 1 - just as an example
}

void loop(){
  csValue = cs.capacitiveSensor(30);

  if (csValue > TOUCH_SENSITIVITY) {
    if(!isPressed) {
      isPressed = true;
      digitalWrite(PRESSED_PIN, HIGH);

      toggleRisingState = !toggleRisingState;
      digitalWrite(TOGGLE_PRESS_PIN, toggleRisingState);
    }
  }
  else {
    if(isPressed) {
      isPressed = false;
      digitalWrite(PRESSED_PIN, LOW);

      toggleFallingState = !toggleFallingState;
      digitalWrite(TOGGLE_RELEASE_PIN, toggleFallingState);
    }
  }
  
  delay(LOOP_DELAY); 
}