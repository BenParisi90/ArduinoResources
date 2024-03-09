#include <Joystick.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false); 

void setup() {
  // put your setup code here, to run once:
  Joystick.begin();
  Joystick.setXAxisRange(1024, 0);
  Joystick.setYAxisRange(-1, 1);
  Joystick.setYAxis(0);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int x_input = analogRead(A2);
  Joystick.setXAxis(x_input);
}