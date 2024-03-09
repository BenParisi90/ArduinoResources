#include <Keyboard.h>
#include <Joystick.h>

const int leftXPin = A1;
const int rightXPin = A2;
bool gasOn = false;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  0, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false); 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  pinMode(leftXPin, INPUT);
  pinMode(rightXPin, INPUT);

  Keyboard.begin();

  Joystick.begin();
  Joystick.setXAxisRange(-1024, 1024);
  Joystick.setYAxisRange(-1, 1);
  Joystick.setYAxis(0);
}

void loop() 
{

  int leftXData = analogRead(leftXPin);
  int rightXData = analogRead(rightXPin);

  if(!gasOn && (leftXData < 500 || rightXData < 500))
  {
    Keyboard.press('W');
    gasOn = true;
  }
  else if(gasOn && (leftXData > 500 && rightXData > 500))
  {
    Keyboard.release('W');
    gasOn = false;
  }

  Joystick.setXAxis(rightXData - leftXData);
}
