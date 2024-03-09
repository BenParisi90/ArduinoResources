#include <SoftwareSerial.h>
SoftwareSerial BTSerial(9, 8);

int touchPin = 5;
bool touched = false;

void setup() {
  pinMode(touchPin, INPUT);
  BTSerial.begin(38400);
  Serial.begin(9600);
}

void loop() {
  if(!touched && digitalRead(touchPin) == HIGH)
  {
    BTSerial.println("All work and no play makes Jack a dull boy;");
    Serial.println(1);
    touched = true;
  }
  else if(touched && digitalRead(touchPin) == LOW)
  {
    touched = false;
  }
}