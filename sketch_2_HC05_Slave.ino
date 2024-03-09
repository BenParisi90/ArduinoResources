#include <SoftwareSerial.h>
SoftwareSerial BTSerial(9, 8);

String message = "";
int ledPin = 5;

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  BTSerial.begin(38400);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(ledPin, LOW);
  while(BTSerial.available() > 0)
  {
    char data = (char) BTSerial.read();
    if(data != ';')
    {
      message += data;
    }
    else
    {
      Serial.print(message);
      message = "";
    }
    digitalWrite(ledPin, HIGH);
  }
}
