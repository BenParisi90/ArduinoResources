#include <Arduino.h>
#include <SoftwareSerial.h>

const byte rxPin = 9;
const byte txPin = 8;

SoftwareSerial BTSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  BTSerial.begin(9600);
  Serial.begin(9600);
}

String messageBuffer = "";
String message = "";

void loop() {
  // put your main code here, to run repeatedly:
  while(BTSerial.available() > 0)
  {
    char data = (char) BTSerial.read();
    messageBuffer += data;
    if(data == ';')
    {
      message = messageBuffer;
      messageBuffer = "";
      Serial.print(message);
      message = "You sent " + message;
      BTSerial.print(message);
    }
  }
}
