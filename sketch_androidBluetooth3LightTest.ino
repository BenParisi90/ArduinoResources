#include <Arduino.h>
#include <SoftwareSerial.h>

const byte rxPin = 9;
const byte txPin = 8;

int lightPin_A = 2;
int lightPin_B = 3;
int lightPin_C = 4;

bool pin_A = false;
bool pin_B = false;
bool pin_C = false;

SoftwareSerial BTSerial(rxPin, txPin);

void setup() {
  // put your setup code here, to run once:
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  pinMode(lightPin_A, OUTPUT);
  pinMode(lightPin_B, OUTPUT);
  pinMode(lightPin_C, OUTPUT);

  digitalWrite(lightPin_A, pin_A);
  digitalWrite(lightPin_B, pin_B);
  digitalWrite(lightPin_C, pin_C);
  
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
      
      Serial.println(message);
      if(message == "Btn_A;")
      {
        pin_A = !pin_A;
        digitalWrite(lightPin_A, pin_A);
      }
      if(message == "Btn_B;")
      {
        pin_B = !pin_B;
        digitalWrite(lightPin_B, pin_B);
      }
      if(message == "Btn_C;")
      {
        pin_C = !pin_C;
        digitalWrite(lightPin_C, pin_C);
      }
    }
  }
}

