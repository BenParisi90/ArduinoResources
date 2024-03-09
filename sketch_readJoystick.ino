//#include <Keyboard.h>

const int sw_pin = 2;
const int x_pin = A0;
const int y_pin = A1;

void setup() {
  // put your setup code here, to run once:
  pinMode(sw_pin, INPUT);
  pinMode(x_pin, INPUT);
  pinMode(y_pin, INPUT);
  digitalWrite(sw_pin, HIGH);

  //
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int x_data = analogRead(A0);
  int y_data = analogRead(A1);
  int sw_data = digitalRead(sw_pin);

  Serial.print("x_data:");
  Serial.print(x_data);
  Serial.print("\t");
  Serial.print("y_data:");
  Serial.print(y_data);
  Serial.print("\t");
  Serial.print("sw_data:");
  Serial.print(sw_data);

  //delay(100);
}
