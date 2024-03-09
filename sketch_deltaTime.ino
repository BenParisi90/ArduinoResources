long previousMillis = 0;  // Variable to store the previous millis() value
long currentMillis = 0;   // Get the current millis() value

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  currentMillis = millis();

  // Calculate the time elapsed since the last output
  unsigned long deltaTime = currentMillis - previousMillis;

  // Output the delta time to the Serial monitor
  Serial.print("Delta Time: ");
  Serial.print(deltaTime);
  Serial.println(" ms");

  // Update the previousMillis with the currentMillis for the next iteration
  previousMillis = currentMillis;
}
