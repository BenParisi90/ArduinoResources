/* Arduino USB HID Keyboard Demo
 * Random Key/Random Delay
 */

uint8_t buf[8] = { 
  0 }; 	/* Keyboard report buffer */

void setup() 
{
  Serial.begin(9600);
  randomSeed(analogRead(0));
  delay(200);
}

void loop() 
{
  int randomChar = random(4, 30);
  buf[2] = randomChar;	  // Random character
  Serial.write(buf, 8);	// Send keypress
  releaseKey();
  Serial.print(randomChar);
  
  delay(3000);
}

void releaseKey() 
{
  buf[0] = 0;
  buf[2] = 0;
  Serial.write(buf, 8);	// Release key  
}
