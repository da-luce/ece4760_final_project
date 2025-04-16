char x = 'Z';

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.write(x);  // Send a byte
  delay(1000); // just to avoid spamming too fast
}
