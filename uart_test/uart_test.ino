float x = 3.14;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.write((uint8_t*)&x, sizeof(float));  // Send 4 bytes
  delay(1000); // just to avoid spamming too fast
}
