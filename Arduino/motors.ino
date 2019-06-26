const int PINS[8] = {2, 3, 4, 5, 6, 7, 8, 9};
void setup() {
  Serial.begin(115200);
  for (char i = 0; i < 8; i++)
    pinMode(PINS[i], OUTPUT);
}

void loop() {
  if (Serial.available() >= 8)
    for (char i = 0; i < 8; i++)
      analogWrite(PINS[i], Serial.read());
}
