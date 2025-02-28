const int
  PIN_RED = 9,
  PIN_GREEN = 10,
  PIN_BLUE = 11;

void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(PIN_RED, redValue);
  analogWrite(PIN_GREEN, greenValue);
  analogWrite(PIN_BLUE, blueValue);
}

void setup() {
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
}

void loop() {
  setColor(0, 127, 0);
  delay(1500);
  setColor(0, 0, 127);
  delay(1500);
  setColor(127, 0, 0);
  delay(1500);
  setColor(255, 255, 0);
  delay(1500);
}
