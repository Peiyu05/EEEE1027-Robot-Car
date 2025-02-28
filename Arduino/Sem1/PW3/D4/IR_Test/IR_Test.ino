// IR Sensors
const int
  IR_LEFT = A1,
  IR_RIGHT = A3;

void setup() {
  // Set up IR sensor pins as inputs
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  // Start Serial Comms
  Serial.begin(115200);
  // Clear lines to indicate new serial data
  Serial.println("");
  Serial.println("");

  delay(500);  // Delay for a bit to show the "MPU6050 Found" message
}

void loop() {
  // Read IR sensor values
  int leftSensor = analogRead(IR_LEFT);
  int rightSensor = analogRead(IR_RIGHT);

  Serial.print(leftSensor);
  Serial.print(",");
  Serial.print(rightSensor);
  Serial.println("");

  delay(20);
}
