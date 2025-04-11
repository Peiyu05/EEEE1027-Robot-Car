#include <NewPing.h>

// Motor control pins
const int
  ENA = 5,   // Enable right-side pair of motors -- Pair A
  ENB = 6,   // Enable left-side pair of motors -- Pair B
  IN1 = 4,   // Command Pair A forwards
  IN3 = A0,  // Command Pair A backwards
  IN2 = 7,   // Command Pair B forwards
  IN4 = A2;  // Command Pair B backwards

// Utrasonic Sensor
const int
  TRIGGER_PIN = 9,
  ECHO_PIN = 10,
  MAX_DISTANCE = 400;  // Maximum distance to measure (in cm)
// Initialize the ultrasonic sensor
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

bool inMotion = false;

// Movement functions
void moveForward(const int speed) {  // Move car forward
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);
  // Set inMotion to indicate robot is moving
  if (inMotion == false) { inMotion = true; }
}
void moveLeft(const int speed) {
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, HIGH);
  // Set inMotion to indicate robot is moving
  if (inMotion == false) { inMotion = true; }
}
void moveRight(const int speed) {
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, LOW);
  // Set inMotion to indicate robot is moving
  if (inMotion == false) { inMotion = true; }
}
void moveStop() {
  // Speed control
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  // Direction control
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);
  // Set inMotion to indicate robot is stationary
  if (inMotion == true) { inMotion = false; }
}

void setup() {
  // Motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
}

void loop() {
  // Get distance from the ultrasonic sensor
  int distance = sonar.ping_cm();

  // Print distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");

  // Check distance and control movement
  if (distance < 5) {
    moveForward(127);
  }
  if (distance >= 5) {
    moveStop();
    Serial.println("Cliff edge detected! Stopping.");
  }

  delay(25);  // Small delay for stability
}
