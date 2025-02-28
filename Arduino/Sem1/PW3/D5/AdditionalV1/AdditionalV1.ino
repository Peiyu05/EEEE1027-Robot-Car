#include <NewPing.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400 // Maximum distance to measure (in cm)

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Initialize the ultrasonic sensor

// Motor control pins
const int ENA = 5;  // Enable right-side pair of motors -- Pair A
const int ENB = 6;  // Enable left-side pair of motors -- Pair B
const int IN1 = 4;  // Command Pair A forwards
const int IN3 = 2;  // Command Pair A backwards
const int IN2 = 7;  // Command Pair B forwards
const int IN4 = 3;  // Command Pair B backwards

// Desired distance from the obstacle (in cm)
const int TARGET_DISTANCE = 100;
const int TOLERANCE = 5; // Allowable deviation from target distance

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

void moveForward() {
  analogWrite(ENA, 150); // Adjust speed if needed
  analogWrite(ENB, 150);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, 150); // Adjust speed if needed
  analogWrite(ENB, 150);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void loop() {
  // Get distance from the ultrasonic sensor
  int distance = sonar.ping_cm();

  // Print distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println("cm");

  // Check distance and control movement
  if (distance > 0) {
    if (distance > TARGET_DISTANCE + TOLERANCE) {
      moveForward(); // Move closer to the obstacle
      Serial.println("Obstacle too far. Moving forward.");
    } else if (distance < TARGET_DISTANCE - TOLERANCE) {
      moveBackward(); // Move away from the obstacle
      Serial.println("Obstacle too close. Moving backward.");
    } else {
      stopCar(); // Stay at the desired distance
      Serial.println("Holding position at 100 cm.");
    }
  } else {
    stopCar(); // If distance measurement fails or is invalid
    Serial.println("Distance unknown. Stopping.");
  }

  delay(50); // Small delay for stability
}
