#include <NewPing.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400 // Maximum distance to measure (in cm)

// Buzzer pin
#define BUZZER_PIN 8 // Define the pin for the buzzer

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // Initialize the ultrasonic sensor

// Motor control pins
const int ENA = 5;  // Enable right-side pair of motors -- Pair A
const int ENB = 6;  // Enable left-side pair of motors -- Pair B
const int IN1 = 4;  // Command Pair A forwards
const int IN3 = A0;  // Command Pair A backwards
const int IN2 = 7;  // Command Pair B forwards
const int IN4 = A2;  // Command Pair B backwards

void setup() {
  // Motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);

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
  if (distance > 0 && distance < 20) { // Stop if an obstacle is detected within 20 cm
    stopCar();
    tone(BUZZER_PIN, 1000); // Activate the buzzer with a 1000 Hz tone
    Serial.println("Obstacle detected! Stopping and sounding buzzer.");
  } else {
    moveForward();
    noTone(BUZZER_PIN); // Deactivate the buzzer
  }

  delay(50); // Small delay for stability
}
