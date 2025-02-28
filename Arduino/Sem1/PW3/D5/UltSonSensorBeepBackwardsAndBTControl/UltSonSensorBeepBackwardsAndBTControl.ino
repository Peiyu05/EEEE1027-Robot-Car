#include <NewPing.h>

// Ultrasonic sensor pins
#define TRIGGER_PIN 9
#define ECHO_PIN 10
#define MAX_DISTANCE 400  // Maximum distance to measure (in cm)

// Buzzer pin
#define BUZZER_PIN 8  // Define the pin for the buzzer

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // Initialize the ultrasonic sensor

// Motor control pins
const int ENA = 5;   // Enable right-side pair of motors -- Pair A
const int ENB = 6;   // Enable left-side pair of motors -- Pair B
const int IN1 = 4;   // Command Pair A forwards
const int IN3 = A0;  // Command Pair A backwards
const int IN2 = 7;   // Command Pair B forwards
const int IN4 = A2;  // Command Pair B backwards

int SPEED = 150;  // Default motor speed (0-255)

#define FORWARD 'F'
#define BACKWARD 'B'
#define LEFT 'L'
#define RIGHT 'R'
#define CIRCLE 'C'
#define CROSS 'X'
#define TRIANGLE 'T'
#define SQUARE 'S'
#define START 'A'
#define PAUSE 'P'

unsigned long
  startTime = 0,     // Time when setup() ends in milliseconds
  elapsedTime = 0,   // Number of milliseconds since setup ended
  detectedTime = 0;  // Time at which obstacle was detected in milliseconds

bool obstacleDetected = false;

void setup() {
  Serial.begin(9600);  // Set the baud rate for serial communication

  // Motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Buzzer pin as output
  pinMode(BUZZER_PIN, OUTPUT);

  // Measure start time
  startTime = millis();
}

void loop() {
  // Measure time since setup() ended
  elapsedTime = millis() - startTime;

  if (Serial.available()) {
    char command = Serial.read();
    executeCommand(command);
  }

  // Get distance from the ultrasonic sensor
  int distToObstacle = sonar.ping_cm();

  if (obstacleDetected == true) {
    tone(BUZZER_PIN, 1000);
  } else {
    noTone(BUZZER_PIN);
  }

  noTone(BUZZER_PIN);  // Deactivate the buzzer when no obstacle is detected

  //// Check distance and control movement
  // Stop if an obstacle is detected within 20 cm
  if (distToObstacle > 0 && distToObstacle < 20 && obstacleDetected == false) {
    obstacleDetected = true;
    detectedTime = elapsedTime;
    stopCar();
    tone(BUZZER_PIN, 1000);  // Activate the buzzer with a 1000 Hz tone
    Serial.println("Obstacle detected! Stopping and sounding buzzer.");
  }

  if (distToObstacle > 0 && distToObstacle < 20 && obstacleDetected == true && (elapsedTime - detectedTime) > 3000) {
    noTone(BUZZER_PIN);
    obstacleDetected = false;
    moveBackward();
    delay(1500);
    stopCar();
  }

  delay(50);
}

// Function to move forward
void moveForward() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Moving Forward");
}

// Function to move backward
void moveBackward() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Moving Backward");
}

// Function to turn left
void turnLeft() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  Serial.println("Turning Left");
}

// Function to turn right
void turnRight() {
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  Serial.println("Turning Right");
}

// Function to stop the car
void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  Serial.println("Stopping");
}

// Function to increase speed
void increaseSpeed() {
  SPEED = min(SPEED + 25, 255);  // Ensure speed doesn't exceed 255
  Serial.print("Increasing Speed. New speed: ");
  Serial.println(SPEED);
}

// Function to decrease speed
void decreaseSpeed() {
  SPEED = max(SPEED - 25, 0);  // Ensure speed doesn't go below 0
  Serial.print("Decreasing Speed. New speed: ");
  Serial.println(SPEED);
}

// Function to execute command
void executeCommand(char command) {
  switch (command) {
    case FORWARD:
      moveForward();
      break;
    case BACKWARD:
      moveBackward();
      break;
    case LEFT:
      turnLeft();
      break;
    case RIGHT:
      turnRight();
      break;
    case CIRCLE:
      increaseSpeed();
      break;
    case CROSS:
      decreaseSpeed();
      break;
    case PAUSE:
      stopCar();
      break;
    default:
      Serial.println("Invalid command");
      break;
  }
}
