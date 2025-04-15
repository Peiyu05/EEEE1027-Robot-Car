#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> // Required for Adafruit MPU-6050 library
#include <Wire.h>

// Motor pins
const int ENA = 3, IN1 = 4, IN2 = 5;  // Right motor
const int ENB = 11, IN3 = 6, IN4 = 7; // Left motor

// Ultrasonic pins
const int TRIG1 = A0, ECHO1 = A1; // Right Sensor
const int TRIG2 = 9, ECHO2 = 8;  // Mid Sensor
const int TRIG3 = A2, ECHO3 = A3; // Left Sensor

// Safe distance in cm
const int mazeWidth = 12; // Increased for clearer open space detection
const int DEADZONE = 3;   // Ignore small differences
const int obstacleThreshold = 15; // Distance to consider an obstacle in front

// Timer variables
unsigned long lastSensorUpdate = 0;
const int SENSOR_INTERVAL = 35; // Update every 20ms

// Sensor distances
float distanceRight = 0;
float distanceFront = 0;
float distanceLeft = 0;

Adafruit_MPU6050 mpu;

// Constants for turning with MPU-6050
const float targetTurnAngle = 180.0; // Degrees to turn
const float rotationSpeedThreshold = 5.0; // Degrees per second to consider as still
float integratedAngle = 0.0;
unsigned long lastTime = 0;
bool isTurning = false;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void AllignLeft();
void AllignRight();
void Allignment();
float getDistance(int trigPin, int echoPin);

void setup() {
  // Motor pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200); // Initialize serial communication for debugging

  // Ultrasonic Sensor pin modes
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);
  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  // Initialize MPU-6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  // Optional: Set sensor ranges (you might need to adjust these)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  lastSensorUpdate = millis();  // Initialize timer variable (optional)
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSensorUpdate >= SENSOR_INTERVAL) {
    lastSensorUpdate = currentMillis;
    distanceRight = getDistance(TRIG1, ECHO1);
    distanceFront = getDistance(TRIG2, ECHO2);
    distanceLeft = getDistance(TRIG3, ECHO3);

    // Optional: Print sensor values for debugging
    // Serial.print("R: "); Serial.print(distanceRight);
    // Serial.print(" F: "); Serial.print(distanceFront);
    // Serial.print(" L: "); Serial.println(distanceLeft);
  }

  // Implement Left-Hand Rule
  if (distanceLeft > mazeWidth) {
    moveForward();
    delay(700);
    turnLeft();
  } else if (distanceFront > obstacleThreshold) {
    Allignment();
  } else if (distanceRight > mazeWidth) {
    moveForward();
    delay(1000);
    turnRight();
  } else if (distanceRight < mazeWidth && distanceLeft < mazeWidth && distanceFront < mazeWidth) {
    Serial.println("Trying to Turn Around");
    turnAround(); // Use MPU-6050 for turn around
  } else {}
}

void turnAround() {
  if (!isTurning) {
    Serial.println("Turning Around...");
    integratedAngle = 0.0;
    lastTime = millis();
    isTurning = true;
    // Start turning (e.g., turn right) - Increased speeds here as well
    stopMotors();
    analogWrite(ENA, 150);
    analogWrite(ENB, 50);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  if (isTurning) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Time in seconds
    lastTime = currentTime;

    // Assuming g.gyro.z is the yaw rate (you might need to adjust the axis)
    float currentRotationRate = g.gyro.z;
    integratedAngle += currentRotationRate * deltaTime;

    Serial.print("Current Angle: ");
    Serial.println(integratedAngle);

    if (abs(integratedAngle) >= targetTurnAngle) {
      Serial.println("Turn Around Complete");
      stopMotors();
      isTurning = false;
      delay(500); // Small delay after turning
    } else if (abs(currentRotationRate) < rotationSpeedThreshold && (millis() - currentTime > 1000)) {
      // Safety break if rotation stops prematurely
      Serial.println("Rotation stopped unexpectedly. Trying again.");
      stopMotors();
      isTurning = false;
      delay(1000);
      // Optionally, you could try turning in the opposite direction briefly here
    }
  }
}

void Allignment() {
  //Move forward and allignment to go straight

  if (distanceLeft < mazeWidth && distanceRight < mazeWidth) { // Walls on both sides
    if (distanceLeft - distanceRight < -DEADZONE) {
      AllignRight();
    } else if (distanceLeft - distanceRight > DEADZONE) {
      AllignLeft();
    } else {
      moveForward();
    }
  }

  else if (distanceLeft < mazeWidth && distanceRight >= mazeWidth) { // Left wall only
    if (distanceLeft < mazeWidth - 2) { // Adjust these values based on your robot
      AllignRight();
    } else if (distanceLeft > mazeWidth + 2) {
      AllignLeft();
    } else {
      moveForward();
    }
  }

  else if (distanceLeft >= mazeWidth && distanceRight < mazeWidth) { // Right wall only
    if (distanceRight < mazeWidth - 2) { // Adjust these values based on your robot
      AllignLeft();
    } else if (distanceRight > mazeWidth + 2) {
      AllignRight();
    } else {
      moveForward();
    }
  }

  else { // Open space on both sides - continue straight
    turnLeft();
  }
}

void AllignLeft() {
  analogWrite(ENA, 90);
  analogWrite(ENB, 85);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void AllignRight() {
  analogWrite(ENA, 85);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveForward() {
  analogWrite(ENA, 70);
  analogWrite(ENB, 70);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  analogWrite(ENA, 65);
  analogWrite(ENB, 80);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// Function to turn left
void turnLeft() {
  analogWrite(ENA, 70);
  analogWrite(ENB, 90);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(250); // Adjust delay based on your robot's turning speed
}

// Function to turn right
void turnRight() {
  analogWrite(ENA, 180);
  analogWrite(ENB, 100);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(150); // Adjust delay based on your robot's turning speed
}

// Function to stop motors
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// Function to get distance from ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms
  return duration * 0.034 / 2;
}