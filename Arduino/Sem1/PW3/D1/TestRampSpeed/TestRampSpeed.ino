#include <LiquidCrystal.h>

// Pin definitions for encoders
#define LEFT_ENCODER_PIN A4
#define RIGHT_ENCODER_PIN A5

// Encoder & Wheel specifications
const double WHEEL_CIRCUMFERENCE = 9;  // in cm (adjust for your wheel size)
const int PULSES_PER_REVOLUTION = 20;  // Adjust for your encoder

// Variables
volatile int leftEncoderCount = 0;
volatile int rightEncoderCount = 0;
double totalDistance = 0;
double maxDistance = 500;

// LCD Pin Configuration
const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Motor Control Pins
const int in1 = 13;  // Left motor input 1
const int in2 = 12;  // Left motor input 2
const int in3 = 11;  // Right motor input 1
const int in4 = 3;   // Right motor input 2

void setup() {
  // Initialize encoder pins
  pinMode(LEFT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), countLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), countRightEncoder, RISING);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("Distance:");
  lcd.setCursor(0, 1);
  lcd.print("0.0 cm");

  // Motor setup
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Start motors (forward direction)
  // digitalWrite(in1, HIGH);
  // digitalWrite(in2, LOW);
  // digitalWrite(in3, HIGH);
  // digitalWrite(in4, LOW);
}

void loop() {
  // Calculate total distance traveled
  double leftDistance = calculateWheelDistance(leftEncoderCount);
  double rightDistance = calculateWheelDistance(rightEncoderCount);
  double linearDistance = (leftDistance + rightDistance) / (double)2;

  // Update total distance
  totalDistance += linearDistance;

  if (totalDistance < maxDistance) {
    // Display total distance on the LCD
    lcd.setCursor(0, 1);
    lcd.print(totalDistance, 1);  // Display distance with 1 decimal place
    lcd.print(" cm ");

    // Start motors (forward direction)
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else {
    // Stop motors
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  // Reset encoder counts for the next update
  resetEncoders();

  delay(500);  // Update every half second
}

// Interrupt service routines (ISR) for encoders
void countLeftEncoder() {
  leftEncoderCount++;
}

void countRightEncoder() {
  rightEncoderCount++;
}

// Calculate the distance traveled by a wheel based on encoder counts
double calculateWheelDistance(int encoderCount) {
  return ((double)encoderCount / (double)PULSES_PER_REVOLUTION) * WHEEL_CIRCUMFERENCE;
}

// Reset encoder counts
void resetEncoders() {
  leftEncoderCount = 0;
  rightEncoderCount = 0;
}
