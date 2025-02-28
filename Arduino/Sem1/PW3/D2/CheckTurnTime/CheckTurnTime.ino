#include <LiquidCrystal.h>

const int LOOP_DELAY = 20;

// Motor controls
const int
  ENA = 5,  // Enable right-side pair of motors -- Pair A
  ENB = 6,  // Enable left-side pair of motors -- Pair B
  IN1 = 4,  // Command Pair A forwards
  IN3 = 7,  // Command Pair A backwards
  IN2 = 2,  // Command Pair B forwards
  IN4 = 3;  //Command Pair B backwards

// // LCD
// LCD control pins
const int
  REGISTER_SELECT = 12,
  ENABLE = 13,
  DB4 = 8,
  DB5 = 9,
  DB6 = 10,
  DB7 = 11;
// Initialize the LCD
LiquidCrystal lcd(REGISTER_SELECT, ENABLE, DB4, DB5, DB6, DB7);

// Time tracking variables
unsigned long TURN_DURATION = 2370;  // Units are milliseconds
unsigned long startTime = 0;
volatile unsigned long elapsedTime = 0;
bool inMotion = false;

//
//

// Display time function
void displayTime(volatile unsigned long time) {
  // Calculate time in seconds
  const unsigned long seconds = time / 1000;

  // Print time on the 1st row of the LCD
  lcd.setCursor(0, 0);
  lcd.print("Time: ");
  lcd.print(seconds);
  lcd.print("s     ");
}
void moveRight(int speed) {
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, LOW);

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

  if (inMotion == true) { inMotion = false; }
}

void setup() {
  // Set up motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set Starting time of run
  startTime = millis();
}

void loop() {
  // Calculate the amount of time that has passed
  elapsedTime = millis() - startTime;

  if (elapsedTime < TURN_DURATION) {
    moveRight(180);
  } else {
    moveStop();
  }

  if (inMotion == true) {
    displayTime(elapsedTime);
  }
}
