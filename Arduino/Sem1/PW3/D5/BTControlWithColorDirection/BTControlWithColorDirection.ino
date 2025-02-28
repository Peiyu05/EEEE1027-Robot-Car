// Mapping gamepad output to each function
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

// Motor control pins
const int
  ENA = 5,   // Enable right-side pair of motors -- Pair A
  ENB = 6,   // Enable left-side pair of motors -- Pair B
  IN1 = 4,   // Command Pair A forwards
  IN3 = A0,  // Command Pair A backwards
  IN2 = 7,   // Command Pair B forwards
  IN4 = A2;  // Command Pair B backwards

bool inMotion = false;

unsigned long
  startTime = 0,     // Time when setup() ends in milliseconds
  elapsedTime = 0,   // Number of milliseconds since setup ended
  detectedTime = 0;  // Time at which obstacle was detected in milliseconds

int speed = 127;

const int
  PIN_RED = 9,
  PIN_GREEN = 10,
  PIN_BLUE = 11;

void setColor(int redValue, int greenValue, int blueValue) {
  analogWrite(PIN_RED, redValue);
  analogWrite(PIN_GREEN, greenValue);
  analogWrite(PIN_BLUE, blueValue);
}

// Movement functions
void moveForward() {  // Move car forward
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, LOW);

  setColor(0, 127, 0);

  // Set inMotion to indicate robot is moving
  if (inMotion == false) {
    inMotion = true;
  }
}
void moveBackward() {  // Move car backward
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, HIGH);

  setColor(127, 0, 0);

  // Set inMotion to indicate robot is moving
  if (inMotion == false) { inMotion = true; }
}
void moveLeft() {
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN4, HIGH);

  setColor(0, 0, 127);

  // Set inMotion to indicate robot is moving
  if (inMotion == false) { inMotion = true; }
}
void moveRight() {
  // Speed control
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
  // Direction control
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN4, LOW);

  setColor(255, 255, 0);

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

  setColor(127, 0, 0);

  // Set inMotion to indicate robot is stationary
  if (inMotion == true) { inMotion = false; }
}
void increaseSpeed() {
  speed = min(speed + 25, 255);  // Ensure speed doesn't exceed 255
  Serial.print("Increasing Speed. New speed: ");
  Serial.println(speed);
}
void decreaseSpeed() {
  speed = max(speed - 25, 0);  // Ensure speed doesn't go below 0
  Serial.print("Decreasing Speed. New speed: ");
  Serial.println(speed);
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
      moveLeft();
      break;
    case RIGHT:
      moveRight();
      break;
    case CIRCLE:
      increaseSpeed();
      break;
    case CROSS:
      decreaseSpeed();
      break;
    case PAUSE:
      moveStop();
      break;
    default:
      Serial.println("Invalid command");
      break;
  }
}

void setup() {
  // Motor pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN4, OUTPUT);

  // LED pins as outputs
  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);

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

  delay(20);
}
