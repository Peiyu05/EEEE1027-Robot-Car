#include <string.h>
#include <LiquidCrystal.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// // LCD
// LCD control pins
const int
  REGISTER_SELECT = 8,
  ENABLE = 9,
  DB4 = 4,
  DB5 = 5,
  DB6 = 6,
  DB7 = 7;
// Initialize the LCD
LiquidCrystal lcd(REGISTER_SELECT, ENABLE, DB4, DB5, DB6, DB7);

// ------------------------------------------------
// ------------------------------------------------

// // Accelerometer

// Var definitions
// Vars for storing raw accelerometer data
float
  accel_x,  // X-axis raw acceleration data
  accel_y,  // Y-axis raw acceleration data
  accel_z;  // Z-axis raw acceleration data

// Vars for storing calculated roll and pitch
double
  roll = 0,   // Calculated roll angle
  pitch = 0;  // Calculated pitch angle

// String vars for LCD printing
char messageSer[15] = "";  // Final message to be printed to the serial monitor
char message[15] = "";     // Final message to be printed on LCD
char angle[10] = "";       // Buffer for string representation of the pitch angle

// Initialise the accelerometer
Adafruit_MPU6050 mpu;

// ------------------------------------------------
// ------------------------------------------------

// Function to calculate roll and pitch from accelerometer data
void calculateRollPitch(float x, float y, float z) {
  double x_Buff = (double)x;
  double y_Buff = (double)y;
  double z_Buff = (double)z;

  double rad2deg = (double)180 / (double)PI;

  roll = atan2(y_Buff, z_Buff) * rad2deg;
  // pitch = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * rad2deg * (double)-1.00;
  pitch = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * rad2deg;
}

// Function to calculate pitch from MPU accelerometer data and return the calculated value
float calcPitch(const float x, const float y, const float z) {
  // Store accelerometer data for each axis as a double
  double x_Buff = (double)x;
  double y_Buff = (double)y;
  double z_Buff = (double)z;

  double rad2deg = (double)180 / (double)PI;                                           // Convert the output of the formula used to degrees
  float result = atan2((-x_Buff), sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * rad2deg;  // Calculate pitch in degrees from accelerometer data

  return result;  // Return calculated pitch in degrees
}

// Function to display the ramp angle on the LCD
void displayAngle(const float angle) {
  lcd.setCursor(0, 1);  // Set cursor to the first character of the second row
  lcd.print(angle, 2);  // Print the angle to two decimal places
  lcd.print("DEG   ");  // Print units and clearing space
}

void setup(void) {
  // Initialize the LCD
  lcd.begin(16, 2);     // Set up the LCD with 16 columns and 2 rows
  lcd.setCursor(0, 0);  // Set cursor to the first character of the first row
  lcd.print("Looking for MPU");

  delay(250);

  // Start Serial Comms
  Serial.begin(115200);

  // Clear lines to indicate new data
  Serial.println("");
  Serial.println("");

  // Try to initialize MPU
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  Serial.println("");  // Clear line to seperate MPU status print from data print
  lcd.print("MPU6050 Found!");

  // set accelerometer range to +-16G
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  // set filter bandwidth to 10 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  delay(500);
  lcd.clear();
  lcd.setCursor(0, 0);       // Set cursor to the first character of the first row
  lcd.print("Ramp Angle:");  // Print "Ramp Angle" text on the first line
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accel_x = a.acceleration.x;
  accel_y = a.acceleration.y;
  accel_z = a.acceleration.z;

  pitch = calcPitch(accel_x, accel_y, accel_z);

  strcpy(messageSer, "Angle: ");
  dtostrf(pitch, 2, 2, angle);
  strcpy(message, angle);
  strcat(message, "DEG");
  strcat(messageSer, message);

  /* Print out the values */
  // Serial.print(a.acceleration.x);
  // Serial.print(",");
  // Serial.print(a.acceleration.y);
  // Serial.print(",");
  // Serial.print(a.acceleration.z);
  // Serial.print(",");
  // Serial.print(pitch);
  // // Serial.print(",");
  // // Serial.print(roll);
  // // Serial.print(", ");
  // // Serial.print(g.gyro.x);
  // // Serial.print(",");
  // // Serial.print(g.gyro.y);
  // // Serial.print(",");
  // // Serial.print(g.gyro.z);
  // Serial.println("");
  Serial.println(messageSer);

  // lcd.setCursor(0, 1);  // Set cursor to the first character of the second row
  // lcd.print(message);   // Print ramp angle
  displayAngle(pitch);

  delay(25);
}
