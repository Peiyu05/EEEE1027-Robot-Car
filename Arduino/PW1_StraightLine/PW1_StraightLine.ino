#include <LiquidCrystal.h>
// #include <TimeLib.h>

unsigned long int seconds = 0;
const long int time_limit = 10;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// const int ena = 2;
// const int enb = 3;
// const int in1 = A1;
// const int in2 = A2;
// const int in3 = A3;
// const int in4 = A4;
const int in1 = 1;
const int in2 = 2;
const int in3 = 11;
const int in4 = 12;
 
void setup()
{

lcd.begin(16,2);
lcd.setCursor(0, 0);
lcd.print("Time elapsed:");
lcd.setCursor(0, 1);
lcd.print(seconds);

// pinMode(ena, OUTPUT);
// pinMode(enb, OUTPUT);
pinMode(in1, OUTPUT);
pinMode(in2, OUTPUT);
pinMode(in3, OUTPUT);
pinMode(in4, OUTPUT);

// digitalWrite(ena, HIGH);
// digitalWrite(enb, HIGH);

delay(1000);

}
 
void loop()
{

if (seconds <  time_limit) {
digitalWrite(in1, HIGH);
digitalWrite(in2, LOW);
digitalWrite(in3, HIGH);
digitalWrite(in4, LOW);

seconds += 1;
lcd.setCursor(0, 1);
lcd.print(seconds);
delay(1000);
} else {
  digitalWrite(in1, LOW);
digitalWrite(in2, LOW);
digitalWrite(in3, LOW);
digitalWrite(in4, LOW);


seconds += 1;
lcd.setCursor(0, 1);
lcd.print(seconds);
delay(1000);
}

}

