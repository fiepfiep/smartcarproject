
#include "SR04.h"
#include <stdio.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

#define TRIG_PIN A3
#define ECHO_PIN A2
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display



SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long a;
char b[16];
int error;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();

  delay(1000);
  
  lcd.begin(16, 2); // initialize the lcd

  lcd.setBacklight(01);
  lcd.home(); lcd.clear();

  lcd.print("Distance is ");
  delay(100);
}


void loop() {
  a = sr04.Distance();
  Serial.print(a);
  Serial.println("cm");
  //lcd.home(); 
  //lcd.clear();
  //sprintf(b,"distance is %d", a);
  lcd.setCursor(12,0);
  lcd.print(a);
  lcd.print("      ");
  delay(300);
}
