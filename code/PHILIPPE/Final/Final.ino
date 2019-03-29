/*
  Smart Car Project
  Philippe Baetens
  March 2019 @ ALTRAN
*/

/*LIBS*/
#include <Servo.h>

#include <IRremote.h> // for IR
#include <Wire.h> // for LCD
#include <LiquidCrystal_PCF8574.h>  // for LCD
#include <SoftwareSerial.h> // for BT
#include "SR04.h" //ultrasonic
#define TRIG_PIN A3  //ultrasonic
#define ECHO_PIN A2  //ultrasonic
#define PROXIMITY 20 //dist in cm to stop the car
#define BUT1 0xFF6897
#define BUT2 0xFF9867
#define BUT3 0xFFB04F
#define BUT4 0xFF30CF
#define BUT0 0xFF4AB5

/*PINS*/
const int IN1 = 10; //Pins usded for motor driver
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENA = 5;
const int ENB = 6; //Pins usded for motor driver

const int RECV_PIN = 12;    //IR module receiver pin
const int SERVO_PIN = 11;

/*globals*/
int IR_MODE;
int BT_CMD;

char cmd = 0;

int pos = 90;    // variable to store the servo position

// car kinematics
float leftspeed = 0;
float rightspeed = 0;

SoftwareSerial hc06(A1, A0);  //serial connection to talk with BT module
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
IRrecv irrecv(RECV_PIN);
decode_results results; //decoded IR character
Servo myservo;  // create servo object to control a servo

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long a;


void setup()
{
  for (int i = 5; i < 11; i ++) //set motor pins as output
  {
    pinMode(i, OUTPUT);
  }
  pinMode(SERVO_PIN, OUTPUT);

  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x27);
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.home(); lcd.clear();
  lcd.blink();
  lcd.print("Altran Smart Car....");
  lcd.setCursor(0, 1);
  lcd.print("press a button");
  irrecv.enableIRIn(); // Start the receiver
  hc06.begin(9600);
  myservo.attach(SERVO_PIN);  // attaches the servo on pin; causing problems !!!!!!!!!

}
void loop() {
  //decode IR, write to display
  drive_controlled();
  irdecode();
  //call correct mode-function depending on IR signal



}

void irdecode()
{
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume(); // Receive the next value

    switch (results.value)
    {
      case BUT1:
        lcd.home(); lcd.clear();
        Serial.println("mode 1: random");
        lcd.print("mode 1: random");
        drive_random();
        break;
      case BUT2:
        lcd.home(); lcd.clear();
        Serial.println("mode 2: BT");
        lcd.print("mode 2: BT");
        drive_controlled();
        break;
      case BUT3:
        lcd.home(); lcd.clear();
        lcd.print("mode 3: linetrack");
        Serial.println("mode 3: linetrack");

        break;
      case BUT4:
        lcd.home(); lcd.clear();
        Serial.println("mode 4: BT assisted");
        lcd.print("mode 4: BT assisted");
        break;
      case BUT0:
        lcd.home(); lcd.clear();
        Serial.println("halt");
        lcd.print("halted");
        break;
    }
  }
}

void drive_random() //drive randomly, avoid objects
{
  while (1)
  {
     stop();
    int d_l, d_m, d_r;
    myservo.write(90);
    delay(1000);
    measuredist();
    d_m = a;
    myservo.write(120);
    delay(1000);
    measuredist();
    d_l = a;
    myservo.write(90);
    delay(1000);
    measuredist();
    d_r = a;


    if (d_l  < 30 )
    {
      right();
      delay(300);
    }
    if (d_r  < 30 )
    {
      left();
      delay(300);
    }
    forward();
    delay(500);
    irdecode();
    break;
  }

}
void drive_controlled() //drive bluetooth
{
  myservo.write(90);
  while (1)
  {
    cmd = (char)hc06.read();

    switch (cmd)
    {
      case 'l':
        Serial.println("left");
        left();
        break;
      case 'r':
        Serial.println("right");
        right();
        break;
      case 'f':
        forward();
        Serial.println("forward");
        break;
      case 'b':
        backward();
        Serial.println("backward");
        break;
      case 'x':
        stop();
        Serial.println("stop");
        break;
      case 'n':
        Serial.println("go");
        break;
      default:
        cmd = 0;
        break;
    }


    measuredist();
    if (a < PROXIMITY)
      stop();
    irdecode();
  }
}
void drive_line(); //track a line. Stop if no line found after some time
void drive_assisted(); //drive bluetooth and avoid obstacles


void calcmotorspeed()
{
  control_motors(leftspeed, rightspeed);

}

void forward()
{
  leftspeed = 100;
  rightspeed = 100;
  calcmotorspeed();

}

void backward()
{
  leftspeed = -50;
  rightspeed = -50;
  calcmotorspeed();

}

void right()
{
  leftspeed = 150;
  rightspeed = 0;
  calcmotorspeed();

}

void left()
{
  rightspeed = 150;
  leftspeed = 0;
  calcmotorspeed();

}

void stop()
{
  leftspeed = 0;
  rightspeed = 0;
  control_motors(0, 0);

}


void control_motors(float left, float right)
{
  Serial.println("left,right");
  Serial.println(left);
  Serial.println(right);

  if (right > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, right);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, right);
  }
  if (left > 0)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, left);
  }

  else
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, left);
  }
}


void measuredist() {
  a = sr04.Distance();

}
