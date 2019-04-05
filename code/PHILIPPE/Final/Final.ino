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
#define PROXIMITY1 15 //dist in cm to stop the car
#define PROXIMITY2 30
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
char drivestate = 's'; // s stop f forward b backward r right l left
char cmd = 0;

// car kinematics
float leftspeed = 0;
float rightspeed = 0;

SoftwareSerial hc06(A1, A0);  //serial connection to talk with BT module
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
IRrecv irrecv(RECV_PIN);
decode_results results; //decoded IR character
Servo myservo;  // create servo object to control a servo

SR04 sr04 = SR04(ECHO_PIN, TRIG_PIN);
long a; //distance
int d_l, d_m, d_r;
//state machine
int servopos = 90;
int servostate = 'm';
int distances[5];
// ledState used to set the LED
unsigned long previous_millis1 = 0;        // will store last time LED was updated
long TIME1 = 200;           // milliseconds of on-time
unsigned long previous_millis2 = 0;        // will store last time LED was updated
long TIME2 = 200;           // milliseconds of on-time
unsigned long previous_millis3 = 0;        // will store last time LED was updated
long TIME3 = 200;           // milliseconds of on-time
int motorstate = 0;

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

void loop()
{
  //decode IR, write to display
  irdecode();
  //call correct mode-function depending on IR signal
}

void irdecode()
{
  if (irrecv.decode(&results))
  {
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
        halt();
        break;
      case 0xFF629D:
        Serial.println("forward");
        forward();
        lcd.home(); lcd.clear();
        lcd.println("forward");
        break;
      case 0xFFA857:
        lcd.home(); lcd.clear();
        Serial.println("back");
        lcd.print("back");
        backward();
        break;
      case 0xFF22DD:
        lcd.home(); lcd.clear();
        Serial.println("left");
        lcd.print("left");
        left();
        break;
      case 0xFFC23D:
        lcd.home(); lcd.clear();
        Serial.println("right");
        lcd.print("right");
        right();
        break;
      case 0xFF02FD:

        halt();
        break;

    }
  }
}

void halt()
{
  stop();
  lcd.home(); lcd.clear();
  Serial.println("halt");
  lcd.print("halted");

  while (1)
  {
    irdecode();
  }
}

void drive_random()
{
  while (1)
  {
    irdecode();
    look_around();

    unsigned long current_millis3 = millis();
    if ((current_millis3 - previous_millis3) > 200 && motorstate == 1)
    {
      previous_millis3 = current_millis3;
      motorstate = 0;
      control_motors(0, 0);
    }
    else if ((current_millis3 - previous_millis3) > 600 && motorstate == 0)
    {
      previous_millis3 = current_millis3;
      motorstate = 1;
      control_motors(leftspeed, rightspeed);
    }
  }

}

void look_around2()
{
  unsigned long current_millis1 = millis();
  if ((current_millis1 - previous_millis1) > 100)
  {
    previous_millis1 = current_millis1;
    switch (servopos)
    {
      case 'm':
        {
          measuredist();
          d_m = a;

          myservo.write(150); //servo to right
          servostate = 'l'; //next state
          break;
        }
      case 'l':
        {
          measuredist();
          d_l = a;
          myservo.write(90); //servo to right

          servostate = 'n';
          break;
        }
      case 'n':
        {
          measuredist();
          d_m = a;
          myservo.write(30); //servo to right

          servostate = 'r'; //next state
          break;
        }
      case 'r':
        {
          measuredist();
          d_r = a;
          myservo.write(90); //servo to right

          servostate = 'm';
          break;
        }
      default:
        {
          break;
        }
    }
    decide_direction();

  }
}


void look_around()
{
  unsigned long current_millis1 = millis();
  if ((current_millis1 - previous_millis1) > 400)
  {
    previous_millis1 = current_millis1;
    switch (servostate)
    {
      case 'm':
        {
          measuredist();
          d_m = a;

          myservo.write(140); //servo to right
          servostate = 'l'; //next state
          break;
        }
      case 'l':
        {
          measuredist();
          d_l = a;
          myservo.write(90); //servo to right

          servostate = 'n';
          break;
        }
      case 'n':
        {
          measuredist();
          d_m = a;
          myservo.write(40); //servo to right

          servostate = 'r'; //next state
          break;
        }
      case 'r':
        {
          measuredist();
          d_r = a;
          myservo.write(90); //servo to right

          servostate = 'm';

          break;
        }
      default:
        {
          break;
        }
    }
    decide_direction();

  }
}

void decide_direction()
{
  //  unsigned long current_millis2 = millis();
  //
  //  if ((current_millis2 - previous_millis2) > 100)
  //  {
  //    previous_millis2 = current_millis2;

  switch (drivestate)
  {

    case 'f':
      {
        if (d_m < 25 || d_l < 15 || d_r < 15)
        {
          backward();
          break;
        }

        else if (d_m < 40 || d_l < 20 || d_r < 20)
        {
          (d_l < d_r) ? (right()) : (left());
          break;
        }
        break;
      }
    case 'b':
      {
        if (d_m > 20 && (d_l > 30 || d_r > 30))
        {
          (d_l < d_r) ? (right()) : (left());
        }

        break;
      }

    case 'l':
      {
        if (d_m > 40 && d_l > 30 && d_r > 15 )
        {
          slow();
          break;
        }
        else if (d_m < 10 || d_l < 20)
        {
          backward();
        }


        break;
      }
    case 'r':
      {
        if (d_m > 40 && d_l > 15 && d_r > 30 )
        {
          slow();
          break;
        }
        else if (d_m < 10 || d_r < 20)
        {
          backward();
          break;
        }

        break;
      }
    case 's':
      {

        if (d_m > 40 && d_l > 30 && d_r > 30 )
        {
          slow();
          break;
        }


        break;
      }
    default:
      {
        break;
      }
  }
  Serial.println(drivestate);

}
void drive_randomold() //drive randomly, avoid objects
{
  while (1)
  {
    stop();
    int d_l, d_m, d_r;

    myservo.write(150);
    delay(200);
    measuredist();
    d_l = a;

    myservo.write(30);
    delay(200);
    measuredist();
    d_r = a;

    myservo.write(90);
    delay(200);
    measuredist();
    d_m = a;


    if (d_m < 50 || d_r < 50 || d_l < 50)
    {

      if (d_m < 20 || d_l < 10 || d_r < 10)
      {
        backward();
        delay(500);
        if (d_l < d_r)
        {
          left();
        }
        else
        {
          right();
        }
        delay(600);
        stop;
      }
      else if (d_r < d_l)
      {
        left();
        delay(600);
        stop();

      }
      else
      {
        right();
        delay(600);
        stop();
      }
    }
    else
    {
      slow();
      delay(600);
      stop();
    }

    irdecode();

  }

}
void drive_controlled() //drive bluetooth
{
  myservo.write(90);
  stop();
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

    if (a < 20 && drivestate != 's' && drivestate != 'b' )
    {
      Serial.println("too close");
      Serial.println(a);
      stop();
      delay(20);
      backward();
      delay(100);
      stop();
    }

    if (a < 7 )
    {
      Serial.println("too close, going back");
      Serial.println(a);
      backward();
      delay(200);
      stop();
    }

    irdecode();
  }
}
void drive_line(); //track a line. Stop if no line found after some time
void drive_assisted(); //drive bluetooth and avoid obstacles


void calcmotorspeed()
{
  control_motors(leftspeed, rightspeed);
}

void slow()
{
  leftspeed = 120;
  rightspeed = 120;
  calcmotorspeed();
  drivestate = 'f';
}

void forward()
{
  leftspeed = 180;
  rightspeed = 180;
  calcmotorspeed();
  drivestate = 'f';
}

void backward()
{
  leftspeed = -140;
  rightspeed = -150;
  calcmotorspeed();
  drivestate = 'b';
}

void right()
{
  leftspeed = 170;
  rightspeed = 0;
  calcmotorspeed();
  drivestate = 'r';
}

void left()
{
  rightspeed = 180;
  leftspeed = 0;
  calcmotorspeed();
  drivestate = 'l';
}

void stop()
{
  leftspeed = 0;
  rightspeed = 0;
  control_motors(0, 0);
  drivestate = 's';
}


void control_motors(float left, float right)
{


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
