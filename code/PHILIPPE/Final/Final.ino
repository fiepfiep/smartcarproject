#include <IRremote.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display
int IN1 = 6;
int IN2 = 7;
int IN3 = 8;
int IN4 = 9;
int ENA = 5;
int ENB = 10;

int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;
void setup()
{
  for (int i = 5; i < 11; i ++)
  {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
  Wire.begin();
  Wire.beginTransmission(0x27);
  lcd.setBacklight(255);
  lcd.home(); lcd.clear();
  lcd.print("Hello LCD");
  delay(1000);
  irrecv.enableIRIn(); // Start the receiver
}
void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    lcd.home(); lcd.clear();
    lcd.print(results.value);
    irrecv.resume(); // Receive the next value
    switch (results.value)
    {
      case 0xFF629D:
        Serial.println("forward");
        forward();
        break;
      case 0xFFA857:
        Serial.println("back");
        backward();
        break;
      case 0xFF22DD:
        Serial.println("left");
        left();
        break;
      case 0xFFC23D:
        Serial.println("right");
        right();
        break;
      case 0xFF02FD:
        Serial.println("stop");
        stop();
        break;
    }

  }
}


void forward()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 140);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 140);
}

void backward()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 140);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 140);
}

void right()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);
}

void left()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 0);
}

void stop()
{
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 0);
}
