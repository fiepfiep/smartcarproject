/*PINS*/
const int IN1 = 10; //Pins usded for motor driver
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENA = 5;
const int ENB = 6; //Pins usded for motor driver

void setup()
{
  for (int i = 5; i < 11; i ++) //set motor pins as output
  {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);
}

void loop(){

  double softTurn_ratio = 0.3; //turn at about 1 carpet tile (altran brussels office)
  double hardTurn_ratio = 0; //turn at about 1/2 carpet tile
  float speed = 170;

  //drive circle counter clockwise
  //control_motors(round(softTurn_ratio * speed), speed);
  control_motors(0, 150);

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
    analogWrite(ENA, -right);
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
    analogWrite(ENB, -left);
  }
}
