/*LIBRARIES*/
#include <Wire.h> // for LCD
#include <LiquidCrystal_PCF8574.h>  // for LCD

/*PINS*/
const int IN1 = 10; //Pins usded for motor driver
const int IN2 = 7;
const int IN3 = 8;
const int IN4 = 9;
const int ENA = 5;
const int ENB = 6; //Pins usded for motor driver

const int IR_LEFT = 2; //Pins used for line-detecting IR modules
const int IR_MIDDLE = 4;
const int IR_RIGHT = 3;

/*INIT DEVICES*/
LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/*OTHERS*/
bool operating = true; //show when the car is following the line and when it is completely lost
unsigned long prev_time = 0;
int ir_l, ir_m, ir_r; // inputs for line tracking

void setup()
{
  for (int i = 5; i < 11; i ++) //set motor pins as output
  {
    pinMode(i, OUTPUT);
  }

  for(int i = 2 ; i < 5; i++){ //set line-detecting IR's as input
    pinMode(i, INPUT);
  }
  
  Serial.begin(9600);

  //LCD setup
  Wire.begin();
  Wire.beginTransmission(0x27);
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.home(); lcd.clear();
  lcd.blink();
  lcd.print("Altran Smart Car....");
  delay(1000);
}

void loop(){

	unsigned long current_time = millis();
	while(true && (current_time - prev_time > 5)){ // replace the "true" in the while-loop by "operating"
      prev_time = current_time;
	  	operating = drive_line();
	}

}

bool drive_line(){ //track a line. Stop if no line found after some time
  //NOTE: At this point, the line is continuously derivable

  double softTurn_ratio = 0; //0.3 = turn at about 1 carpet tile (altran brussels office floor carpet) at speed ???
  double hardTurn_ratio = -0.5; // negative to cause more friction for turning
  int speed = 130;
  int speedHardTurn = 160; // larger than forward speed for smaller turn radius

  int line = 1; // defines the color of the line: 1 -> black detected 0 -> white detected
  int no_line = 0;

  bool back_on_track; //when car gets lost and finds the line again


  //Get detections
  ir_l = digitalRead(IR_LEFT);
  ir_m = digitalRead(IR_MIDDLE);
  ir_r = digitalRead(IR_RIGHT);

  //Turn based on measurements
  if(ir_l == no_line && ir_m == line && ir_r == no_line){
    // continue straight ahead
    control_motors(speed, speed);
    Serial.println("Straight ahead");
    lcd.home();
    lcd.clear();
    lcd.print("Straight ahead");
  }
  else if (ir_l == line && ir_m == line && ir_r == line){
    // Stop
    control_motors(0, 0);
    Serial.println("Stop");
    lcd.home();
    lcd.clear();
    lcd.print("Stop, destination found");
    return false;
  }
  else if (ir_l == line && ir_m == line && ir_r == no_line){
    // soft turn left
    control_motors(round(softTurn_ratio * speed), speed);
    Serial.println("Soft turn left");
    lcd.home();
    lcd.clear();
    lcd.print("Soft left");
  }
  else if (ir_l == line && ir_m == no_line && ir_r == no_line){
    // hard turn left
    control_motors(round(hardTurn_ratio * speedHardTurn), speedHardTurn);
    Serial.println("Hard turn left");
    lcd.home();
    lcd.clear();
    lcd.print("Hard left");
  }
  else if (ir_l == no_line && ir_m == line && ir_r == line){
    // soft turn right
    control_motors(speed, round(softTurn_ratio * speed));
    Serial.println("Soft turn right");
    lcd.home();
    lcd.clear();
    lcd.print("Soft right");
  }
  else if (ir_l == no_line && ir_m == no_line && ir_r == line){
    // hard turn right
    control_motors(speedHardTurn, round(hardTurn_ratio * speedHardTurn));
    Serial.println("Hard turn right");
    lcd.home();
    lcd.clear();
    lcd.print("Hard right");
  }
  else if (ir_l == no_line && ir_m == no_line && ir_r == no_line){
    // car is lost
    control_motors(0, 0);
    Serial.println("Backup, car is lost...");
    lcd.home();
    lcd.clear();
    lcd.print("Backup, car lost");
    delay(500);
    lcd.home();
    lcd.clear();
    lcd.print("Backing up...");
    back_on_track = Backup_for_line();
    if(!back_on_track){
    	lcd.home();
    	lcd.clear();
    	lcd.print("Completely lost...");
    	Serial.println("Car is completely lost...");
    	return false;
    }
  }
  else if (ir_l == line && ir_m == no_line && ir_r == line){
    // corrupt state
    control_motors(0, 0);
    Serial.println("ERROR, what is happening, is there a split in the line?");
    lcd.home();
    lcd.clear();
    lcd.print("ERROR split line");
    return false;
  }

  return true;
} 

bool Backup_for_line(){
	//Go in reverse till line is found, or till timeout

	int speed = -100;

	int line = 1; // defines the color of the line
	int no_line = 0;

	bool line_found = false;
	
	unsigned long timeout = 10000; //10s

	unsigned long time = millis();
	while((millis() - time < timeout) && !line_found){
		//Go backwards
		control_motors(speed,speed);
		delay(50);
		control_motors(0,0);

		//check for line
		ir_l = digitalRead(IR_LEFT);
		ir_m = digitalRead(IR_MIDDLE);
		ir_r = digitalRead(IR_RIGHT);

		if(ir_l == line || ir_m == line || ir_r == line){
			line_found = true;
		}
	}

	return line_found;
}

void control_motors(int left, int right)
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
