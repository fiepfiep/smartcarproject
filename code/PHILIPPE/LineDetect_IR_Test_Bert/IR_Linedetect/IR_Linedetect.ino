const int IR_LEFT = 2;
const int IR_MIDDLE = 4;
const int IR_RIGHT = 3;

/*%%%%%%%%%%%%%%%%%%%%%%%%NOTE%%%%%%%%%%%%%%%%%%%%%%%%*/
/*you can change the sensitivity of the IR sensors 
  by turning the potentiometer!
*/


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_RIGHT, INPUT);

  //Header
  Serial.print("LEFT\t");
  Serial.print("MIDDlE\t");
  Serial.println("RIGHT");

}

void loop() {
  // put your main code here, to run repeatedly:
  int value_left = digitalRead(IR_LEFT);
  int value_middle = digitalRead(IR_MIDDLE);
  int value_right = digitalRead(IR_RIGHT);

  //Show results
  printResults(value_left, value_middle, value_right);

  //Wait time
  delay(500);
  

}

void printResults(int val1, int val2, int val3){
  //Content
  Serial.print(val1);
  Serial.print("\t");
  Serial.print(val2);
  Serial.print("\t");
  Serial.println(val3);
}
