#include <Servo.h>
#include <Arduino.h>
//universal library

#include "variable.h"
#include "move.h"
//created library

move m;
Servo s;
//objects

int leftscanval, centerscanval, rightscanval, ldiagonalscanval, rdiagonalscanval;
int distance;
int numcycles = 0;
const int turntime = 250; //Time the robot spends turning (miliseconds)
const int backtime = 300; //Time the robot spends turning (miliseconds)
int thereis;

void setup(){
  s.attach(SERVO_PIN);
  //servo
  pinMode(Trig_PIN, output); 
  pinMode(Echo_PIN, input); 
  //soudn sensor
  pinMode(sensor1, input);
  pinMode(sensor2, input);
  pinMode(sensor3, input);
  pinMode(sensor4, input);
  pinMode(sensor5, input);
  //line sensor

  Serial.begin(9600);
}
//---------------------------------------------------------------------
void loop(){
  if(watch() < 20){
    auto_avoidance();
  }
  else{
    tracking();
  }
}
//---------------------------------------------------------------------
int watch(){
  long echo_distance;

  digitalWrite(Trig_PIN,LOW);
  delayMicroseconds(5);                                                                              
  digitalWrite(Trig_PIN,HIGH);
  delayMicroseconds(15);

  digitalWrite(Trig_PIN,LOW);
  echo_distance=pulseIn(Echo_PIN,HIGH);
  echo_distance=echo_distance*0.01657; //in cm

  return round(echo_distance);
}

void tracking()
{
  String senstr="";
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);
  int sensorvalue=32;
  sensorvalue +=s0*16+s1*8+s2*4+s3*2+s4;
  senstr= String(sensorvalue,BIN);
  senstr=senstr.substring(1,6);
  
  Serial.print(senstr);
  Serial.print("\t");
 
  if ( senstr=="10000" || senstr=="01000" || senstr=="11000")
   {
     Serial.println(" Shift Left");
      sharpLeftTurn(LOW_SPEED,MID_SPEED);
    //  left_shift(HIGH_SPEED,HIGH_SPEED,HIGH_SPEED,HIGH_SPEED);
      delay(DELAY_TIME);
      m.stop();     
   }
   
  if ( senstr=="11100" || senstr=="10100" )
  {
     Serial.println("Slight Shift Left");
      forward(0,HIGH_SPEED);
      delay(DELAY_TIME);
      m.stop(); 
  }
  if ( senstr=="01100" ||  senstr=="11110"  || senstr=="10010"  || senstr=="10110"  || senstr=="11010")
  {
     Serial.println("Slight Left");
      forward(LOW_SPEED,MID_SPEED);
      delay(DELAY_TIME);
  }
 if (senstr=="01110" || senstr=="01010" || senstr=="00100"  || senstr=="10001"  || senstr=="10101"  || senstr=="10011" || senstr=="11101" || senstr=="10111" || senstr=="11011"  || senstr=="11001")
  {
     Serial.println("Forward");
      forward(MID_SPEED,MID_SPEED);
      delay(DELAY_TIME);
       m.stop(); 
  }
 if ( senstr=="00110" || senstr=="01111" || senstr=="01001" || senstr=="01011" || senstr=="01101")
  {
        Serial.println("Slit Right");
      forward(MID_SPEED,LOW_SPEED);
      delay(DELAY_TIME);
       m.stop(); 
  }
 if (senstr=="00111" || senstr=="00101" )
  {    Serial.println("Slight Shift to Right ");
       forward(HIGH_SPEED,0);
      delay(DELAY_TIME);
      m.stop(); 
  }
 if (senstr=="00001" || senstr=="00010" || senstr=="00011")
 {
   Serial.println("Shift to Right");
   sharpRightTurn(MID_SPEED,LOW_SPEED);
    //  right_shift(HIGH_SPEED,HIGH_SPEED,HIGH_SPEED,HIGH_SPEED);
      delay(DELAY_TIME);
      m.stop();   
        
 }
  if (  senstr=="00000"){
        reverse(MID_SPEED);
   
      delay(DELAY_TIME/2*3);
      m.stop();  
  }
 if (  senstr=="11111")
 {
   Serial.println("Sharp Right U Turn");
      sharpRightTurn(MID_SPEED,MID_SPEED);
      delay(DELAY_TIME);
      m.stop();     
 }
}

void sharpRightTurn(int speed_left,int speed_right)
{
  m.Motor_FL(speed_left);
  m.Motor_BL(speed_left);
  m.Motor_FR(-1 * speed_right);
  m.Motor_BR(-1 * speed_right);
  /*
   RL_fwd(speed_left);
   RR_bck(speed_right);
   FR_bck(speed_right);
   FL_fwd(speed_left); 
   */
}
void sharpLeftTurn(int speed_left,int speed_right){
  m.Motor_FL(-1 * speed_left);
  m.Motor_BL(-1 * speed_left);
  m.Motor_FR(speed_right);
  m.Motor_BR(speed_right);
  /*
   RL_bck(speed_left);
   RR_fwd(speed_right);
   FR_fwd(speed_right);
   FL_bck(speed_left); 
   */
}
void forward(int speed_left,int speed_right)
{
  m.Motor_FL(speed_left);
  m.Motor_BL(speed_left);
  m.Motor_FR(speed_right);
  m.Motor_BR(speed_right); 
}
void reverse (int speed)
{
  m.Motor_FL(-1 * speed);
  m.Motor_BL(-1 * speed);
  m.Motor_FR(-1 * speed);
  m.Motor_BR(-1 * speed); 
}

void auto_avoidance(){

  ++numcycles;
  if(numcycles>=LPT){ //Watch if something is around every LPT loops while moving forward 
     m.stop();
    String obstacle_sign=watchsurrounding(); // 5 digits of obstacle_sign binary value means the 5 direction obstacle status
      Serial.print("begin str=");
        Serial.println(obstacle_sign);
                    if( obstacle_sign=="10000"){
     Serial.println("SLIT right");
          set_Motorspeed(FAST_SPEED,SPEED,FAST_SPEED,SPEED);
     m.Forward(100);
 
      delay(turntime);
      m.stop();
    }
        else    if( obstacle_sign=="00001"  ){
     Serial.println("SLIT LEFT");
       set_Motorspeed(SPEED,FAST_SPEED,SPEED,FAST_SPEED);
      m.Forward(100);
  
      delay(turntime);
      m.stop();
    }
    else if( obstacle_sign=="11100" || obstacle_sign=="01000" || obstacle_sign=="11000"  || obstacle_sign=="10100"  || obstacle_sign=="01100" ||obstacle_sign=="00100"  ||obstacle_sign=="01000" ){
     Serial.println("hand right");
	    //go_Right();
      m.turn(100, -100);
      set_Motorspeed(TURN_SPEED,TURN_SPEED,TURN_SPEED,TURN_SPEED);
      delay(turntime);
      m.stop();
    } 
    else if( obstacle_sign=="00010" || obstacle_sign=="00111" || obstacle_sign=="00011"  || obstacle_sign=="00101" || obstacle_sign=="00110" || obstacle_sign=="01010" ){
    Serial.println("hand left");
     //go_Left();//Turn left
     m.turn(-100, 100);
     set_Motorspeed(TURN_SPEED,TURN_SPEED,TURN_SPEED,TURN_SPEED);
      delay(turntime);
      m.stop();
    }
 
    else if(  obstacle_sign=="01111" ||  obstacle_sign=="10111" || obstacle_sign=="11111"  ){
    Serial.println("hand back left");
	  m.Forward(-100);
		set_Motorspeed( BACK_SPEED1,BACK_SPEED2,BACK_SPEED1,BACK_SPEED2);
       delay(backtime);
          m.stop();
        } 
         else if( obstacle_sign=="11011"  ||    obstacle_sign=="11101"  ||  obstacle_sign=="11110"  || obstacle_sign=="01110"  ){
    Serial.println("hand back right");
   m.Forward(-100);
    set_Motorspeed(BACK_SPEED2,BACK_SPEED1,BACK_SPEED2,BACK_SPEED1);
       delay(backtime);
          m.stop();
        }    
  
        else Serial.println("no handle");
    numcycles=0; //Restart count of cycles
  } else {
    Serial.println("moving forward");
     set_Motorspeed(SPEED,SPEED,SPEED,SPEED);
     //go_Advance();  // if nothing is wrong go forward using go() function above.
     m.Forward(100);
        delay(backtime);
          m.stop();
  }
  
  //else  Serial.println(numcycles);
  
  distance = watch(); // use the watch() function to see if anything is ahead (when the robot is just moving forward and not looking around it will test the distance in front)
  if (distance<distancelimit){ // The robot will just stop if it is completely sure there's an obstacle ahead (must test 25 times) (needed to ignore ultrasonic sensor's false signals)
 Serial.println("final go back");
	m.Forward(100);
  set_Motorspeed(BACK_SPEED1,BACK_SPEED2,BACK_SPEED1,BACK_SPEED2);
  delay(backtime);
      ++thereis;}
  if (distance>distancelimit){
      thereis=0;} //Count is restarted
  if (thereis > 25){
  Serial.println("final stop");
    m.stop(); // Since something is ahead, stop moving.
    thereis=0;
  }
}
//IR OBSTACLE SENSOR
// ===== IR Sensor =====
const int irPin = 2;


// ===== Speed =====
int speedPWM = 100;

// ===== Setup =====
void setup() {

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);

  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);

  pinMode(irPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
}

// ===== Movement Functions =====

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);

  analogWrite(ENA, speedPWM);
  analogWrite(ENB, speedPWM);
  analogWrite(ENC, speedPWM);
  analogWrite(END, speedPWM);
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);

  analogWrite(ENA, speedPWM);
  analogWrite(ENB, speedPWM);
  analogWrite(ENC, speedPWM);
  analogWrite(END, speedPWM);
}

void moveRight() {
  // Mecanum right movement
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // FL forward
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  // RL backward
  digitalWrite(IN5, LOW);  digitalWrite(IN6, HIGH);  // FR backward
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);   // RR forward

  analogWrite(ENA, speedPWM);
  analogWrite(ENB, speedPWM);
  analogWrite(ENC, speedPWM);
  analogWrite(END, speedPWM);
}

void stopCar() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  analogWrite(ENC, 0);
  analogWrite(END, 0);
}


// ===== Loop =====
void loop() {

  int sensorValue = digitalRead(irPin);

  // If no obstacle → move forward
  if (sensorValue == HIGH) {
    moveForward();
  }

  // If obstacle detected
  else {
    stopCar();

    beep();

    // Move backward for 1 second
    moveBackward();
    delay(1000);

    stopCar();
    delay(200);

    // Move right for 2 seconds
    moveRight();
    delay(2000);

    stopCar();
    delay(200);
  }
}
String watchsurrounding(){
/*  obstacle_status is a binary integer, its last 5 digits stands for if there is any obstacles in 5 directions,
 *   for example B101000 last 5 digits is 01000, which stands for Left front has obstacle, B100111 means front, right front and right ha
 */
 
int obstacle_status =B100000;
  centerscanval = watch();
  if(centerscanval<distancelimit){
    m.stop();
    
    obstacle_status  =obstacle_status | B100;
    }
  s.write(120);
  delay(100);
  ldiagonalscanval = watch();
  if(ldiagonalscanval<distancelimit){
    m.stop();
    
     obstacle_status  =obstacle_status | B1000;
    }
  s.write(170); //Didn't use 180 degrees because my servo is not able to take this angle
  delay(300);
  leftscanval = watch();
  if(leftscanval<sidedistancelimit){
    m.stop();
    
     obstacle_status  =obstacle_status | B10000;
    }

  s.write(90); //use 90 degrees if you are moving your servo through the whole 180 degrees
  delay(100);
  centerscanval = watch();
  if(centerscanval<distancelimit){
    m.stop();
    
    obstacle_status  = obstacle_status | B100;
    }
  s.write(40);
  delay(100);
  rdiagonalscanval = watch();
  if(rdiagonalscanval<distancelimit){
    m.stop();
    
    obstacle_status  = obstacle_status | B10;
    }
  s.write(0);
  delay(100);
  rightscanval = watch();
  if(rightscanval<sidedistancelimit){
    m.stop();
    
    obstacle_status  =obstacle_status | 1;
    }
  s.write(90); //Finish looking around (look forward again)
  delay(300);
   String obstacle_str= String(obstacle_status,BIN);
  obstacle_str= obstacle_str.substring(1,6);
  
  return obstacle_str; //return 5-character string standing for 5 direction obstacle status
}

void set_Motorspeed(int leftFront,int rightFront,int leftBack,int rightBack)
{
 analogWrite(speedPinL,leftFront); 
 analogWrite(speedPinR,rightFront); 
 analogWrite(speedPinLB,leftBack);  
 analogWrite(speedPinRB,rightBack);

}