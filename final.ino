#include "variable.h"
#include "move.h"

move m;

const int ENA = 11;
const int ENB = 12;
const int ENC = 9;
const int END = 10;

const int IN1 = 5;
const int IN2 = 6;

const int IN3 = 7;
const int IN4 = 8;

const int IN5 = 22;
const int IN6 = 24;

const int IN7 = 26;
const int IN8 = 28;

const int irLeft  = 36;
const int irRight = 37;

int speedPWM = 75;

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

  pinMode(irLeft,  INPUT);
  pinMode(irRight, INPUT);

  Serial.begin(9600);
}

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

void moveLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW);  digitalWrite(IN8, HIGH);

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

  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
}

void loop() {
  int left  = digitalRead(irLeft);
  int right = digitalRead(irRight);

  if (left == LOW && right == LOW) {
    stopCar();
    delay(200);

    moveBackward();
    delay( 500);

    stopCar();
    delay(200);

    moveLeft();
    delay(1000);

    stopCar();
    delay(200);

  }
  else {
    tracking();
  }
}
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
      forward(0, HIGH_SPEED);
      delay(DELAY_TIME);
      //m.stop(); 
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
      // m.stop(); 
  }
 if ( senstr=="00110" || senstr=="01111" || senstr=="01001" || senstr=="01011" || senstr=="01101")
  {
        Serial.println("Slit Right");
      forward(MID_SPEED,LOW_SPEED);
      delay(DELAY_TIME);
      // m.stop(); 
  }
 if (senstr=="00111" || senstr=="00101" )
  {    Serial.println("Slight Shift to Right ");
       forward(HIGH_SPEED,0);
      delay(DELAY_TIME);
     // m.stop(); 
  }
 if (senstr=="00001" || senstr=="00010" || senstr=="00011")
 {
   Serial.println("Shift to Right");
   sharpRightTurn(MID_SPEED,LOW_SPEED);
    //  right_shift(HIGH_SPEED,HIGH_SPEED,HIGH_SPEED,HIGH_SPEED);
      delay(DELAY_TIME);
     // m.stop();   
        
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
     // m.stop();     
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
