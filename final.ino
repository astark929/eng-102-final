#include "variable.h"
#include "move.h"

move m;
Servo s;



int speedPWM = 75;

enum AvoidState {
  CHOOSE_DIR,
  SLIDE,
  FORWARD_CLEAR,
  SCAN_EDGE,
  RETURN_PATH
};
enum RobotState {
  TRACKING,
  AVOIDING
};

AvoidState avoidState = CHOOSE_DIR;
int avoidDir = 0; // -1 = left, +1 = right

RobotState state = TRACKING;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ENC, OUTPUT);
  pinMode(END, OUTPUT);
  //speed control pins

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(IN7, OUTPUT);
  pinMode(IN8, OUTPUT);
  //motor pins

  pinMode(irLeft,  INPUT);
  pinMode(irRight, INPUT);
  //lidar pins

  pinMode(Trig_PIN, OUTPUT);
  pinMode(Echo_PIN, INPUT);
  //ultrasonic pins

  s.attach(SERVO_PIN);
  //servo pin

  s.write(90); // center

  Serial.begin(9600);
}

void loop() {
  // /*
  if(digitalRead(LeftObstacleSensor) == low || digitalRead(RightObstacleSensor) == low){
    //detects object with lidar 

    Serial.println("object detected");
    avoidObstacle();
    //m.stop();
  }
  else{
    Serial.println("object not detected");
    tracking();
  }
  //*/
  //tracking();
  //avoidObstacle();
  //moveRight();
  //m.Forward(100);
  //m.turn(-100, 100);
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

bool obstacleDetected() {
  int dist = watch();  // ultrasonic

  int leftLidar  = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  if (dist < 10) return true;

  if (leftLidar == LOW || rightLidar == LOW) return true;

  return false;
}

bool lineDetected() {
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);

  return (s0 || s1 || s2 || s3 || s4);
}

void avoidObstacle() {

  int dist = watch();
  int leftLidar  = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  switch (avoidState) {

    // 1. Decide direction
    case CHOOSE_DIR:
      if (leftLidar == LOW) {
        avoidDir = 1; // go right
      } 
      else {
        avoidDir = -1; // go left
      }
      avoidState = SLIDE;
      break;

    // 2. Move sideways until clear
    case SLIDE:
      if (dist < distancelimit || leftLidar == LOW || rightLidar == LOW) {
        if (avoidDir == -1) moveLeft();
        else moveRight();
      } 
      else {
        stopCar();
        delay(100);
        avoidState = FORWARD_CLEAR;
      }
      break;

    // 3. Move forward past object
    case FORWARD_CLEAR:
      moveForward();
      delay(100);
      stopCar();

      // turn ultrasonic toward object
      if (avoidDir == -1) {
        s.write(150); // looking right
      } 
      else {
        s.write(30); // looking left
      }
      avoidState = SCAN_EDGE;
      break;

    // 4. Follow object edge
    case SCAN_EDGE:
      if (watch() < distancelimit) {
        moveForward();
      } 
      else {
        stopCar();
        avoidState = RETURN_PATH;
        moveForward();
        delay(200);
        stopCar();
      }
      break;

    // 5. Move back toward original path
    case RETURN_PATH:
      if (avoidDir == -1) moveRight();
      else moveLeft();

      delay(400);
      stopCar();

      avoidState = CHOOSE_DIR;
      state = TRACKING; // hand control back
      break;
  }
}

bool obstacleAhead(){
  int distance = watch();
  int lidarleft = digitalRead(RightObstacleSensor);
  int lidarright = digitalRead(LeftObstacleSensor);

  if(distance < distancelimit || lidarleft == LOW || lidarright == LOW){
    return true;
  }
  return false;
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

void moveRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW);  digitalWrite(IN6, HIGH);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);

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
