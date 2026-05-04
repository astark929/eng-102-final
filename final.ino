#include "variable.h"
#include "move.h"
//personal libraries

#include <Arduino.h>
#include <Servo.h>
//global library

move m;
Servo s;
//objects



//int speedPWM = 75;

enum RobotState {
  TRACKING,
  LOST_LINE,
  AVOIDING
};

enum AvoidState {
  AVOID_START,
  AVOID_SIDE_STEP,
  AVOID_FOLLOW_SIDE,
  AVOID_SEARCH_LINE
};

RobotState state = TRACKING;
AvoidState avoidState = AVOID_START;

// -1 = left, +1 = right
int avoidDir = 1;

// ================= SETTINGS =================

int speedPWM = 85;

const int lineRecoverSpeed = 90;
const int trackSpeed = 100;
const int turnSpeed = 100;

const int obstacleDistance = 25;
const int sideDistance = 30;

const int servoCenter = 90;
const int servoLeft = 150;
const int servoRight = 30;

bool hasLeftLine;

unsigned long lastLineSeenTime = 0;
int lastLineDir = 0; 
// -1 = line was left
// +1 = line was right
//  0 = center/unknown

//RobotState state = TRACKING;

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

/*
void loop() {
  debugMain();

  if (state == TRACKING) {
    s.write(servoCenter);

    if (obstacleAhead()) {
      Serial.println("[MAIN] Obstacle detected -> AVOIDING");

      stopCar();
      delay(100);

      state = AVOIDING;
      avoidState = AVOID_START;
    }
    else if (!lineDetected()) {
      Serial.println("[MAIN] Line lost -> LOST_LINE");

      stopCar();
      delay(50);

      state = LOST_LINE;
    }
    else {
      tracking();
    }
  }

  else if (state == LOST_LINE) {
    s.write(servoCenter);

    if (obstacleAhead()) {
      Serial.println("[LOST_LINE] Obstacle detected -> AVOIDING");

      stopCar();
      delay(100);

      state = AVOIDING;
      avoidState = AVOID_START;
    }
    else if (lineDetected()) {
      Serial.println("[LOST_LINE] Line found -> TRACKING");

      stopCar();
      delay(50);

      state = TRACKING;
    }
    else {
      recoverLine();
    }
  }

  else if (state == AVOIDING) {
    avoidObstacle();
  }

  delay(30);
}
*/

/*
void loop(){
  if (state == TRACKING) { 
    if (obstacleAhead()) { 
      Serial.println("[LOOP] Obstacle detected -> AVOIDING"); 
      
      state = AVOIDING; 
      stopCar(); 
      delay(100); 
      } 
      else { 
        tracking(); 
        } 
        } 
        else 
        if (state == AVOIDING) { 
          avoidObstacle(); 
          // Only return when YOU decide inside avoidObstacle() 
          // (do NOT auto-switch here) }
}
*/

void loop() {
  if (state == TRACKING) {

    if (obstacleAhead()) {
      Serial.println("[LOOP] Obstacle detected -> AVOIDING");

      stopCar();
      delay(100);

      state = AVOIDING;
    }
    else if (!lineDetected()) {
      Serial.println("[LOOP] Line lost -> LOST_LINE");

      stopCar();
      delay(50);

      state = LOST_LINE;
    }
    else {
      tracking();
    }
  }

  else if (state == LOST_LINE) {
    if (obstacleAhead()) {
      Serial.println("[LOST_LINE] Obstacle detected -> AVOIDING");

      stopCar();
      delay(100);

      state = AVOIDING;
    }
    else if (lineDetected()) {
      Serial.println("[LOST_LINE] Line found -> TRACKING");

      stopCar();
      delay(50);

      state = TRACKING;
    }
    else {
      recoverLine();
    }
  }

  else if (state == AVOIDING) {
    avoidObstacle2();
  }

  delay(30);
}

// ================= TRACKING =================

void tracking() {
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);

  if (s0 || s1 || s2 || s3 || s4) {
    lastLineSeenTime = millis();

    if (s0 || s1) lastLineDir = -1;
    else if (s3 || s4) lastLineDir = 1;
    else lastLineDir = 0;
  }

  String senstr = "";
  int sensorvalue = 32;
  sensorvalue += s0 * 16 + s1 * 8 + s2 * 4 + s3 * 2 + s4;
  senstr = String(sensorvalue, BIN);
  senstr = senstr.substring(1, 6);

  Serial.print("[TRACKING] Sensors: ");
  Serial.println(senstr);

  if (senstr == "00100" || senstr == "01110" || senstr == "01100" || senstr == "00110") {
    Serial.println("[TRACKING] Forward");
    forward(trackSpeed, trackSpeed);
  }

  else if (senstr == "10000" || senstr == "11000" || senstr == "01000") {
    Serial.println("[TRACKING] Hard left");
    sharpLeftTurn(turnSpeed, turnSpeed);
  }

  else if (senstr == "11100" || senstr == "10100" || senstr == "01100") {
    Serial.println("[TRACKING] Slight left");
    forward(70, trackSpeed);
  }

  else if (senstr == "00001" || senstr == "00011" || senstr == "00010") {
    Serial.println("[TRACKING] Hard right");
    sharpRightTurn(turnSpeed, turnSpeed);
  }

  else if (senstr == "00111" || senstr == "00101" || senstr == "00110") {
    Serial.println("[TRACKING] Slight right");
    forward(trackSpeed, 70);
  }

  else if (senstr == "00000") {
    Serial.println("[TRACKING] Line lost");
    stopCar();
    state = LOST_LINE;
  }

  else {
    Serial.println("[TRACKING] Default forward");
    forward(trackSpeed, trackSpeed);
  }
}

// ================= LINE RECOVERY =================

void recoverLine() {
  Serial.println("[RECOVER] Searching for line");

  if (lastLineDir == -1) {
    Serial.println("[RECOVER] Last line was left, turning left");
    sharpLeftTurn(lineRecoverSpeed, lineRecoverSpeed);
  }
  else if (lastLineDir == 1) {
    Serial.println("[RECOVER] Last line was right, turning right");
    sharpRightTurn(lineRecoverSpeed, lineRecoverSpeed);
  }
  else {
    Serial.println("[RECOVER] Unknown direction, slow right spin");
    sharpRightTurn(lineRecoverSpeed, lineRecoverSpeed);
  }

  delay(80);
  stopCar();
}

// ================= OBSTACLE AVOIDANCE =================

void avoidObstacle() {
  Serial.println("===== AVOID OBJECT =====");

  stopCar();
  delay(150);

  // 1. Choose side using lidars
  int leftLidar  = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  if (leftLidar == LOW && rightLidar == HIGH) {
    avoidDir = 1;   // object more left, go right
  }
  else if (rightLidar == LOW && leftLidar == HIGH) {
    avoidDir = -1;  // object more right, go left
  }
  else {
    avoidDir = 1;   // default right
  }

  // 2. Sidestep until front is clear
  while (obstacleAhead()) {

  if (!lineDetected()) {
    hasLeftLine = true;
  }
  //checks if it left the line it detected the object from

  if (hasLeftLine && lineDetected()) {
    Serial.println("[AVOID] Line found again after leaving it");
    state = TRACKING;
    s.write(90);
    stopCar();
    return;
  }
  //once it knows it left the line, and detects a line, it breaks the while 
  //loop and continue with this code

  if (avoidDir == 1) {
    Serial.println("[AVOID] Sliding right");
    moveRight();
  } 
  else {
    Serial.println("[AVOID] Sliding left");
    moveLeft();
  }
  //moves left or right depending on where it sees the object at 

  delay(150);
  stopCar();
  delay(50);
  }

  // 3. Turn servo toward object
  if (avoidDir == 1) {
    s.write(170); // look left
  } 
  else {
    s.write(10);  // look right
  }

  delay(300);
  m.Forward(100);
  delay(500);
  m.stop();
  delay(50);

  // 4. Follow the side of the object
  while (watch() < sidedistancelimit) {
    //uses the ultrasonic sensor to see the object
    //condition only applies when robot sees the object

    if (lineDetected()) {
      state = TRACKING;
      s.write(90);
      stopCar();
      return;
    }
    //returns servo to initial condition when line is detected 

    Serial.println("[AVOID] Following object side");

    // Use lidar object-following behavior
    int r = digitalRead(RightObstacleSensor);
    int l = digitalRead(LeftObstacleSensor);

    if (l == LOW && r == LOW) {
      moveForward();
    }
    else if (l == LOW && r == HIGH) {
      sharpRightTurn(TURN_SPEED, TURN_SPEED);
    }
    else if (l == HIGH && r == LOW) {
      sharpLeftTurn(TURN_SPEED, TURN_SPEED);
    }
    else {
      moveForward();
    }
    

    //delay(120);
    //stopCar();
    delay(30);
  }

  // 5. Object side ended, wrap around corner
  Serial.println("[AVOID] Object edge reached");

  moveForward();
  delay(300);
  stopCar();

  if (avoidDir == 1) {
    sharpLeftTurn(TURN_SPEED, TURN_SPEED);
  } else {
    sharpRightTurn(TURN_SPEED, TURN_SPEED);
  }

  delay(350);
  stopCar();

  s.write(90);
  delay(150);

  // 6. Move forward after corner while looking for line
  while (!lineDetected()) {
    Serial.println("[AVOID] Moving after corner, searching line");

    // keep avoiding if object appears again
    if (obstacleAhead()) {
      if (avoidDir == 1) moveRight();
      else moveLeft();
    } 
    else {
      moveForward();
    }

    delay(130);
    stopCar();
    delay(40);
  }

  Serial.println("[AVOID] Line found -> TRACKING");

  stopCar();
  s.write(90);
  state = TRACKING;
}

void avoidObstacle1() {
  Serial.println("===== AVOID OBJECT =====");

  bool hasLeftLine = false;

  stopCar();
  delay(150);

  // Pick avoid direction
  int leftLidar  = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  if (leftLidar == LOW && rightLidar == HIGH) {
    avoidDir = 1;    // go right, object will be on left
  }
  else if (rightLidar == LOW && leftLidar == HIGH) {
    avoidDir = -1;   // go left, object will be on right
  }
  else {
    avoidDir = 1;    // default go right
  }

  // Move off the original line first
  Serial.println("[AVOID] Leaving original line");

  while (lineDetected()) {
    if (avoidDir == 1) {
      moveRight();
    } else {
      moveLeft();
    }

    delay(120);
    stopCar();
    delay(30);
  }

  hasLeftLine = true;

  // Point ultrasonic toward the object
  if (avoidDir == 1) {
    s.write(170); // looking left
  } else {
    s.write(10);  // looking right
  }

  delay(300);

  // Main avoidance loop
  while (true) {
    int sideDist = watch();

    Serial.print("[AVOID] Side distance: ");
    Serial.println(sideDist);

    // If line is found again after leaving original line, exit avoidance
    if (hasLeftLine && lineDetected()) {
      Serial.println("[AVOID] New line found -> TRACKING");

      stopCar();
      s.write(90);
      delay(100);

      state = TRACKING;
      return;
    }

    // Emergency: object directly in front
    if (obstacleAhead()) {
      Serial.println("[AVOID] Front blocked, sidestepping");

      if (avoidDir == 1) {
        moveRight();
      } else {
        moveLeft();
      }

      delay(150);
      stopCar();
      delay(40);
      continue;
    }

    // Object is too close on the side: move away slightly
    if (sideDist < 5) {
      Serial.println("[AVOID] Too close to object, moving away");

      if (avoidDir == 1) {
        moveRight();
      } else {
        moveLeft();
      }

      delay(120);
      stopCar();
      delay(40);
    }

    // Good distance: move forward along object
    else if (sideDist >= 5 && sideDist <= sidedistancelimit) {
      Serial.println("[AVOID] Following object side");

      moveForward();
      delay(140);
      stopCar();
      delay(40);
    }

    // Object disappeared from side: turn toward the object side to wrap corner
    else {
      Serial.println("[AVOID] Object side lost, turning toward object");

      if (avoidDir == 1) {
        // robot went right, object is on left, so turn left
        sharpLeftTurn(TURN_SPEED, TURN_SPEED);
      } else {
        // robot went left, object is on right, so turn right
        sharpRightTurn(TURN_SPEED, TURN_SPEED);
      }

      delay(180);
      stopCar();
      delay(40);
    }
  }
  //while statement bracket (i loose track of this one a lot)


}

void avoidObstacle2() {
  Serial.println("===== AVOID OBJECT 1 =====");

  stopCar();
  delay(150);

  // Choose avoid direction
  int leftLidar  = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  if (leftLidar == LOW && rightLidar == HIGH) {
    avoidDir = 1;    // move right, object is left
  }
  else if (rightLidar == LOW && leftLidar == HIGH) {
    avoidDir = -1;   // move left, object is right
  }
  else {
    avoidDir = 1;    // default right
  }

  // ============================
  // WHILE LOOP 1: escape obstacle/line
  // ============================
  while (obstacleAhead() || lineDetected()) {
    Serial.println("[AVOID] Escaping line/front obstacle");

    if (avoidDir == 1) {
      moveRight();
    } else {
      moveLeft();
    }

    delay(130);
    stopCar();
    delay(40);
  }

  // Move forward a bit after clear
  Serial.println("[AVOID] Clear space found, moving forward");

  moveForward();
  delay(350);
  stopCar();
  delay(100);

  // Turn servo toward object side
  if (avoidDir == 1) {
    s.write(170);    // moved right, object is left
  } else {
    s.write(10);   // moved left, object is right
  }

  delay(300);

  // ============================
  // WHILE LOOP 2: follow object/search line
  // ============================
  while (!lineDetected()) {
    int sideDistance = watch();

    Serial.print("[AVOID] Side distance: ");
    Serial.println(sideDistance);

    if (leftLidar == LOW || rightLidar == LOW) {
      Serial.println("[AVOID] Front blocked, sidestepping");

      if (avoidDir == 1) {
        moveRight();
      } else {
        moveLeft();
      }
    }
    //if lidar detects something in front
    //it will try to move sideways

    // If ultrasonic sees object, move forward along it
    else if (sideDistance < 10) {
      Serial.println("[AVOID] Following object side");

      moveForward();
    }

    // If lidar and ultrasonic see nothing, object edge is gone
    else {
      Serial.println("[AVOID] Object lost, turning toward object side");

      if (avoidDir == 1) {
        sharpLeftTurn(TURN_SPEED, TURN_SPEED);
      } else {
        sharpRightTurn(TURN_SPEED, TURN_SPEED);
      }

      delay(250);
      stopCar();
      delay(80);

      moveForward();
    }

    delay(130);
    stopCar();
    delay(40);
  }

  // Line found
  Serial.println("[AVOID] Line found -> TRACKING");

  stopCar();
  s.write(servoCenter);
  state = TRACKING;
}

// ================= SENSOR HELPERS =================

bool obstacleAhead() {
  int distance = watch();

  int leftLidar = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  if (distance > 0 && distance < obstacleDistance) return true;
  if (leftLidar == LOW || rightLidar == LOW) return true;

  return false;
}

bool lineDetected() {
  int s0 = !digitalRead(sensor1);
  int s1 = !digitalRead(sensor2);
  int s2 = !digitalRead(sensor3);
  int s3 = !digitalRead(sensor4);
  int s4 = !digitalRead(sensor5);

  return s0 || s1 || s2 || s3 || s4;
}

void chooseAvoidDirection() {
  int leftLidar = digitalRead(LeftObstacleSensor);
  int rightLidar = digitalRead(RightObstacleSensor);

  Serial.print("[AVOID] Left lidar: ");
  Serial.println(leftLidar);

  Serial.print("[AVOID] Right lidar: ");
  Serial.println(rightLidar);

  if (leftLidar == LOW && rightLidar != LOW) {
    avoidDir = 1;
  }
  else if (rightLidar == LOW && leftLidar != LOW) {
    avoidDir = -1;
  }
  else {
    // Default direction if both see object.
    avoidDir = 1;
  }

  Serial.print("[AVOID] avoidDir = ");
  Serial.println(avoidDir);
}

int watch() {
  long duration;

  digitalWrite(Trig_PIN, LOW);
  delayMicroseconds(5);

  digitalWrite(Trig_PIN, HIGH);
  delayMicroseconds(15);

  digitalWrite(Trig_PIN, LOW);

  duration = pulseIn(Echo_PIN, HIGH, 25000);

  if (duration == 0) {
    return 999;
  }

  int distance = duration * 0.0343 / 2;
  return distance;
}

// ================= DEBUG =================

void debugMain() {
  Serial.println("========== MAIN DEBUG ==========");

  Serial.print("Robot State: ");
  if (state == TRACKING) Serial.println("TRACKING");
  else if (state == LOST_LINE) Serial.println("LOST_LINE");
  else if (state == AVOIDING) Serial.println("AVOIDING");

  Serial.print("Avoid State: ");
  if (avoidState == AVOID_START) Serial.println("AVOID_START");
  else if (avoidState == AVOID_SIDE_STEP) Serial.println("AVOID_SIDE_STEP");
  else if (avoidState == AVOID_FOLLOW_SIDE) Serial.println("AVOID_FOLLOW_SIDE");
  else if (avoidState == AVOID_SEARCH_LINE) Serial.println("AVOID_SEARCH_LINE");

  Serial.print("Ultrasonic: ");
  Serial.println(watch());

  Serial.print("Left Lidar: ");
  Serial.println(digitalRead(LeftObstacleSensor));

  Serial.print("Right Lidar: ");
  Serial.println(digitalRead(RightObstacleSensor));

  Serial.print("Line Detected: ");
  Serial.println(lineDetected() ? "YES" : "NO");

  Serial.print("Line Sensors: ");
  printLineSensors();

  Serial.println("================================");
}

void printLineSensors() {
  Serial.print(!digitalRead(sensor1));
  Serial.print(" ");
  Serial.print(!digitalRead(sensor2));
  Serial.print(" ");
  Serial.print(!digitalRead(sensor3));
  Serial.print(" ");
  Serial.print(!digitalRead(sensor4));
  Serial.print(" ");
  Serial.println(!digitalRead(sensor5));
}

// ================= MOTOR FUNCTIONS =================

void sharpRightTurn(int speed_left, int speed_right) {
  m.Motor_FL(speed_left);
  m.Motor_BL(speed_left);
  m.Motor_FR(-1 * speed_right);
  m.Motor_BR(-1 * speed_right);
}

void sharpLeftTurn(int speed_left, int speed_right) {
  m.Motor_FL(-1 * speed_left);
  m.Motor_BL(-1 * speed_left);
  m.Motor_FR(speed_right);
  m.Motor_BR(speed_right);
}

void forward(int speed_left, int speed_right) {
  m.Motor_FL(speed_left);
  m.Motor_BL(speed_left);
  m.Motor_FR(speed_right);
  m.Motor_BR(speed_right);
}

void reverse(int speed) {
  m.Motor_FL(-1 * speed);
  m.Motor_BL(-1 * speed);
  m.Motor_FR(-1 * speed);
  m.Motor_BR(-1 * speed);
}

void moveForward() {
  forward(speedPWM, speedPWM);
}

void moveBackward() {
  reverse(speedPWM);
}

void moveLeft() {
  m.Motor_FL(-speedPWM);
  m.Motor_BL(speedPWM);
  m.Motor_FR(speedPWM);
  m.Motor_BR(-speedPWM);
}

void moveRight() {
  m.Motor_FL(speedPWM);
  m.Motor_BL(-speedPWM);
  m.Motor_FR(-speedPWM);
  m.Motor_BR(speedPWM);
}

void stopCar() {
  m.stop();
}
