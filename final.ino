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
  Serial.println("[AVOID] Running avoidObstacle()");

  // Most important rule:
  // If line is seen ANYWHERE during avoidance, immediately return to tracking.
  if (lineDetected()) {
    Serial.println("[AVOID] Line detected during avoidance -> TRACKING");

    stopCar();
    delay(80);

    s.write(servoCenter);
    state = TRACKING;
    avoidState = AVOID_START;
    return;
  }

  switch (avoidState) {

    case AVOID_START:
      Serial.println("[AVOID] State: AVOID_START");

      stopCar();
      s.write(servoCenter);
      delay(150);

      chooseAvoidDirection();

      if (avoidDir == 1) {
        Serial.println("[AVOID] Moving RIGHT around obstacle");
        s.write(servoLeft); 
      }
      else {
        Serial.println("[AVOID] Moving LEFT around obstacle");
        s.write(servoRight);
      }

      delay(300);

      avoidState = AVOID_SIDE_STEP;
      break;


    case AVOID_SIDE_STEP:
      Serial.println("[AVOID] State: AVOID_SIDE_STEP");

      // Move away from obstacle until front is clear.
      if (obstacleAhead()) {
        if (avoidDir == 1) {
          Serial.println("[AVOID] Front blocked, moveRight()");
          moveRight();
        }
        else {
          Serial.println("[AVOID] Front blocked, moveLeft()");
          moveLeft();
        }

        delay(120);
        stopCar();
      }
      else {
        Serial.println("[AVOID] Front clear -> AVOID_FOLLOW_SIDE");

        stopCar();
        delay(100);

        avoidState = AVOID_FOLLOW_SIDE;
      }
      break;


    case AVOID_FOLLOW_SIDE:
      Serial.println("[AVOID] State: AVOID_FOLLOW_SIDE");

      // Servo is looking sideways at the obstacle.
      // If it still sees the object, keep moving forward.
      if (watch() < sideDistance) {
        Serial.println("[AVOID] Object still beside robot, moving forward");

        moveForward();
        delay(130);
        stopCar();
      }
      else {
        Serial.println("[AVOID] Side object lost, wrapping around corner");

        moveForward();
        delay(250);
        stopCar();

        if (avoidDir == 1) {
          Serial.println("[AVOID] Turning left around object edge");
          sharpLeftTurn(turnSpeed, turnSpeed);
        }
        else {
          Serial.println("[AVOID] Turning right around object edge");
          sharpRightTurn(turnSpeed, turnSpeed);
        }

        delay(300);
        stopCar();

        s.write(servoCenter);
        delay(150);

        avoidState = AVOID_SEARCH_LINE;
      }
      break;


    case AVOID_SEARCH_LINE:
      Serial.println("[AVOID] State: AVOID_SEARCH_LINE");

      // Search for the line while moving back toward original path.
      if (avoidDir == 1) {
        Serial.println("[AVOID] Searching line by moving left");
        moveLeft();
      }
      else {
        Serial.println("[AVOID] Searching line by moving right");
        moveRight();
      }

      delay(120);
      stopCar();

      // Do NOT switch state here unless lineDetected() happens.
      // The top of avoidObstacle() handles that immediately.
      break;
  }
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
