#pragma once


#include <SoftwareSerial.h>
#include <stdbool.h>
#include <Servo.h>
#include <Arduino.h>


#include <WiFiEsp.h>
#include <WiFiEspUDP.h>

#define SPEED 100
#define TURN_SPEED 100

#define speedPinR 9
#define RightMotorDirPin1 22
#define RightMotorDirPin2 24
#define LeftMotorDirPin1 26
#define LeftMotorDirPin2 28
#define speedPinL 10

#define speedPinRB 11
#define RightMotorDirPin1B 5
#define RightMotorDirPin2B 6
#define LeftMotorDirPin1B 7
#define LeftMotorDirPin2B 8
#define speedPinLB 12

//#
/*
motor pins above
----------------------------------------------
sensor pins below
*/

#define Echo_PIN    31 // Ultrasonic Echo pin connect to A5
#define Trig_PIN    30  // Ultrasonic Trig pin connect to A4
// ultrasonic sensor

#define sensor1   A4 // Left most sensor
#define sensor2   A3 // 2nd Left   sensor
#define sensor3   A2 // center sensor
#define sensor4   A1 // 2nd right sensor// Right most sensor
#define sensor5   A0 // Right most sensor
// the line detector

#define servopin 13



/*
---------------------------------------------
othervariables need
*/

#define SERVO_PIN     13  //servo connect to D5

#define LPT 2 // scan loop coumter

#define FAST_SPEED  160   //both sides of the motor speed
#define SPEED  120     //both sides of the motor speed
#define TURN_SPEED  120   //both sides of the motor speed
#define BACK_SPEED1  160   //back speed
#define BACK_SPEED2  90    //back speed

#define MID_SPEED 100    
#define HIGH_SPEED 120    
#define LOW_SPEED 80    
#define LONG_DELAY_TIME 70 
#define DELAY_TIME 40 

#define SHORT_DELAY_TIME 30 

const int distancelimit = 30; //distance limit for obstacles in front           
const int sidedistancelimit = 30; //minimum distance in cm to obstacles at both sides (the car will allow a shorter distance sideways)

const uint8_t output  = OUTPUT;
const uint8_t input  = INPUT;
const uint8_t high = HIGH;
const uint8_t low  = LOW;


/*
---------------------------------------------------
other stuff that may be useful later
*/

const int red = 5;
const int blue = 10;
const int green = 6;


#define RightObstacleSensor 37  //Right lidar
#define LeftObstacleSensor 36   //Left lidar
//lidar sensors


//the stuff below is the bluetooth stuff
#define MAX_PACKETSIZE 32    //Serial receive buffer
extern char buffUART[MAX_PACKETSIZE];
extern unsigned int buffUARTIndex;
extern unsigned long preUARTTick;
struct car_status {
  int speed;
  int angle;
  int direct;
};


//-------------------------------------------------------

//this is the wifi stuff
extern char ssid[];
extern char pass[];
extern int status;
extern char packetBuffer[5];
extern WiFiEspUDP Udp;
extern unsigned int localPort;


