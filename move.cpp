#include "move.h"

#define forward 1
#define reverse 0

using namespace std;

move::move()
{
  pinMode(RightMotorDirPin1, output); 
  pinMode(RightMotorDirPin2, output); 
  pinMode(speedPinL, output);  
 
  pinMode(LeftMotorDirPin1, output);
  pinMode(LeftMotorDirPin2, output); 
  pinMode(speedPinR, output);

  pinMode(RightMotorDirPin1B, output); 
  pinMode(RightMotorDirPin2B, output); 
  pinMode(speedPinLB, output);  
 
  pinMode(LeftMotorDirPin1B, output);
  pinMode(LeftMotorDirPin2B, output); 
  pinMode(speedPinRB, output);
}


// stopArdumoto makes a motor stop
void move::stopArdumoto(byte motor)
{
  digitalWrite(RightMotorDirPin1, forward); 
  digitalWrite(RightMotorDirPin2, forward); 
 
  digitalWrite(LeftMotorDirPin1, forward);
  digitalWrite(LeftMotorDirPin2, forward); 

  digitalWrite(RightMotorDirPin1B, forward); 
  digitalWrite(RightMotorDirPin2B, forward); 
 
  digitalWrite(LeftMotorDirPin1B, forward);
  digitalWrite(LeftMotorDirPin2B, forward);

  analogWrite(speedPinR, 0);
  analogWrite(speedPinL, 0);
  analogWrite(speedPinRB, 0);
  analogWrite(speedPinLB, 0);
  
}

void move::stop(){
  Motor_FR(0);
  Motor_FL(0);
  Motor_BR(0);
  Motor_BL(0);

}

void move::Forward(int speed){
  Motor_FR(speed);
  Motor_FL(speed);
  Motor_BR(speed);
  Motor_BL(speed);
}

void move::turn(int ls, int rs){
  Motor_FR(rs);
  Motor_FL(ls);
  Motor_BR(rs);
  Motor_BL(ls);
}
void move::Motor_FR(int motorSpeed)                          
{
  if (motorSpeed > 0)                                 
  {
    digitalWrite(RightMotorDirPin1, forward);                      
    digitalWrite(RightMotorDirPin2, low);
    analogWrite(speedPinR, abs(motorSpeed));
  }
  else if (motorSpeed < 0)                            
  {
    digitalWrite(RightMotorDirPin1, reverse);                      
    digitalWrite(RightMotorDirPin2, high);
    analogWrite(speedPinR, abs(motorSpeed));
  }
  else                                               
  {
    digitalWrite(RightMotorDirPin1, 0);
    digitalWrite(RightMotorDirPin2, 0);
    analogWrite(speedPinR, 0);
   }                 
}

void move::Motor_FL(int motorSpeed)                           
{
  if (motorSpeed > 0)                                  
  {
    digitalWrite(LeftMotorDirPin1, forward);                       
    digitalWrite(LeftMotorDirPin2, low);
    analogWrite(speedPinL, abs(motorSpeed)); 
    
  }
  else if (motorSpeed < 0)                             
  {
    digitalWrite(LeftMotorDirPin1, reverse);                      
    digitalWrite(LeftMotorDirPin2, forward);
    analogWrite(speedPinL, abs(motorSpeed)); 
  }
  else                                                
  {
    digitalWrite(LeftMotorDirPin1, 0);                              
    digitalWrite(LeftMotorDirPin2, 0);
    analogWrite(speedPinL, 0);
  }
                  

}
void move::Motor_BR(int motorSpeed)                          
{
  if (motorSpeed > 0)                                  
  {
    digitalWrite(RightMotorDirPin1B, forward);                       
    digitalWrite(RightMotorDirPin2B, reverse); 
    analogWrite(speedPinRB, abs(motorSpeed));
  }
  else if (motorSpeed < 0)                             
  {
    digitalWrite(RightMotorDirPin1B, reverse);                       
    digitalWrite(RightMotorDirPin2B, forward);
    analogWrite(speedPinRB, abs(motorSpeed));
  }
  else                                                
  {
    digitalWrite(RightMotorDirPin1B, 0);                              
    digitalWrite(RightMotorDirPin2B, 0);
    analogWrite(speedPinRB, 0);
  }               

}

void move::Motor_BL(int motorSpeed)                           
{
  if (motorSpeed > 0)                                  
  {
    digitalWrite(LeftMotorDirPin1B, forward);                       
    digitalWrite(LeftMotorDirPin2B, reverse);
    analogWrite(speedPinLB, abs(motorSpeed)); 
  }
  else if (motorSpeed < 0)                             
  {
    digitalWrite(LeftMotorDirPin1B, reverse);                       
    digitalWrite(LeftMotorDirPin2B, forward);
    analogWrite(speedPinLB, abs(motorSpeed)); 
  }
  else                                               
  {
    digitalWrite(LeftMotorDirPin1B, 0);                             
    digitalWrite(LeftMotorDirPin2B, 0);
    analogWrite(speedPinLB, 0);
  }
                  

}