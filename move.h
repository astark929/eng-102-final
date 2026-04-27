#ifndef MOVE_H
#define MOVE_H
#pragma once

#include "variable.h"

class move {
  public:
    move::move();

    void stopArdumoto(byte motor);
    void stop();  
    void Forward(int speed);
    void turn(int ls, int rs);
    void Motor_FR(int motorSpeed);
    void Motor_FL(int motorSpeed);
    void Motor_BR(int motorSpeed);
    void Motor_BL(int motorSpeed);


  private:
    //#define FORWARD
    //#define REVERSE

};
#endif


