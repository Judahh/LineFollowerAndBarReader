/*
 *  Motor.h
 *
 */

// header defining the interface of the source.
#ifndef MOTOR_H
#define Motor_H

// include Arduino basic header.
#include <Arduino.h>

class Motor{
  public:
    Motor(int motorNegativeLeftPin, int motorPositiveLeftPin, int motorPositiveRightPin, int motorNegativeRightPin, byte intensity);
    void runBackwards();
    void runForward();
    void runBackwardsLeft();
    void runBackwardsRight();
    void runForwardLeft();
    void runForwardRight();
    void turnLeft();
    void turnRight();
    void brake();
    void neutral();
  private:
    byte intensity;
    int motorNegativeLeftPin, motorPositiveLeftPin, motorPositiveRightPin, motorNegativeRightPin;
    
}; 

#endif // MOTOR_H
