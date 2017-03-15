#include "Arduino.h"
#include "Motor.h"

Motor::Motor(int motorNegativeLeftPin, int motorPositiveLeftPin, int motorPositiveRightPin, int motorNegativeRightPin, byte intensity){
    pinMode(motorNegativeLeftPin, OUTPUT);
    pinMode(motorPositiveLeftPin, OUTPUT);
    pinMode(motorPositiveRightPin, OUTPUT);
    pinMode(motorNegativeRightPin, OUTPUT);

    this->intensity = intensity;
    this->motorNegativeLeftPin = motorNegativeLeftPin;
    this->motorPositiveLeftPin = motorPositiveLeftPin;
    this->motorPositiveRightPin = motorPositiveRightPin;
    this->motorNegativeRightPin = motorNegativeRightPin;
}

void Motor:: runBackwards(){
    digitalWrite(motorPositiveLeftPin, LOW);
    digitalWrite(motorPositiveRightPin, LOW);

    analogWrite(motorNegativeLeftPin, intensity);
    analogWrite(motorNegativeRightPin, intensity);
}

void Motor:: runForward(){
    digitalWrite(motorNegativeLeftPin, LOW);
    digitalWrite(motorNegativeRightPin, LOW);

    analogWrite(motorPositiveLeftPin, intensity);
    analogWrite(motorPositiveRightPin, intensity);
}

void Motor:: runBackwardsLeft(){
    digitalWrite(motorNegativeLeftPin, LOW);
    digitalWrite(motorPositiveLeftPin, LOW);
    digitalWrite(motorPositiveRightPin, LOW);

    analogWrite(motorNegativeRightPin, intensity);
}

void Motor:: runBackwardsRight(){
    digitalWrite(motorPositiveLeftPin, LOW);
    digitalWrite(motorPositiveRightPin, LOW);
    digitalWrite(motorNegativeRightPin, LOW);

    analogWrite(motorNegativeLeftPin, intensity);
}

void Motor:: runForwardLeft(){
    digitalWrite(motorNegativeLeftPin, LOW);
    digitalWrite(motorPositiveLeftPin, LOW);
    digitalWrite(motorNegativeRightPin, LOW);

    analogWrite(motorPositiveRightPin, intensity);
}

void Motor:: runForwardRight(){
    digitalWrite(motorNegativeLeftPin, LOW);
    digitalWrite(motorPositiveRightPin, LOW);
    digitalWrite(motorNegativeRightPin, LOW);

    analogWrite(motorPositiveLeftPin, intensity);
}

void Motor:: turnLeft(){
    digitalWrite(motorPositiveLeftPin, LOW);
    digitalWrite(motorNegativeRightPin, LOW);

    analogWrite(motorNegativeLeftPin, intensity);
    analogWrite(motorPositiveRightPin, intensity);
}

void Motor:: turnRight(){
    digitalWrite(motorNegativeLeftPin, LOW);
    digitalWrite(motorPositiveRightPin, LOW);

    analogWrite(motorPositiveLeftPin, intensity);
    analogWrite(motorNegativeRightPin, intensity);
}

void Motor:: brake(){
    digitalWrite(motorNegativeLeftPin, intensity);
    digitalWrite(motorPositiveLeftPin, intensity);
    digitalWrite(motorPositiveRightPin, intensity);
    digitalWrite(motorNegativeRightPin, intensity);
}

void Motor:: neutral(){
    digitalWrite(motorNegativeLeftPin, LOW);
    digitalWrite(motorPositiveLeftPin, LOW);
    digitalWrite(motorPositiveRightPin, LOW);
    digitalWrite(motorNegativeRightPin, LOW);
}