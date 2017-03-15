/*
 *  LineFollowerBarCodeReader.h
 *
 */

// header defining the interface of the source.
#ifndef LINEFOLLOWERBARCODEREADER_H
#define LineFollowerBarCodeReader_H

// include Arduino basic header.
#include <Arduino.h>
#include "../Motor/Motor.h"
#include <QueueArray.h>

class LineFollowerBarCodeReader{
  public:
    LineFollowerBarCodeReader(int barCodeSensorPin, Motor motor, bool led);
    void executeCommand(byte command);
    void executeCommands();
    void checkBar();
  private:
    int barCodeSensorPin; 
    unsigned long bitTime;
    unsigned long currentCounter;
    unsigned long counter;
    int bitPosition;
    int centerSensorLastValue;
    int centerSensorValue;
    bool led;
    byte byteReceived;
    QueueArray <byte> queue;
    Motor motor;
}; 

#endif // LINEFOLLOWERBARCODEREADER_H
