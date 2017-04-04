#include <Motor.h>
#include <QueueArray.h>
#include <math.h>

//Definicoes pinos Arduino ligados a entrada da Ponte H
int const motorNegativeLeftPin = 3;
int const motorPositiveLeftPin = 2;
int const motorPositiveRightPin = 4;
int const motorNegativeRightPin = 5;
int const motorSpeedRightPin = 10;
int const motorSpeedLeftPin = 9;

float intensityPercentage = 50;
int percentageMax = 100;
int intensityMax = 255;

float intensityP = intensityPercentage / percentageMax;
byte intensity = intensityP * intensityMax;

Motor motor(motorNegativeLeftPin, motorPositiveLeftPin, motorPositiveRightPin, motorNegativeRightPin, motorSpeedRightPin, motorSpeedLeftPin, intensity);

float intensityPercentageR = intensityPercentage * 1.5;

float intensityPR = intensityPercentageR / percentageMax;
byte intensityR = intensityPR * intensityMax;

int const leftSensorPin = 6;
int const centerSensorPin = 7;
int const rightSensorPin = 8;

int const serialBaudRate = 9600;

int const numberOfSamples = 10;
int const sampleDelay = 100;

bool bitValue = false;
bool bitLastValue = false;
bool bitFirstValue = true;


int leftSensorValue = 0;
int centerSensorValue = 0;
int rightSensorValue = 0;

int leftOffset = 0;
int centerOffset = 0;
int rightOffset = 0;

bool goLeft = false;
bool goRight = false;
bool ok = false;

bool warning = false;
bool error = false;
bool led = false;
bool finish = false;

int errorChecker = 10;
int errorNumber = 0;

unsigned long bitTime = 0;

unsigned long currentCounter = 0;
unsigned long counter = 0;

int bitPosition = -1;

int minBitTime = 35;

QueueArray <byte> queue;
QueueArray <bool> queueBitsFound;
byte byteReceived = 0;

void executeCommand(byte command) {
  Serial.print("Command:");
  Serial.println(command);
  switch (command) {
    case 0b00000000:
      Serial.println("DEAD!!!");
      //END
      motor.brake();
      led = true;
      finish = true;
      digitalWrite(LED_BUILTIN, HIGH);
      break;

    case 0b00000001:
      Serial.println("LED ON!!!");
      led = true;
      digitalWrite(LED_BUILTIN, led);
      break;

    case 0b00000010:
      Serial.println("LED OFF!!!");
      led = false;
      digitalWrite(LED_BUILTIN, led);
      break;

    case 0b00000011:
      Serial.println("LED SWITCH!!!");
      led = !led;
      digitalWrite(LED_BUILTIN, led);
      break;

    case 0b00000100:
      Serial.println("IF TURN!!!");
      if (led) {
        motor.runForwardRight();
      } else {
        motor.runForwardLeft();
      }
      delay(1000 / 10); //delay de 1/10 de segundo
      break;

    case 0b00000101:
      Serial.println("Delay 0.5s!!!");
      motor.neutral();
      delay(1000 / 2);
      break;

    case 0b00000110:
      Serial.println("Delay 1s!!!");
      motor.neutral();
      delay(1000);
      break;

    case 0b00000111:
      Serial.println("Delay 10s!!!");
      motor.neutral();
      delay(10000);
      break;

    case 0b00001000:
      Serial.println("Turn Left!!!");
      motor.turnLeft(intensityR);
      delay(1000 / 10);
      break;

    case 0b00001001:
      Serial.println("Turn Right!!!");
      motor.turnRight(intensityR);
      delay(1000 / 10);
      break;

    case 0b00001010:
      Serial.println("Run Forward!!!");
      motor.runForward();
      delay(1000 / 5);
      break;

    case 0b00001011:
      Serial.println("brake!!!");
      motor.brake();
      delay(1000 / 5);
      break;

    case 0b00001100:
      Serial.println("Run Backwards!!!");
      motor.runBackwards();
      delay(1000 / 5);
      break;

    case 0b10000000:
      Serial.println("Turn Right!!!");
      motor.turnRight(intensityR);
      delay(1000 / 10);
      break;

    default:
      break;
  }

  while (finish) {}
}

void executeCommands() {
  while (!queue.isEmpty()) {
    Serial.println("Command FOUND!!!");
    executeCommand(queue.dequeue());
  }
}

void checkBits() {
  while (queueBitsFound.count()>0) {
    bool newBit=queueBitsFound.dequeue();
    if(bitPosition==-1){
      if(newBit){
        Serial.println("P++");
        bitPosition++;
      }
    }else{
      if(bitPosition<8){
        Serial.print("recebido:");
        Serial.println((int)newBit);
        byteReceived = byteReceived | newBit;
        if (bitPosition < 7) {
          byteReceived = byteReceived << 1;
        }
        bitPosition++;
      }else{
        queue.enqueue(byteReceived);
        bitPosition=-1;
      }
    }
  }
}

void checkBar() {
  if (bitLastValue != bitValue) {
    currentCounter = millis();
    int currentBitTime = currentCounter - counter;
    Serial.print(bitValue);
    Serial.print(",");
    Serial.print(bitLastValue);
    Serial.print(",");
    Serial.println(currentBitTime);
    int numberOfBits = round(currentBitTime / bitTime);
    for(int index = 0; index < numberOfBits; index++){
      queueBitsFound.enqueue(bitLastValue);
      //Serial.print(bitLastValue);
    }
    if(!queueBitsFound.isEmpty()){
      checkBits();
    }
    counter = currentCounter;
  }
  bitLastValue = bitValue;
}

void checkBitTime() {
  if (bitLastValue != bitValue) {
    currentCounter = millis();
//    if(bitFirstValue){
//      bitFirstValue = false;
//    }
    if(bitLastValue && !bitValue){
      bitTime = currentCounter - counter;
      if(bitTime>=minBitTime){
        bitTime=bitTime-20;
        Serial.print("bitTime:");
        Serial.println(bitTime);
      }
    }
    counter = currentCounter;
  }
  bitLastValue = bitValue;
}

void setup() {
  Serial.begin(serialBaudRate);
  //Define os pinos como saida
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println(" ");
  Serial.println("START");
}

void loop() {
  leftSensorValue = digitalRead(leftSensorPin);
  centerSensorValue = digitalRead(centerSensorPin);
  rightSensorValue = digitalRead(rightSensorPin);
  int total=leftSensorValue + centerSensorValue + rightSensorValue;
  bitValue = (total > 2);

//  Serial.print("left:");
//  Serial.println(leftSensorValue);
//  Serial.print("right:");
//  Serial.println(rightSensorValue);

  if (leftSensorValue == 0 && rightSensorValue == 1) {//W?B
    goLeft = false;
    goRight = true;
    warning = false;
    error = false;
  }

  if (leftSensorValue == 1 && rightSensorValue == 0) {//B?W
    goLeft = true;
    goRight = false;
    warning = false;
    error = false;
  }

  if (leftSensorValue == 0 && rightSensorValue == 0 && centerSensorValue == 0) {//WWW
    if (goLeft || goRight) {
      warning = true;
    } else {
      error = true;
    }
  }

  if (centerSensorValue == 1){// && !goRight && !goLeft) {
    //ok
    goLeft = false;
    goRight = false;
    warning = false;
    error = false;
  }

  if (error) {
    motor.brake();
    digitalWrite(LED_BUILTIN, HIGH);
    led = true;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    led = false;
  }

  if (warning) {
    errorNumber--;
    if (errorNumber <= 0) {
      error = true;
    }
  }

  if (!goRight && !goLeft && !warning && !error) {
    //Serial.println("RUN!");
    motor.runForward();
  }

  if (goRight && !goLeft && !error) {
//    if (!warning) {
//      motor.runForwardRight(motor.getIntensity(100));
//    } else {
//      motor.turnRight();
//    }
    motor.turnRight(intensityR);
  }

  if (!goRight && goLeft && !error) {
//    if (!warning) {
//      motor.runForwardLeft(motor.getIntensity(100));
//    } else {
//      motor.turnLeft();
//    }
    motor.turnLeft(intensityR);
  }

  if (!warning && !error && (total>0) ) {
    if (bitTime < minBitTime) {
      checkBitTime();
    } else {
      checkBar();
    }
  }

  executeCommands();
}


