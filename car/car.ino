#include <Motor.h>
#include <QueueArray.h>
#include <math.h>

//Definicoes pinos Arduino ligados a entrada da Ponte H
int const motorNegativeLeftPin = 2;
int const motorPositiveLeftPin = 3;
int const motorPositiveRightPin = 5;
int const motorNegativeRightPin = 4;
int const motorSpeedRightPin = 10;
int const motorSpeedLeftPin = 9;

float intensityPercentage = 25;
int percentageMax = 100;
int intensityMax = 255;

float intensityP = intensityPercentage / percentageMax;
byte intensity = intensityP * intensityMax;

Motor motor(motorNegativeLeftPin, motorPositiveLeftPin, motorPositiveRightPin, motorNegativeRightPin, motorSpeedRightPin, motorSpeedLeftPin, intensity);

float intensityPercentageR = 50;

float intensityPR = intensityPercentageR / percentageMax;
byte intensityR = intensityPR * intensityMax;

int const leftSensorPin = 6;
int const centerSensorPin = 7;
int const rightSensorPin = 8;

int const serialBaudRate = 115200;

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

QueueArray <byte> queue;
byte byteReceived = 0;

void executeCommand(byte command) {
  Serial.print("Command:");
  Serial.println(command);
  switch (command) {
    case 0b10000000:
      Serial.println("DEAD!!!");
      //END
      motor.brake();
      led = true;
      finish = true;
      digitalWrite(LED_BUILTIN, HIGH);
      break;

    case 0b10000001:
      Serial.println("LED ON!!!");
      led = true;
      digitalWrite(LED_BUILTIN, led);
      break;

    case 0b10000010:
      Serial.println("LED OFF!!!");
      led = false;
      digitalWrite(LED_BUILTIN, led);
      break;

    case 0b10000011:
      Serial.println("LED SWITCH!!!");
      led = !led;
      digitalWrite(LED_BUILTIN, led);
      break;

    case 0b10000100:
      Serial.println("IF TURN!!!");
      if (led) {
        motor.runForwardRight();
      } else {
        motor.runForwardLeft();
      }
      delay(1000 / 10); //delay de 1/10 de segundo
      break;

    case 0b10000101:
      Serial.println("Delay 0.5s!!!");
      delay(1000 / 2);
      break;

    case 0b10000110:
      Serial.println("Delay 1s!!!");
      delay(1000);
      break;

    case 0b10000111:
      Serial.println("Delay 10s!!!");
      delay(10000);
      break;

    case 0b10001000:
      Serial.println("Turn Left!!!");
      motor.turnLeft();
      delay(1000 / 10);
      break;

    case 0b10001001:
      Serial.println("Turn Right!!!");
      motor.turnRight();
      delay(1000 / 10);
      break;

    case 0b10001010:
      Serial.println("Run Forward!!!");
      motor.runForward();
      delay(1000 / 5);
      break;

    case 0b10001011:
      Serial.println("brake!!!");
      motor.brake();
      delay(1000 / 5);
      break;

    case 0b10001100:
      Serial.println("Run Backwards!!!");
      motor.runBackwards();
      delay(1000 / 5);
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


void checkBit(int numberOfBits) {
  if (bitPosition > -1) {
    int index;

    for (index = bitPosition; index < 8 && index < numberOfBits + bitPosition; index++) {
      byte value = bitLastValue;
//      Serial.print("recebido:");
//      Serial.println(value);
      byteReceived = byteReceived | value;
      if (index < 7) {
        byteReceived = byteReceived << 1;
      }
    }

//    Serial.print("INDEX:");
//    Serial.print(index);

    if (index >= 7) {
      numberOfBits = numberOfBits - (index - bitPosition);
      bitPosition = -1;
      queue.enqueue(byteReceived);
      byteReceived = 0;

      //      //------------------------TEMP------------------------------------
      //      numberOfBits = 0; //fila de tamanho 1

    } else {
      numberOfBits = numberOfBits - (index - bitPosition);
      bitPosition = index;
    }

    if ( numberOfBits > 0 ) {
      checkBit(numberOfBits);
    }

  } else {
    bitPosition++;
  }
}


void checkBar() {
  if (bitLastValue != bitValue) {
    if ((bitPosition == -1 && bitValue) || bitPosition > -1) {
//      Serial.print("p");
//      Serial.println(bitPosition);
//      Serial.print("B");
//      Serial.println(bitValue);
//      Serial.print("T");
//      Serial.println(bitLastValue);
//      Serial.print("F");
      currentCounter = millis();
      int numberOfBits = round((currentCounter - counter) / bitTime);
//      Serial.print(numberOfBits);
      checkBit(numberOfBits);

    } else {
      bitPosition++;
    }
    counter = currentCounter;
  }
  bitLastValue = bitValue;
  bitFirstValue = false;
}

void checkBitTime() {
  if (bitLastValue != bitValue || bitFirstValue) {
    if (bitValue) {
      counter = millis();
    } else {
      bitTime = counter;
      counter = millis();
      bitTime = counter - bitTime;
      Serial.print("bitTime:");
      Serial.println(bitTime);
    }
  }
//  Serial.print("B");
//  Serial.println(bitValue);
//  Serial.print("T");
//  Serial.println(bitLastValue);
  bitLastValue = bitValue;
  bitFirstValue = false;
}

void setup() {
  Serial.begin(serialBaudRate);
  //Define os pinos como saida
  pinMode(LED_BUILTIN, OUTPUT);
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
    goLeft = false;
    goRight = true;
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

  if (centerSensorValue == 1 && !goRight && !goLeft) {
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
    motor.turnRight(255);
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
    if (bitTime < 50) {
      checkBitTime();
    } else {
      checkBar();
    }
  }

  executeCommands();
}


