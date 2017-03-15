#include <Motor.h>
#include <QueueArray.h>
#include <math.h>

//Definicoes pinos Arduino ligados a entrada da Ponte H
int const motorNegativeLeftPin = 9;
int const motorPositiveLeftPin = 10;
int const motorPositiveRightPin = 11;
int const motorNegativeRightPin = 5;
int const intensityPercentage = 40;
int const intensityMax = 255;
int const percentageMax = 100;
Motor motor(motorNegativeLeftPin, motorPositiveLeftPin, motorPositiveRightPin, motorNegativeRightPin, (intensityPercentage / percentageMax)*intensityMax);

int const leftSensorPin = 6;
int const centerSensorPin = 7;
int const rightSensorPin = 8;

int const serialBaudRate = 9600;

int const numberOfSamples = 10;
int const sampleDelay = 100;

int centerSensorLastValue = -1;

int leftSensorValue = 0;
int centerSensorValue = 0;
int rightSensorValue = 0;

int leftOffset = 0;
int centerOffset = 0;
int rightOffset = 0;

bool goLeft = false;
bool goRight = false;

bool warning = false;
bool error = false;
bool led = false;
bool finish = false;

int errorChecker = 10;
int errorNumber = 0;

unsigned long bitTime = 0;

unsigned long currentCounter = 0;
unsigned long counter = 0;

int bitPosition = 0;

QueueArray <byte> queue;
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
      delay(1000 / 2);
      break;

    case 0b00000110:
      Serial.println("Delay 1s!!!");
      delay(1000);
      break;

    case 0b00000111:
      Serial.println("Delay 10s!!!");
      delay(10000);
      break;

    case 0b00001000:
      Serial.println("Turn Left!!!");
      motor.turnLeft();
      delay(1000 / 10);
      break;

    case 0b00001001:
      Serial.println("Turn Right!!!");
      motor.turnRight();
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

void checkBar() {

  if (centerSensorLastValue == centerSensorValue) {

  } else {
    if ((bitPosition == 0 && centerSensorValue == 1) || bitPosition != 0) {
      Serial.print("F");
      currentCounter = millis();
      if (bitPosition != 0) {
        Serial.print("C");
        int numberOfBits = round((currentCounter - counter) / bitTime);
        Serial.print(numberOfBits);
        do {
          int index;
          for (index = bitPosition; index < 8 && index < numberOfBits + bitPosition; index++) {
            byte value = (centerSensorLastValue == 1);
            byteReceived = byteReceived | value;
            if (index < 7) {
              byteReceived = byteReceived << 1;
            }
          }



          Serial.print("INDEX:");
          Serial.print(index);

          if (index >= 7) {
            numberOfBits = numberOfBits - (index - bitPosition);
            bitPosition = 0;
            Serial.println("BYTE");
            queue.enqueue(byteReceived);
            byteReceived = 0;
            //------------------------TEMP------------------------------------
            numberOfBits = 0; //fila de tamanho 1

          } else {
            numberOfBits = numberOfBits - (index - bitPosition);
            bitPosition = index;
          }
        } while ((bitPosition + 1) + numberOfBits > 8);
      } else {
        //byteReceived=byteReceived|1;
        bitPosition++;
      }

      counter = currentCounter;
    }
  }
  centerSensorLastValue = centerSensorValue;
}

void checkBitTime() {
  if (centerSensorLastValue != centerSensorValue) {
    if (centerSensorValue == 1) {
      counter = millis();
    } else {
      bitTime = counter;
      counter = millis();
      bitTime = counter - bitTime;
      Serial.print("bitTime:");
      Serial.println(bitTime);
    }
  }
  centerSensorLastValue = centerSensorValue;
}

void setup() {
  Serial.begin(serialBaudRate);
  //Define os pinos como saida
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  leftSensorValue = digitalRead(leftSensorPin);
  centerSensorValue = digitalRead(centerSensorPin);
  rightSensorValue = digitalRead(rightSensorPin);

  //    Serial.println("left:");
  //    Serial.println(leftSensorValue);
  //    Serial.println("right:");
  //    Serial.println(rightSensorValue);

  if (bitTime < 100) {
    checkBitTime();
  } else {
    checkBar();
  }
  if (leftSensorValue == 1 && rightSensorValue == 1) {
    //ok
    goLeft = false;
    goRight = false;
    warning = false;
    error = false;
  }

  if (leftSensorValue == 0 && rightSensorValue == 1) {
    goLeft = true;
    goRight = false;
    warning = false;
    error = false;
  }

  if (leftSensorValue == 1 && rightSensorValue == 0) {
    goLeft = false;
    goRight = true;
    warning = false;
    error = false;
  }

  if (leftSensorValue == 0 && rightSensorValue == 0) {
    if (goLeft) {
      if (!warning) {
        errorNumber = errorChecker;
      }
      warning = true;
    }

    if (goRight) {
      if (!warning) {
        errorNumber = errorChecker;
      }
      warning = true;
    }

    if ((goRight && goLeft) || (!goRight && !goLeft)) {
      error = true;
    }
  }

  if (!goRight && !goLeft) {
    motor.runForward();
  }

  if (goRight && !goLeft) {
    if (!warning) {
      motor.runForwardRight();
    } else {
      motor.turnRight();
    }
  }

  if (!goRight && goLeft) {
    if (!warning) {
      motor.runForwardLeft();
    } else {
      motor.turnLeft();
    }
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
  executeCommands();
}


