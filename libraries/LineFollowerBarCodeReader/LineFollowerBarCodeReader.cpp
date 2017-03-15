#include "Arduino.h"

//#include "../Motor/Motor.h"
#include "../QueueArray/QueueArray.h"
#include "LineFollowerBarCodeReader.h"

LineFollowerBarCodeReader::LineFollowerBarCodeReader(int barCodeSensorPin, Motor motor, bool led){
    this->barCodeSensorPin=barCodeSensorPin;
    this->motor=motor;
    this->led=led;
    
    bitTime = 0;
    currentCounter = 0;
    counter = 0;
    bitPosition = 0;
    byteReceived = 0;
    centerSensorLastValue = -1;
    centerSensorValue = 0;
}

void LineFollowerBarCodeReader:: executeCommand(byte command){

    switch (command) {
        case 0b10000000:
            //END
            motor.brake();
            led=true;
            digitalWrite(LED_BUILTIN, HIGH);
        break;
        case 0b10000001:
            led=true;
            digitalWrite(LED_BUILTIN, HIGH);
        break;

        case 0b10000010:
            led=true;
            digitalWrite(LED_BUILTIN, LOW);
        break;
        case 0b10000011:
            led=!led;
            digitalWrite(LED_BUILTIN, led);
        break;
        default: 
        break;
    }

}

void LineFollowerBarCodeReader:: executeCommands(){
    while(!queue.isEmpty()){
        executeCommand(queue.dequeue());
    }
}

void LineFollowerBarCodeReader:: checkBar(){

    if(centerSensorLastValue==centerSensorValue){
        
    }else{
        if((bitPosition==1 && centerSensorValue==1) || bitPosition!=1){
            currentCounter = millis();
            int numberOfBits=round((currentCounter-counter)/bitTime);
            
            do{
                int index;
                for(int index=bitPosition; index<8 && index<numberOfBits+bitPosition; index++){
                    byte value=(centerSensorLastValue==1);
                    byteReceived=byteReceived|value;
                    if(index<7){
                        byteReceived=byteReceived<<1;
                    }
                }

                if(index>7){
                    queue.enqueue (byteReceived);
                    byteReceived=0;
                }
                
                if(numberOfBits>8){
                    numberOfBits-=8;
                }else{
                    if(numberOfBits+(bitPosition+1)>8){
                        bitPosition=numberOfBits-8+bitPosition;
                        numberOfBits=8;
                    }else{
                        bitPosition=(bitPosition+1)+numberOfBits-(8+1);
                    }
                }
            }while((bitPosition+1)+numberOfBits>8);
            
            counter = currentCounter;
        }
    }
    centerSensorLastValue=centerSensorValue;
}