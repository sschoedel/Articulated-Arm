#include <AccelStepper.h>
#include <MultiStepper.h>

AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5);
AccelStepper stepper3(AccelStepper::DRIVER, 6, 7);
AccelStepper stepper4(AccelStepper::DRIVER, 8, 9);

int prevC = 0;
int prevZ = 0;
//int serialData[] = {};
int serialData = 0;
int deadValue = 30;

void setup()
{  
    Serial.begin(250000);
    pinMode(13, OUTPUT);
    
    stepper1.setMaxSpeed(1000.0);
    stepper1.setAcceleration(10000.0);

    stepper2.setMaxSpeed(16000.0);
    stepper2.setAcceleration(32000.0);
    
    stepper3.setMaxSpeed(16000.0);
    stepper3.setAcceleration(32000.0);
    
    stepper4.setMaxSpeed(16000.0);
    stepper4.setAcceleration(32000.0);
    
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
}

void loop()
{
    int axis_1 = 0;
    int axis_2 = 0;

    //receive data from python on laptop
    /*if(sizeof(Serial.read()) == 2){
      for(int i = 0; i < sizeof(Serial.read()); i++){
        serialData[i] = Serial.read();
      }
    }
    */
    /*//deadzone for hand
    if (serialData >= deadValue){
      axis_1 = 1;
    }else if(serialData <= -deadValue){
      axis_1 = -1;
    }*/
    if(Serial.available() > 0){
      axis_2 = Serial.read();
      if(axis_2 == '1'){
        stepper2.moveTo(100000);
      }else if (axis_2 == '0'){
        stepper2.moveTo(-100000);
      }else{
        stepper2.setCurrentPosition(0);
        stepper2.moveTo(0);
      }
    }
/*
    Serial.print("serialData0: ");
    Serial.print(serialData[0]);
    Serial.print("\t");
    Serial.print("serialData1: ");
    Serial.print(serialData[1]);
    Serial.print("\r\n");*/
    //Moves different motor sets depending on c and z buttons
    //if(segment_counter == 1){
    /*if(axis_1 == 1) {
      stepper1.moveTo(100000);
    }else if(axis_1 == -1){
      stepper1.moveTo(-100000);
    }else{
      stepper1.setCurrentPosition(0);
      stepper1.moveTo(0);
    }*/
    /*else if(segment_counter == 2){
      if(axis_1 == 1) {
        stepper3.moveTo(100000);
      }else if(axis_1 == -1){
        stepper3.moveTo(-100000);
      }else{
        stepper3.setCurrentPosition(0);
        stepper3.moveTo(0);
      }
      if(axis_2 == 1){
        stepper4.moveTo(100000);
      }else if (axis_2 == -1){
        stepper4.moveTo(-100000);
      }else{
        stepper4.setCurrentPosition(0);
        stepper4.moveTo(0);
      }
    }else if(segment_counter == 3){
      if(axis_1 == 1) {
        stepper5.moveTo(100000);
      }else if(axis_1 == -1){
        stepper5.moveTo(-100000);
      }else{
        stepper5.setCurrentPosition(0);
        stepper5.moveTo(0);
      }
      if(axis_2 == 1){
        stepper6.moveTo(100000);
      }else if (axis_2 == -1){
        stepper6.moveTo(-100000);
      }else{
        stepper6.setCurrentPosition(0);
        stepper6.moveTo(0);
      }
    }*/

    //Runs motors a lot to get max speed
    for(int i=0;i<50;i++){
      stepper1.run();
      stepper2.run();
      stepper3.run();
    }
}
