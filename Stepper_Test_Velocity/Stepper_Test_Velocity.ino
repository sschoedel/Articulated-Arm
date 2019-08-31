
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>

AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5);
AccelStepper stepper3(AccelStepper::DRIVER, 6, 7);
AccelStepper stepper4(AccelStepper::DRIVER, 8, 9);

static uint8_t nunchuck_buf[6];

int prevC = 0;
int prevZ = 0;
char serialData;

int segment_counter = 1;

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
    
    nunchuck_setpowerpins();
    nunchuck_init();
    
    stepper1.setCurrentPosition(0);
    stepper2.setCurrentPosition(0);
    stepper3.setCurrentPosition(0);
    stepper4.setCurrentPosition(0);
}

void loop()
{
    int axis_1 = 0;
    int axis_2 = 0;
/*
    //receive data from python on laptop
    if(Serial.available() > 0){
      serialData = Serial.read();

      if(serialData == '1'){
        Serial.print("One Received    ");
        digitalWrite(13, HIGH);
      }else if(serialData == '0'){
        Serial.print("Zero Received    ");
        digitalWrite(13, LOW);
      }
    }else{
      Serial.print("None Received    ");
    }
    */
    int segment_counter = get_button_values();
    
    nunchuck_get_data();
    
    //deadzone for nunchuck
    if (nunchuck_buf[0] >=140){
      axis_1 = 1;
    }else if(nunchuck_buf[0] <=120){
      axis_1 = -1;
    }
    if (nunchuck_buf[1] >=140){
      axis_2 = 1;
    }else if(nunchuck_buf[1] <=120){
      axis_2 = -1;
    }

    //Moves different motor sets depending on c and z buttons
    if(segment_counter == 1){
      if(axis_1 == 1) {
        stepper1.moveTo(100000);
      }else if(axis_1 == -1){
        stepper1.moveTo(-100000);
      }else{
        stepper1.setCurrentPosition(0);
        stepper1.moveTo(0);
      }
      if(axis_2 == 1){
        stepper2.moveTo(100000);
      }else if (axis_2 == -1){
        stepper2.moveTo(-100000);
      }else{
        stepper2.setCurrentPosition(0);
        stepper2.moveTo(0);
      }
    }else if(segment_counter == 2){
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
    }/*else if(segment_counter == 3){
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

static void nunchuck_setpowerpins()
{
#define pwrpin PORTC3
#define gndpin PORTC2
    DDRC |= _BV(pwrpin) | _BV(gndpin);
    PORTC &=~ _BV(gndpin);
    PORTC |=  _BV(pwrpin);
    delay(100);
}

void nunchuck_init()
{ 
  Wire.begin();
  Wire.beginTransmission(0x52);
  Wire.write(0x40);
  Wire.write(0x00);
  Wire.endTransmission();
  stepper1.run();
}

void nunchuck_send_request()
{
  Wire.beginTransmission(0x52);
  Wire.write(0x00);
  Wire.endTransmission();
  stepper1.run();
}

int nunchuck_get_data()
{
    stepper1.run();
    int cnt=0;
    Wire.requestFrom(0x52, 6);
    while (Wire.available()) {
      nunchuck_buf[cnt] = nunchuk_decode_byte(Wire.read());
      cnt++;
    }
    nunchuck_send_request();
    if(cnt >= 5) {
      return 1;
    }
    return 0;
}

int get_button_values()
{
  int z_button = 0;
  int c_button = 0;

  if ((nunchuck_buf[5] >> 0) & 1)
    z_button = 1;
  if ((nunchuck_buf[5] >> 1) & 1)
    c_button = 1;

  if (z_button == 0 && prevZ == 1){
    if(segment_counter > 1){
      segment_counter--;
    }
    prevZ = 0;
  }else if (z_button == 1 && prevZ == 0){
    prevZ = 1;
  }

  if (c_button == 0 && prevC == 1){
    if(segment_counter < 3){
      segment_counter++;
    }
    prevC = 0;
  }else if (c_button == 1 && prevC == 0){
    prevC = 1;
  }
  
  Serial.print("segment: ");
  Serial.print(segment_counter, DEC);
  Serial.print("\r\n");

  return segment_counter;
}

char nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}
