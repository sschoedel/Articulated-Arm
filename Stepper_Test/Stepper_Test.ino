
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Wire.h>

AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5);

MultiStepper steppers;

static uint8_t nunchuck_buf[6];

void setup()
{  
    Serial.begin(250000);
    stepper1.setMaxSpeed(8000.0);
    stepper1.setAcceleration(6000.0);

    
    stepper2.setMaxSpeed(8000.0);
    stepper2.setAcceleration(6000.0);
    
    nunchuck_setpowerpins();
    nunchuck_init();
    
    //Serial.print("Ready\n");
    stepper1.setCurrentPosition(0);
}

void loop()
{
    nunchuck_get_data();
    nunchuck_print_data();
    
    //if (nunchuck_buf[0] >= 135
    int axis_1 = map(nunchuck_buf[0], 61, 197, -12000, 12000);
    int axis_2 = map(nunchuck_buf[1], 57, 197, -40000,40000);

    short axis_1_est = axis_1 / 100;
    short axis_2_est = axis_2 / 100;
    axis_1_est = axis_1_est * 100;
    axis_2_est = axis_2_est * 100;
    
    
    Serial.print("axes_raw: ");
    Serial.print(axis_1);
    Serial.print(", ");
    Serial.print(axis_2);
    Serial.print(" \t");
    Serial.print(" \t");

    Serial.print("axes_est: ");
    Serial.print(axis_1_est);
    Serial.print(", ");
    Serial.print(axis_2_est);
    
    Serial.print(" \r \n");
    
    stepper1.moveTo(axis_1_est);
    stepper2.moveTo(axis_2_est);
    stepper1.run();
    stepper2.run();
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

void nunchuck_print_data()
{
  static int i=0;
  int joy_x_axis = nunchuck_buf[0];
  int joy_y_axis = nunchuck_buf[1];
  
  int z_button = 0;
  int c_button = 0;

  int c_on_off = 0;
  int z_on_off = 0;
  int prevC = 0;
  int prevZ = 0;
  
  stepper1.run();

  if ((nunchuck_buf[5] >> 0) & 1)
    z_button = 1;
  if ((nunchuck_buf[5] >> 1) & 1)
    c_button = 1;

  if (z_button == 0 && prevZ == 1){
    if(z_on_off == 0){
      z_on_off = 1;     
    }else{
      z_on_off = 0;
    }
    prevZ = 0;
  }
  if (z_button == 1 && prevZ == 0){
    prevZ = 1;
  }

  if (c_button == 0 && prevC == 1){
    if(c_on_off == 0){
      c_on_off = 1;     
    }else{
      c_on_off = 0;
    }
    prevC = 0;
  }
  if (c_button == 1 && prevC == 0){
    prevC = 1;
  }
  /*
  Serial.print("axes:");
  Serial.print(joy_x_axis,DEC);
  Serial.print(",");
  Serial.print(joy_y_axis, DEC);
  Serial.print("  \t");

  Serial.print("button:");
  Serial.print(z_button, DEC);
  Serial.print(",");
  Serial.print(c_button, DEC); 
  Serial.print("  \t");
     
  Serial.print("on_off: ");
  Serial.print(z_on_off);
  Serial.print(", ");
  Serial.print(c_on_off);
  Serial.print("   \t");

  Serial.print("prev: ");
  Serial.print(prevZ);
  Serial.print(", ");
  Serial.print(prevC);
  Serial.print("   \t");
  
  Serial.print("\r\n");*/
  i++;
}

char nunchuk_decode_byte (char x)
{
  x = (x ^ 0x17) + 0x17;
  return x;
}
