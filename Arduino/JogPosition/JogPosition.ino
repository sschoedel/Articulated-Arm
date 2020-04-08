#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Adafruit_NeoPixel.h>
#include <AS5600.h>
 
#define PIN 6
#define MAX_BUF 64
#define TCAADDR 0x70

AccelStepper stepper1(AccelStepper::DRIVER, 43, 42);
AccelStepper stepper2(AccelStepper::DRIVER, 49, 48);
AccelStepper stepper3(AccelStepper::DRIVER, 47, 46);
AccelStepper stepper4(AccelStepper::DRIVER, 41, 40);
AccelStepper stepper5(AccelStepper::DRIVER, 45, 44);
AccelStepper stepper6(AccelStepper::DRIVER, 39, 38);
AS5600 encoder;

Adafruit_NeoPixel ring = Adafruit_NeoPixel(24, PIN, NEO_GRB + NEO_KHZ800);


void setupSteppers();
void intoArray(String data, char separator, String* temp);
void checkCommand(String* data);
void moveto();
void updateGoalPos(long &target, int dir);

String cmd[MAX_BUF];

// Variables for encoders
bool debuggingEncoders = true;

byte _RAWANGLEAddressMSB = 0x0C;
byte _RAWANGLEAddressLSB = 0x0D;
long _msb;
long _lsb;
long _msbMask = 0b00001111;
int _AS5600Address = 0x36;
  
double encoderThetasRads[6];
double encoderThetasDegrees[6];
float currentPositions[6] = {0, 0, 0, 0, 0, 0};
long revolutions[6] = {0, 0, 0, 0, 0, 0};   // number of revolutions the encoder has made
double raw[6] = {0, 0, 0, 0, 0, 0};    // the calculated value the encoder is at
double lastRaw[6] = {0, 0, 0, 0, 0, 0};
long ratio[6] = {1, 1, 1, 20 * 1.484, 50, 1}; // gear ratios for each axis
int encoderOffsetsDegrees[6] = {0, 0, 0, 0, 90, -10};

MultiStepper motors;

int cnt = 0;
int * cntP;

long motor1Target = 0;
long motor2Target = 0;
long motor3Target = 0;
long motor4Target = 0;
long motor5Target = 0;
long motor6Target = 0;

void setup() {
  Serial.begin(57600);
  setupSteppers();
  cntP = &cnt;
  ring.begin();
  ring.setBrightness(10);
  ring.show();
}

void loop() {
  ring.setPixelColor(6, ring.Color(0, 255, 255)); 
  ring.show();
  if(Serial.available() > 0){
    String message = Serial.readStringUntil('\n');
    if(message == "-1") // 1
    {
      Serial.println("Moving");
      updateGoalPos(motor1Target, -1);
    } 
    else if(message == "1")
    {
      Serial.println("Moving");
      updateGoalPos(motor1Target, 1);
    }
    else if(message == "-2") // 2
    {
      Serial.println("Moving");
      updateGoalPos(motor2Target, -1);
    }
    else if(message == "2")
    {
      Serial.println("Moving");
      updateGoalPos(motor2Target, 1);
    }
    else if(message == "-3") // 3
    {
      Serial.println("Moving");
      updateGoalPos(motor3Target, -1);
    }
    else if(message == "3")
    {
      Serial.println("Moving");
      updateGoalPos(motor3Target, 1);
    }
    else if(message == "-4") // 4
    {
      Serial.println("Moving");
      updateGoalPos(motor4Target, -1);
    }
    else if(message == "4")
    {
      Serial.println("Moving");
      updateGoalPos(motor4Target, 1);
    }
    else if(message == "-5") // 5
    {
      Serial.println("Moving");
      updateGoalPos(motor5Target, -1);
    }
    else if(message == "5")
    {
      Serial.println("Moving");
      updateGoalPos(motor5Target, 1);
    }
    else if(message == "-6") // 6
    {
      Serial.println("Moving");
      updateGoalPos(motor6Target, -1);
    }
    else if(message == "6")
    {
      Serial.println("Moving");
      updateGoalPos(motor6Target, 1);
    }
    else
    {
      Serial.println("Invalid command");
    }
    
    // set new positions
    stepper1.moveTo(motor1Target);
    stepper2.moveTo(motor2Target);
    stepper3.moveTo(motor3Target);
    stepper4.moveTo(motor4Target);
    stepper5.moveTo(motor5Target);
    stepper6.moveTo(motor6Target);

    // this blocks until position is reached so buffering inputs won't be possible with run to position
    stepper1.runToPosition();
    stepper2.runToPosition();
    stepper3.runToPosition();
    stepper4.runToPosition();
    stepper5.runToPosition();
    stepper6.runToPosition();
    
    Serial.println("Done");
  }
  readEncoders();
  // targetPosition() most recent target position
  // stop()
  // runToNewPosition(long position) accel/decel
  // runToPosition() runs motor to previously set position with accel/decel
  // moveTo(long absoluteposition)
  // move(long relativeposition)
  // bool isRunning()
  
  /*
  if(Serial.available() > 0){
    String read_string = Serial.readStringUntil('\n');
    read_string.trim();
    Serial.println("Input: " + String(read_string));
    intoArray(read_string, ' ', cmd);
    for(int c = 0; c <= *cntP; c++){
      Serial.print(cmd[c]);
    }
    Serial.print("\n");
    checkCommand(cmd);
  }*/
}

void readEncoders() {
  for (int i = 2; i < 8; i++)
  {
    tcaselect(i);
    raw[i - 2] = encoder.getPosition(); // Get the absolute position of the encoder
    // Check if a full rotation has been made
    if ((lastRaw[i] - raw[i]) > 2047 )   // Encoder value goes from max to 0
      revolutions[i]++;
    if ((lastRaw[i] - raw[i]) < -2047 )  // Encoder value goes from 0 to max
      revolutions[i]--;

    currentPositions[i] = (revolutions[i] * 4096 + raw[i]);
    encoderThetasRads[i] = (currentPositions[i]/ ratio[i]) * (2 * M_PI / 4096); // Calculate scaled encoderThetas in radians
    encoderThetasDegrees[i] = encoderThetasRads[i] * 180 / M_PI; // Calculate scaled encoderThetas in degrees

    lastRaw[i] = raw[i];
  }

  // Add theta offsets to each joint angle
  for(int i=0;i<6;i++)
  {
    encoderThetasDegrees[i] += encoderOffsetsDegrees[i];
    encoderThetasRads[i] += encoderOffsetsDegrees[i] * M_PI / 180;
  }

  if (debuggingEncoders)
  {
    // Print all encoder values
    for (int i = 0; i < 6; i++)
    {
      Serial.print(encoderThetasDegrees[i]); Serial.print(", ");
    }
    Serial.print("\n");
    for (int i = 0; i < 6; i++)
    {
      Serial.print(currentPositions[i]); Serial.print(", ");
    }
    Serial.print("\n");
  }
}

// tcaselect reads encoders in the following order, by axis: _ _ _ 1 2 3
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void updateGoalPos(long &target, int dir) // dir is 1 for CW, -1 for CCW
{
  int motorMoveAmount = 2000;
  target += dir * motorMoveAmount;
}

void moveto(long table[][6]){
  for(int i=0;i<3;i++)
  {
    motors.moveTo(table[i]);
    while(motors.run());
    delay(500);
  }
}

void checkCommand(String* data){
  if(data[0] == "moveto"){
    int index = 0;
    while(data[index].indexOf(',') == -1){ // Find number of motor positions
      index++;
    }
    if(index < 63){
      int numAxes = index;
      int numLocs = index;
      int axes[numAxes];
      int locs[numLocs];
      for(int a = 1; a <= index; a++){
        axes[a - 1] = data[a].toInt(); // Axes array starts at 0 here, so a-1
      }
      for(int l = index + 1; l <= index + numLocs; l++){
        locs[l - index - 1] = data[l].toInt();
      }
      if(numAxes <= numLocs){ // Each axis has a location to go to
        Serial.print("Move axes ");
        for(int a = 0; a < numAxes; a++){
          Serial.print(axes[a]);
          Serial.print(" ");
        }
        Serial.print("to ");
        for(int l = 0; l < numLocs; l++){
          Serial.print(locs[l]);
          Serial.print(" ");
        }
        Serial.print("\n");
      } else{
        Serial.println("Each specified axis does not have a corresponding location.");
      }
    }
    else
    {
      Serial.println("No motors specified.");
    }
  }
  else
  {
    Serial.println("No command.");
  }
}

void intoArray(String data, char separator, String* temp)
{
  int get_index = 0;
  cnt = 0;

  String copy = data;
  
  while(true)
  {
    get_index = copy.indexOf(separator);

    if(-1 != get_index)
    {
      temp[cnt] = copy.substring(0, get_index);

      copy = copy.substring(get_index + 1);
    }
    else
    {
      temp[cnt] = copy.substring(0, copy.length());
      break;
    }
    ++cnt;
  }
}

void setupSteppers(){
  stepper1.setMaxSpeed(100000.0);
  stepper2.setMaxSpeed(100000.0);
  stepper3.setMaxSpeed(100000.0);
  stepper4.setMaxSpeed(100000.0);
  stepper5.setMaxSpeed(100000.0);
  stepper6.setMaxSpeed(100000.0);
  stepper1.setAcceleration(10000.0);
  stepper2.setAcceleration(10000.0);
  stepper3.setAcceleration(10000.0);
  stepper4.setAcceleration(100000.0);
  stepper5.setAcceleration(100000.0);
  stepper6.setAcceleration(100000.0);
  
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);
}
