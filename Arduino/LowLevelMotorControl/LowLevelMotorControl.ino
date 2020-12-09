// Tells motors to step
// Returns sensor data to nano
// This lets the nano control the motors at a low level
// Using run() instead of run to position() allows the nano to assess the situation
// at each motor step so that it can change the arm's trajectory as it runs

/*
 * Overall logic structure possibilities:
 * 
 * 1) Nano determines where and in what orientation the end effector should be, communicating those six values to the teensy
 * 2)
 *  a) Teensy determines required motor rotations to meet given specifications and moves the motors, allowing the nano to update the end effector location and orientation at any time
 *  b) Teensy determines required motor rotations and interpolates between the current and goal positions to find intermediate steps to take to ensure the end effector travels along a defined path
 * Notes) Teensy code will redirect motor movement immediately upon receiving updated end effector specifications. This ensures minimal lag if the nano decides to abort or change the current goal position
 * 
 * OR
 * 
 * 1) Nano determines where and in what orientation the end effector should be
 * 2) Nano determines optimal path between the current and goal end effector positions
 * 3) Nano interpolates between current and goal positions along the chosen path and defines a finite number of locations for the end effector to pass through.      
 *      It is possible that, during interpolation (on the Nano or the Teensy), there are multiple solutions for the arm. Choose the one that requires the least amount of motion.
 * 4) Nano sends interpolation waypoints one at a time to the Teensy
 * 5) Teensy determines required motor rotations for each end effector waypoint position and orientation and rotates the motors the determined amount.
 * Notes) This method (interpolating between two points and going to each waypoint individually, used in both possible logic structures) can't use acceleration during intermediate
 *      steps. Otherwise, the movement will not be smooth. Acceleration must only be performed when going between the first few and last few steps.
 *      Also, if the Nano decides to change the end effector location or orientation mid-transit, it must first calculate where the end effector will be once it stops accelerating. It must then use that 
 *      location to determine new waypoints along a chosen path. The Nano should also continue sending the Teensy waypoints until the arm has stopped moving. Then it should send the waypoints for the new motion.
 *      
 *      Another solution can be implemented where the Nano defines a new path between the current (in motion) end effector location and the new goal location that allows for a fluid transition between the two paths.
 */

// Some useful accelstepper functions:
// setCurrentPosition(long position) sets the current stepper position to position
// currentPosition() returns current position in steps
// distanceToGo() returns distance from current position to target position in steps where positive is clockwise from current position
// moveTo(long absolute) sets target position. then run() moves one step per call to the target position (recalculates speeds every step)


/*
 * TODO:
 *  Test to see if can interrupt accelstepper movement mid-motion.
 */

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <AS5600.h>
#include <RH_NRF24.h>

#define TCAADDR 0x70

#define MAX_CMDS 64

// arm geometry constants in mm:
long len1 = 300;
long len2 = len1;
long currentToolLen = 54; // two finger gripper
long len3 = 83 + currentToolLen;
int h1 = 135;

//                     motors 1 2 3 4 5 6
float startingDegrees[6] = {0, 90, 8, 0, -12, 0};
long stepsPerRev[6] = {48000, 312000, 15200, 5720, 10280, 4150};

// Interrupt constants
const uint16_t t1_load = 0;
const uint16_t t1_comp = 65535; // t1_comp / clock speed = isr period

// Variables for encoders
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

//            motors 4 5 6 1 2 3
long ratio[6] = {1, 1, 1, 29.68, 50, 1}; // gear ratios for each axis
int encoderOffsetsDegrees[6] = {0, 0, 0, 0, 90, -10};
// tcaselect reads encoders in the following order, by axis: _ _ _ 1 2 3


// Other globals

// Thetas means radians, positions means steps
float goalThetas[6] = {0, 0, 0, 0, 0, 0};
long goalPositions[6] = {0, 0, 0, 0, 0, 0};
String cmd[MAX_CMDS] = {};
bool debugging = true;
bool debuggingEncoders = false;
bool newCmd = false;

AccelStepper stepper1(AccelStepper::DRIVER, 43, 42);
AccelStepper stepper2(AccelStepper::DRIVER, 49, 48);
AccelStepper stepper3(AccelStepper::DRIVER, 47, 46);
AccelStepper stepper4(AccelStepper::DRIVER, 41, 40);
AccelStepper stepper5(AccelStepper::DRIVER, 45, 44);
AccelStepper stepper6(AccelStepper::DRIVER, 39, 38);
MultiStepper steppers;
//readSensors sensors(false);
AS5600 encoder;
RH_NRF24 nrf24;


// Function declarations
bool readCommand();
void parseCommand();
void updateGoalAngles();
void setupMotors();
void readEncoders();
void tcaselect(uint8_t i);
void setAllMotorPos();
void stopMotors();
void setGoalLoc();
void setGoalPos();

void checkNRF();

void setup() {
  // Serial, i2c, and motor setup
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Wire.begin();
  setupMotors();

  while (!Serial);
  if (!nrf24.init())
    Serial.println("initialization failed");                           
  if (!nrf24.setChannel(1))
    Serial.println("Channel Set failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("RF set failed");    


  // Interrupt setup
  // Reset Timer1 control register A
  TCCR1A = 0;

  // Prescaler to no prescaling
  TCCR1B &= ~(1 << CS12);
  TCCR1B &= ~(1 << CS11);
  TCCR1B |= (1 << CS10);

  // Reset Timer1 and set compare value
  TCNT1 = t1_load;
  OCR1A = t1_comp;

  // Enable Timer1 compare interrupt
  TIMSK1 = (1 << OCIE1A);

  // Enable global interrupts
  sei();
}


// Interrupt service routine
ISR(TIMER1_COMPA_vect) { // Whenever clock value register matches compa register
}

void loop()
{
  checkNRF();
  newCmd = readCommand();
  if (newCmd)
  {
    parseCommand();
  }

  steppers.run();  // no acceleration
  
//  readEncoders();   // MAKES STEPPERS RUN TOO SLOW. Try again using teensy. Teensy might be be able to loop fast enough that readEncoders lag doesn't matter
//  returnData();
  // It might be possible for the serial buffers on teensy and nano to build up,
  // meaning they are reading irrelevant data on each clock cycle
  // *not implemented yet*: clearing the input buffer after reading may fix that.
//  Serial.flush(); // waits for serial data to send  - MAKES STEPPERS RUN MUCH SLOWER
}

void checkNRF()
{
  if (nrf24.available())
  {
    uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if(nrf24.recv(buf, &len))
    {
      Serial.print("Received: ");
      Serial.println((char*)buf);
      String data=String((char*)buf);

      if (data == "on")
      {
        digitalWrite(13, HIGH);
      }
      else if (data == "off")
      {
        digitalWrite(13, LOW);
      }
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}


bool readCommand()
{
  int newCommand = false;
  if (Serial.available() > 0)
  {
    newCommand = true;
    String message = Serial.readStringUntil('\n');
    if (debugging) {
      Serial.print("New command received: ");
    }

    // Return number of words in full command
    int num_cmds = determine_num_cmds(message);

    // Recreate command input as array
    String wrd = "";
    int lnth = message.length();
    int command_num = 0;
    for (int i = 0; i < lnth; i++)
    {
      if (message[i] == ' ')
      {
        cmd[command_num] = wrd;
        command_num++;
        wrd = "";
      }
      else
      {
        wrd += message[i];
      }
    }
    cmd[command_num] = wrd;

    // Set command lowercase
    for (int i = 0; i < num_cmds; i++)
    {
      cmd[i].toLowerCase();
    }

    if (debugging)
    {
      // Print recreated command
      for (int i = 0; i < num_cmds; i++)
      {
        Serial.print(cmd[i]);
        Serial.print(" ");
      }
      Serial.print('\n');
    }
    
  }
  return newCommand;
}

int determine_num_cmds(String message)
{
  int numCommands = 1;
  int lnth = message.length();
  for (int i = 0; i < lnth; i++)
  {
    if (message[i] == ' ')
      numCommands++;
  }
  return numCommands;
}


// Most of these are for debugging.
// Nano only needs to send goal position, stop, and home

void parseCommand()
{
  if (cmd[0] == "gthetas")
  {
    setAllMotorsPos();
  }
  else if (cmd[0] == "gloc")
  {
    setGoalLoc();
  }
  else if (cmd[0] == "gpos")
  {
    setGoalPos();
  }
  else if (cmd[0] == "home")
  {
    homeMotors();
  }
  else if (cmd[0] == "pgoalthetas")
  {
    printGoalThetas();
  }
  else if (cmd[0] == "pencoderthetas") // Where the encoders actually are (assuming encoders haven't slipped)
  {
    printEncoderThetas();
  }
  else if (cmd[0] == "pstepperthetas") // Where accelstepper thinks the encoders are
  {
    printStepperThetas();
  }
  else if (cmd[0] == "pdisttogo") // Accelstepper's distance to go function
  {
    printDistToGo();
  }
  else if (cmd[0] == "stop")
  {
    if (debugging) { Serial.println("STOPPING"); }
    stopMotors();
  }
}

/*
 * Parse command goes here to execute given commands
 */
void setAllMotorsPos()
{
  // Updates goal thetas and positions
  goalThetas[0] = cmd[1].toFloat() * ratio[3];
  goalThetas[1] = cmd[2].toFloat() * ratio[4];
  goalThetas[2] = cmd[3].toFloat() * ratio[5];
  goalThetas[3] = cmd[4].toFloat() * ratio[0];
  goalThetas[4] = cmd[5].toFloat() * ratio[1]; 
  goalThetas[5] = cmd[6].toFloat() * ratio[2];
  
  for(int i=0; i<6; i++)
  {
    goalPositions[i] = goalThetas[i] * 4096/(2*M_PI);
  }
  
  if (debugging) { 
    Serial.print("Updated goal positions: ");
    for(int i=0; i<5; i++){Serial.print(goalPositions[i]);Serial.print(", ");}
    Serial.print(goalPositions[5]);
    Serial.print("\n");
  }
  
  steppers.moveTo(goalPositions);
  
//  steppers.runSpeedToPosition(); // this blocks but might work if using fine enough waypoints between two positions to allow for data transfer between Nano and Teensy
}

void setGoalLoc()
{
  float x = cmd[1].toFloat();
  float y = cmd[2].toFloat();
  float z = cmd[3].toFloat();
  float a = cmd[4].toFloat();
  float b = cmd[5].toFloat();
  float g = cmd[6].toFloat();
  if(sqrt(sq(x) + sq(y) + sq(z-130)) <= len1 + len2)
  {
    kinematics(x, y, z, a, b, g);
  }
  else
  {
    Serial.println("not in range");
  }
}

void setGoalPos()
{
  goalPositions[0] = cmd[1].toInt();
  goalPositions[1] = cmd[2].toInt();
  goalPositions[2] = cmd[3].toInt();
  goalPositions[3] = cmd[4].toInt();
  goalPositions[4] = cmd[5].toInt();
  goalPositions[5] = cmd[6].toInt();
  
  if (debugging) { 
    Serial.print("Updated goal positions: ");
    for(int i=0; i<5; i++){Serial.print(goalPositions[i]);Serial.print(", ");}
    Serial.print(goalPositions[5]);
    Serial.print("\n");
  }
  
  steppers.moveTo(goalPositions);
}

void homeMotors()
{
  float startingDegrees[6] = {0, 90, 8, 0, 0, 0};
  for(int i=0;i<6;i++)
  {
    goalPositions[i] = startingDegrees[i] * stepsPerRev[i]/360;
  }
  steppers.moveTo(goalPositions);
  
  Serial.println("Returning to home");
}

void printGoalThetas()
{
  Serial.print("Goal thetas: ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(goalThetas[i]); Serial.print(", ");
  }
  Serial.println(goalThetas[5]);
}

void printEncoderThetas()
{
  Serial.print("Thetas tracked by encoders (radians): ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(encoderThetasRads[i]); Serial.print(", ");
  }
  Serial.println(encoderThetasRads[5]);
  Serial.print('\n');
  Serial.print("Thetas tracked by encoders (degrees): ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(encoderThetasDegrees[i]); Serial.print(", ");
  }
  Serial.println(encoderThetasDegrees[5]);
}

void printStepperThetas()
{
  Serial.print("Thetas tracked by AccelStepper: "); // need to be scaled
  Serial.print((stepper1.currentPosition()/ratio[0]) * (2 * M_PI / 4096)); Serial.print(", ");
  Serial.print((stepper2.currentPosition()/ratio[1]) * (2 * M_PI / 4096)); Serial.print(", ");
  Serial.print((stepper3.currentPosition()/ratio[2]) * (2 * M_PI / 4096)); Serial.print(", ");
  Serial.print((stepper4.currentPosition()/ratio[3]) * (2 * M_PI / 4096)); Serial.print(", ");
  Serial.print((stepper5.currentPosition()/ratio[4]) * (2 * M_PI / 4096)); Serial.print(", ");
  Serial.println((stepper6.currentPosition()/ratio[5]) * (2 * M_PI / 4096));
}

void printDistToGo()
{
  Serial.print("Steps to go: ");
  Serial.print(stepper1.distanceToGo()); Serial.print(", ");
  Serial.print(stepper2.distanceToGo()); Serial.print(", ");
  Serial.print(stepper3.distanceToGo()); Serial.print(", ");
  Serial.print(stepper4.distanceToGo()); Serial.print(", ");
  Serial.print(stepper5.distanceToGo()); Serial.print(", ");
  Serial.println(stepper6.distanceToGo());
}

void stopMotors()
{
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  stepper3.setSpeed(0);
  stepper4.setSpeed(0);
  stepper5.setSpeed(0);
  stepper6.setSpeed(0);
  
  stepper1.setCurrentPosition(stepper1.targetPosition());
  stepper2.setCurrentPosition(stepper2.targetPosition());
  stepper3.setCurrentPosition(stepper3.targetPosition());
  stepper4.setCurrentPosition(stepper4.targetPosition());
  stepper5.setCurrentPosition(stepper5.targetPosition());
  stepper6.setCurrentPosition(stepper6.targetPosition());
  
  steppers.moveTo(goalPositions);
  Serial.println("Motors stopped");
}
/*
 * End of parse command section
 */
 


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
  }
}

// tcaselect reads encoders in the following order, by axis: _ _ _ 1 2 3
void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Copy pasted from AS5600 source code
long _getRegisters2(byte registerMSB, byte registerLSB)
{
  _lsb = 0;
  _msb = 0;
  
  Wire.beginTransmission(_AS5600Address);
  Wire.write(registerMSB);
  Wire.endTransmission();
//  delay(10);

  Wire.requestFrom(_AS5600Address, 1);

  if(Wire.available() <=1) {
    _msb = Wire.read();
  }

  Wire.requestFrom(_AS5600Address, 1);

  Wire.beginTransmission(_AS5600Address);
  Wire.write(registerLSB);
  Wire.endTransmission();

  if(Wire.available() <=1) {
    _lsb = Wire.read();
  }

  return (_lsb) + (_msb & _msbMask) * 256;
}


void returnData()
{
  // send 6 encoder values, 6 current position values
}

void setupMotors()
{
  int max_accel = 5000;
  int max_speed = 200000;
  
  stepper1.setMaxSpeed(max_speed);
  stepper2.setMaxSpeed(max_speed);
  stepper3.setMaxSpeed(max_speed);
  stepper4.setMaxSpeed(max_speed);
  stepper5.setMaxSpeed(max_speed);
  stepper6.setMaxSpeed(max_speed);
  
  stepper1.setAcceleration(max_accel);
  stepper2.setAcceleration(max_accel);
  stepper3.setAcceleration(max_accel);
  stepper4.setAcceleration(max_accel);
  stepper5.setAcceleration(max_accel);
  stepper6.setAcceleration(max_accel);

  // Starting thetas
  // TODO: Update starting thetas from encoder values
  stepper1.setCurrentPosition(startingDegrees[0] * stepsPerRev[0]/360);
  stepper2.setCurrentPosition(startingDegrees[1] * stepsPerRev[1]/360);
  stepper3.setCurrentPosition(startingDegrees[2] * stepsPerRev[2]/360);
  stepper4.setCurrentPosition(startingDegrees[3] * stepsPerRev[3]/360);
  stepper5.setCurrentPosition(startingDegrees[4] * stepsPerRev[4]/360);
  stepper6.setCurrentPosition(startingDegrees[5] * stepsPerRev[5]/360);
//  stepper1.setCurrentPosition(0);
//  stepper2.setCurrentPosition(0);
//  stepper3.setCurrentPosition(0);
//  stepper4.setCurrentPosition(0);
//  stepper5.setCurrentPosition(0);
//  stepper6.setCurrentPosition(0);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);
}
