// Tells motors to step
// Returns sensor data to nano
// This lets the nano control the motors at a low level
// Using run() instead of run to position() allows the nano to assess the situation
// at each motor step so that it can change the arm's trajectory as it runs

// Some useful accelstepper functions:
// setCurrentPosition(long position) sets the current stepper position to position
// currentPosition() returns current position in steps
// distanceToGo() returns distance from current position to target position in steps where positive is clockwise from current position
// moveTo(long absolute) sets target position. then run() moves one step per call to the target position (recalculates speeds every step)

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <AS5600.h>

#define TCAADDR 0x70

#define MAX_CMDS 64
#define max_accel 5000

// Variables for encoders
double currentThetas[6];
long revolutions[6] = {0, 0, 0, 0, 0, 0};   // number of revolutions the encoder has made
double raw[6] = {0, 0, 0, 0, 0, 0};    // the calculated value the encoder is at
double lastRaw[6] = {0, 0, 0, 0, 0, 0};
long ratio[6] = {1, 1, 1, 20 * 1.484, 50, 1}; // gear ratios for each axis
// tcaselect reads encoders in the following order, by axis: _ _ _ 1 2 3


// Other globals

// Nano sends theta values to arduino, arduino turns them into goal positions in steps and returns current
// thetas to nano for evaluation
// Naming convention: Thetas means radians, positions means steps
float goalThetas[6] = {0, 0, 0, 0, 0, 0};
String cmd[MAX_CMDS] = {};
bool debugging = true;
bool debuggingEncoders = false;

AccelStepper stepper1(AccelStepper::DRIVER, 43, 42);
AccelStepper stepper2(AccelStepper::DRIVER, 49, 48);
AccelStepper stepper3(AccelStepper::DRIVER, 47, 46);
AccelStepper stepper4(AccelStepper::DRIVER, 41, 40);
AccelStepper stepper5(AccelStepper::DRIVER, 45, 44);
AccelStepper stepper6(AccelStepper::DRIVER, 39, 38);
MultiStepper steppers;
//readSensors sensors(false);
AS5600 encoder;


// Function declarations
void readCommand();
void parseCommand();
void updateGoalAngles();
void setupMotors();
void readEncoders();
void tcaselect(uint8_t i);


void setup() {
  Serial.begin(115200);
  setupMotors();
}




void loop() {
  readCommand();
  parseCommand();
  steppers.run();  // step once
  readEncoders();
  returnData();
  // It might be possible for the serial buffers on teensy and nano to build up,
  // meaning they are reading irrelevant data on each clock cycle
  Serial.flush(); // waits for serial data to send
}



void readCommand()
{
  if (Serial.available() > 0)
  {
    String message = Serial.readStringUntil('\n');
    if (debugging) {
      Serial.println("New command received");
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

    // Set command lowercase
    for (int i = 0; i < num_cmds; i++)
    {
      cmd[i].toLowerCase();
    }
  }
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
// Nano sends "set goal all 1 2 3 4 5 6" and "eStop"

void parseCommand()
{
  if (cmd[0] == "set")
  {
    if (cmd[1] == "goal")
    {
      if (cmd[1] == "all")
      {
        // Updates goal thetas
        goalThetas[0] = cmd[2].toInt();
        goalThetas[1] = cmd[3].toInt();
        goalThetas[2] = cmd[4].toInt();
        goalThetas[3] = cmd[5].toInt();
        goalThetas[4] = cmd[6].toInt();
        goalThetas[5] = cmd[7].toInt();
      }
    }
  }
  else if (cmd[0] == "get")
  {
    if (cmd[1] == "goal")
    {
      if (cmd[2] == "thetas")
      {
        printGoalThetas();
      }
    }
    else if (cmd[1] == "current")
    {
      if (cmd[2] == "thetas")
      {
        printCurrentThetas();
      }
    }
  }
  else if (cmd[0] == "eStop")
  {
    for (int i = 0; i < 6; i++)
    {
      goalThetas[i] = currentThetas[i];
    }
  }
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

    currentThetas[i] = ((revolutions[i] * 4096 + raw[i]) / ratio[i]) * (2 * M_PI / 4096); // Calculate scaled currentThetas in radians

    lastRaw[i] = raw[i];
  }

  if (debuggingEncoders)
  {
    // Print all encoder values
    for (int i = 0; i < 6; i++)
    {
      Serial.print(currentThetas[i]); Serial.print(", ");
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

void returnData()
{
  // send 6 encoder values, 6 current position values
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

void printCurrentThetas()
{
  Serial.print("Current thetas: ");
  for (int i = 0; i < 5; i++)
  {
    Serial.print(currentThetas[i]); Serial.print(", ");
  }
  Serial.println(currentThetas[5]);
}

void setupMotors()
{
  stepper1.setMaxSpeed(5000);
  stepper2.setMaxSpeed(16000);
  stepper3.setMaxSpeed(16000);
  stepper4.setMaxSpeed(max_accel);
  stepper5.setMaxSpeed(max_accel);
  stepper6.setMaxSpeed(max_accel);
  stepper1.setAcceleration(max_accel);
  stepper2.setAcceleration(max_accel);
  stepper3.setAcceleration(max_accel);
  stepper4.setAcceleration(max_accel);
  stepper5.setAcceleration(max_accel);
  stepper6.setAcceleration(max_accel);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);

  steppers.addStepper(stepper1);
  steppers.addStepper(stepper2);
  steppers.addStepper(stepper3);
  steppers.addStepper(stepper4);
  steppers.addStepper(stepper5);
  steppers.addStepper(stepper6);
}
