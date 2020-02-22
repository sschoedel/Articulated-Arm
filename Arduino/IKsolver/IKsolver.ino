// motor 1: 48000 steps for full rotation on base spindle
// motor 2: 304000 steps for full rotation on output shaft
// motor 3: 14800 steps for full rotation on output shaft

#include <AS5600.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>

#define MAX_CMDS 64
#define max_accel 5000

bool debugging = false;

int numMotors = 3;

void calcThetas();
bool parse_command();
void determine_trajectory();
void run_trajectory_to_goal();
int determine_num_cmds(String message);
void updateSensors();

// preset end effector inputs for solver
//float x = 300;
//float y = 300;
//float z = 300;
//float a = 0;
//float b = 0;
//float g = 0;

// arm constants
const int lenEnd = 60;
const float lengths[3] = {135.7, 300.32, 293};
const float theta_1_offset = 0;
const float theta_2_offset = M_PI/2;
const float theta_3_offset = 0.3635;

// sensor variables
const int currentSensorPins[6] = {A2, A3, A4, A5, A6, A7};
int averageCurrents[6] = {0,0,0,0,0,0};
const int sensorAvgNum = 10;                                 // Sensor Avg
int flatten[6][sensorAvgNum] = {{},{},{},{},{},{}};

// Globals
long positions[6];
float thetas[6];
  // x, y, z, a, b, g
float inputs[6] = {100, 100, 100, 0, 0, 0};
String cmd[MAX_CMDS] = {};

AccelStepper stepper1(AccelStepper::DRIVER, 43, 42);
AccelStepper stepper2(AccelStepper::DRIVER, 49, 48);
AccelStepper stepper3(AccelStepper::DRIVER, 47, 46);
AccelStepper stepper4(AccelStepper::DRIVER, 41, 40);
AccelStepper stepper5(AccelStepper::DRIVER, 45, 44);
AccelStepper stepper6(AccelStepper::DRIVER, 39, 38);
MultiStepper steppers;
AS5600 encoder;

long motor3Target = 0;
double encoderOutput;
double encoderRad;
double encoderTheta;

void setup() {
  Serial.begin(115200);

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
  
  for(int i=0; i<MAX_CMDS; i++)
  {
    cmd[i] = " ";
  }

  for(int j=0; j<6; j++)
  {
    for(int i=0; i<sensorAvgNum; i++)
    {
      flatten[j][i] = 0;
    }
  }
}

void loop() {
  bool new_command = parse_command();
  if(new_command)
  {
    determine_trajectory();
  }
  run_trajectory_to_goal();
}

void run_trajectory_to_goal()
{
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); 
  updateSensors();
//  This method blocks until completion - BAD
//  should probably use a loop that checks encoders/force sensors and runs one step each time through so there is some error protection/safety
}

void determine_trajectory()
{
//  Only updates goal positions, for now
  Serial.println("Determining trajectory");
  calcThetas();
  positions[0] = (thetas[0] - theta_1_offset) * 48000 / (2*M_PI);
  positions[1] = ((thetas[1] - theta_2_offset) * 304000/(2*M_PI));
  positions[2] = (thetas[2] - theta_3_offset) * 14800 / (2*M_PI);
  Serial.println("Trajectory determined");
}

//  OLD VOID (for pre-set motor positions
//  encoderOutput = encoder.getPosition(); // get the absolute position of the encoder
//  encoderRad = (encoderOutput/4096)*2*M_PI;
//  encoderTheta = encoderRad * 180/M_PI;
//  Serial.print("current position (deg): ");Serial.println(encoderTheta);

void calcThetas() {
  float xSw = -lenEnd*sin(inputs[4]) + inputs[0];
  float ySw = lenEnd*cos(inputs[4])*sin(inputs[3]) + inputs[1];
  float zSw = -lenEnd*cos(inputs[3])*cos(inputs[4]) + inputs[2];
  float hSw = zSw - lengths[0];
  Serial.print("xSw: ");Serial.print(xSw);  
  Serial.print("\tySw: ");Serial.print(ySw);
  Serial.print("\tzSw: ");Serial.println(zSw);
  if(sqrt(sq(xSw) + sq(ySw) + sq(hSw)) > lengths[1] + lengths[2])
  {
    Serial.println("Desired position and orientation not in workspace.");
    Serial.print("Desired: ");Serial.println(sqrt(sq(xSw) + sq(ySw) + sq(zSw)));
    Serial.print("Max: :");Serial.println(lengths[1] + lengths[2]);
  }
  else
  {
    float RSw = sqrt(sq(xSw) + sq(ySw));
    float rSw = sqrt(sq(hSw) + sq(RSw));
    float alpha2 = asin(hSw/rSw);
    thetas[0] = atan(ySw/xSw);
    thetas[1] = acos((sq(lengths[1]) - sq(lengths[2]) + sq(rSw))/(2*lengths[1]*rSw)) + alpha2;
    thetas[2] = acos((sq(lengths[2]) + sq(lengths[1]) - sq(rSw))/(2*lengths[1]*lengths[2]));
  }
}

bool parse_command()
{
  bool new_command = false;
  if(Serial.available() > 0)
  {
    new_command = true;
    String message = Serial.readStringUntil('\n');
    Serial.println("New command received");

    // Return number of words in full command
    int num_cmds = determine_num_cmds(message);

    // Recreate command input as array
    String wrd = "";
    int lnth = message.length();
    int command_num = 0;
    for(int i=0; i<lnth; i++)
    {
      if(message[i] == ' ')
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

    // Print recreated command
    for(int i=0; i<num_cmds; i++)
    {
      Serial.print(cmd[i]);
      Serial.print(" ");
    }
    Serial.print('\n');

    // Set command lowercase
    for(int i=0; i<num_cmds; i++)
    {
      cmd[i].toLowerCase();
    }

    // Determine what to do with command
    if(cmd[0] == "set")
    {
      if(cmd[1] == "end")
      {
        if(cmd[2] == "all")
        {
          inputs[0] = cmd[3].toInt();
          inputs[1] = cmd[4].toInt();
          inputs[2] = cmd[5].toInt();
          inputs[3] = cmd[6].toInt();
          inputs[4] = cmd[7].toInt();
          inputs[5] = cmd[8].toInt();
        }
        else if(cmd[2] == "pos")
        {
          inputs[0] = cmd[3].toInt();
          inputs[1] = cmd[4].toInt();
          inputs[2] = cmd[5].toInt();
        }
        else if(cmd[2] == "x")
        {
          inputs[0] = cmd[3].toInt();
        }
        else if(cmd[2] == "y")
        {
          inputs[1] = cmd[3].toInt();
        }
        else if(cmd[2] == "z")
        {
          inputs[2] = cmd[3].toInt();
        }
        else if(cmd[2] == "ang")
        {
          inputs[3] = cmd[3].toInt();
          inputs[4] = cmd[4].toInt();
          inputs[5] = cmd[5].toInt();
        }
        else if(cmd[2] == "alpha")
        {
          inputs[3] = cmd[3].toInt();
        }
        else if(cmd[2] == "beta")
        {
          inputs[4] = cmd[3].toInt();
        }
        else if(cmd[2] == "gamma")
        {
          inputs[5] = cmd[3].toInt();
        }
      }
    }
    if(cmd[0] == "return")
    {
      if(cmd[1] == "home")
      {
        inputs[0] = 0;
        inputs[1] = 100;
        inputs[2] = 50;
      }
    }
  }
  return new_command;
}

int determine_num_cmds(String message)
{
  int numCommands = 1;
  int lnth = message.length();
  for(int i=0; i<lnth; i++)
  {
    if(message[i] == ' ')
      numCommands++;
  }
  return numCommands;
}

void updateSensors()
{
  for(int i=0;i<6;i++)
  {
    int newCurrent = analogRead(currentSensorPins[i]);
    averageCurrents[i] = averageData(newCurrent, i);
    
    if (debugging){ Serial.print(averageCurrents[i]);Serial.print("\t"); }
  }
  if (debugging){ Serial.print("\n");}
  Serial.println(averageCurrents[0]);
}

int averageData(int mostRecent, int index)
{
  int total = 0;
  
  // Update end of array
  flatten[index][sensorAvgNum-1] = mostRecent;
  // Shift all values left by one
  for(int i=1;i<sensorAvgNum;i++)
  {
    flatten[index][i-1] = flatten[index][i];
    total += flatten[index][i];
  }
  // Add the tail value that wasn't added in the loop
  total += flatten[index][0];
  // Return average
  return total/(sensorAvgNum);
}


// PRESET end effector positions
//      if (message == "1")
//      {
//        Serial.println("'1' selected");
//        x = 300;
//        y = 300;
//        z = 300;
//        a = 0;
//        b = 0;
//        g = 0;
//      }
//      else if (message == "2")
//      {
//        Serial.println("'2' selected");
//        x = 300;
//        y = 300;
//        z = 500;
//        b = 0;
//        b = 0;
//        g = 0;
//      }
//      else
//      {
//        Serial.println("'home' selected");
//        x = 0;
//        y = 0;
//        z = 0;
//        b = 0;
//        b = 0;
//        g = 0;
//      } 
