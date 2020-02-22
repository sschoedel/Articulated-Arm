#include <AccelStepper.h>
#include <MultiStepper.h>

#define MAX_BUF 64

AccelStepper stepper1(AccelStepper::DRIVER, 44, 45); 
AccelStepper stepper2(AccelStepper::DRIVER, 38, 39); 
AccelStepper stepper3(AccelStepper::DRIVER, 40, 41);
AccelStepper stepper4(AccelStepper::DRIVER, 42, 43);
AccelStepper stepper5(AccelStepper::DRIVER, 46, 47);
AccelStepper stepper6(AccelStepper::DRIVER, 48, 49);


void setupSteppers();
void intoArray(String data, char separator, String* temp);
void checkCommand(String* data);
void moveto();

String cmd[MAX_BUF];

MultiStepper motors;

int cnt = 0;
int * cntP;

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  // Set up initial stepper speeds and accels and add them to multistepper motor list
  setupSteppers();
  motors.addStepper(stepper1);
  motors.addStepper(stepper2);
  motors.addStepper(stepper3);
  motors.addStepper(stepper4);
  motors.addStepper(stepper5);
  motors.addStepper(stepper6);
  cntP = &cnt;
}

void loop() {
  if(Serial.available() > 0){
    String message = Serial.readStringUntil('\n');
    Serial.println(message);
    Serial.println("Moving");
  
    int idleRot1 = 2000;
    int idleRot2 = 2000;
    int idleRot3 = 2000;
    int idleRot4 = 2000;
    int idleRot5 = 2000;
    int idleRot6 = 2000;
    long positions1[6] = {idleRot1, idleRot2, idleRot3, idleRot4, idleRot5, idleRot6};
    long positions2[6] = {-2000, -2000, 100, -2000, -2000, -2000};
    long positions3[6] = {4000, -8000, 4000, 800, 4000, 300};
    long table[5][6] = {idleRot1, idleRot2, idleRot3, idleRot4, idleRot5, idleRot6,
                        -2000, -2000, 100, -2000, -2000, -2000, 
                        4000, -10000, 5000, 800, 4000, 300,
                        0, 0, 0, 0, 0, 0,
                        7000, 1000, 2700, 3000, 100, -500};
    moveto(table);
    Serial.println("Done");
  }
  
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

void moveto(long table[][6]){
  while(1){
    for(int i=0;i<5;i++)
    {
      motors.moveTo(table[i]);
      while(motors.run());
      delay(500);
    }
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
  stepper1.setMaxSpeed(5000.0);
  stepper2.setMaxSpeed(10000.0);
  stepper3.setMaxSpeed(10000.0);
  stepper4.setMaxSpeed(10000.0);
  stepper5.setMaxSpeed(10000.0);
  stepper6.setMaxSpeed(10000.0);
  stepper1.setAcceleration(1000.0);
  stepper2.setAcceleration(1000.0);
  stepper3.setAcceleration(1000.0);
  stepper4.setAcceleration(1000.0);
  stepper5.setAcceleration(1000.0);
  stepper6.setAcceleration(1000.0);
  
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
  stepper5.setCurrentPosition(0);
  stepper6.setCurrentPosition(0);
}
