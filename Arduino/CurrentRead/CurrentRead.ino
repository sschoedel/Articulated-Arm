int sensor0 = A2;
int sensor0Avg = 0;
const int sensorAvgNum = 1000;
int flatten0[sensorAvgNum+1] = {};

void setup()
{
  Serial.begin(9600);
  pinMode(sensor0, INPUT);
}

void loop()
{
  int pinValue = analogRead(sensor0);
  sensor0Avg = 0;
  for(int i=sensorAvgNum;i>1;i--)
  {
    flatten0[i-1] = flatten0[i];
  }
  flatten0[sensorAvgNum] = pinValue*100;
  for(int i=0;i<sensorAvgNum;i++)
  {
    sensor0Avg += flatten0[i];
  }
  sensor0Avg = sensor0Avg/(sensorAvgNum-1);
  Serial.println(sensor0Avg);Serial.print('\t');
//  Serial.println(pinValue);
}
