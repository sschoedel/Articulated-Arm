#include "readSensors.h"
#include "Arduino.h"
#include <AS5600.h>

#define TCAADDR 0x70

AS5600 encoder;


readSensors::readSensors(bool debugBool)
{
  debugging = debugBool;
//  Serial.begin(115200);
//  if (debugging){ Serial.println("Sensor reader started"); }
	tcaselect(0);
}


void readSensors::readEncoders() {
  for(int i = 2; i < 8; i++)
  {
    tcaselect(i);
    raw[i-2] = encoder.getPosition(); // Get the absolute position of the encoder
    
    // Check if a full rotation has been made
    if ((lastRaw[i] - raw[i]) > 2047 )   // Encoder value goes from max to 0
      revolutions[i]++;
    if ((lastRaw[i] - raw[i]) < -2047 )  // Encoder value goes from 0 to max
      revolutions[i]--;

    output[i] = ((revolutions[i] * 4096 + raw[i])/ratio[i]) * (2 * M_PI/4096);  // Calculate scaled output in radians

    lastRaw[i] = raw[i];
  }

  if (debugging)
  {
    // Print all encoder values
    for(int i = 0; i < 6; i++)
    {
//      Serial.print(output[i]);Serial.print(", ");
    }
//    Serial.print("\n"); 
  }
}

// tcaselect reads encoders in the following order, by axis: _ _ _ 1 2 3
void readSensors::tcaselect(int i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
