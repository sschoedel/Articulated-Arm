#include <AS5600.h>

#define TCAADDR 0x70

AS5600 encoder;
double output[6];

long revolutions[6] = {0, 0, 0, 0, 0, 0};   // number of revolutions the encoder has made
double raw[6] = {0, 0, 0, 0, 0, 0};    // the calculated value the encoder is at
double lastRaw[6] = {0, 0, 0, 0, 0, 0};
long lastOutput[6] = {0, 0, 0, 0, 0, 0};        // last output from AS5600
//long ratio[6] = {20*1.484, 50, 1, 1, 1, 1}; // gear ratios for each axis
long ratio[6] = {1, 1, 1, 20*1.484, 50, 1}; // gear ratios for each axis

//void tcaselect(uint8_t);

void setup() {
  Serial.begin(115200);
  tcaselect(0);
}

// It doesn't seem like the encoder values require filtering - they're stable enough as is
// However, the option is available if averaging is desired
void loop() {
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

  // Print all encoder values
  for(int i = 0; i < 6; i++)
  {
  Serial.print(output[i]);Serial.print(", ");
//    Serial.print("encoder ");Serial.print(i);Serial.print(": ");Serial.print(output[i]);Serial.print('\t');
  }
  Serial.print("\n");
//  Serial.print('\n');
}
 
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
