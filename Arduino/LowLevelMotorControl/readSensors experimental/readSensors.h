#ifndef READENCODERS_H
#define READENCODERS_H

#include "Arduino.h"



class readSensors
{
	public:
		readSensors(bool debugBool);
		void tcaselect(int select);
		void readEncoders();
		double output[6];
	private:
    bool debugging;
		// Variables for encoders
		long revolutions[6] = {0, 0, 0, 0, 0, 0};   // number of revolutions the encoder has made
		double raw[6] = {0, 0, 0, 0, 0, 0};    // the calculated value the encoder is at
		double lastRaw[6] = {0, 0, 0, 0, 0, 0};
		long lastOutput[6] = {0, 0, 0, 0, 0, 0};        // last output from AS5600
		long ratio[6] = {1, 1, 1, 20*1.484, 50, 1}; // gear ratios for each axis
    // tcaselect reads encoders in the following order, by axis: _ _ _ 1 2 3
};

#endif
