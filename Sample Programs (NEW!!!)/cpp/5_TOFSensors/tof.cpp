// This program demonstrates the usage of the time of flight sensors.
// Due to the complexity of the C++ VL53L0X library, a helper class was written.
// After running the program, move your hand in front of each sensor to verify that it's working.
// See https://learn.adafruit.com/adafruit-vl53l0x-micro-lidar-distance-sensor-breakout/overview for more details.

#include <iostream>
#include <wiringPi.h>
#include <cassert>
#include <csignal>
#include "tofHelper.h"

using namespace std;

VL53L0X_Dev_t lSensor;
VL53L0X_Dev_t fSensor;
VL53L0X_Dev_t rSensor;

// This function is called when Ctrl+C is pressed.
// It's intended for properly exiting the program.
void ctrlC(int sig)
{
    // Disconnect all three sensors.
	disconnectSensors(&lSensor, &fSensor, &rSensor);
	exit(0);
}

int main()
{
    // Attach the Ctrl+C signal interrupt
    signal(SIGINT, ctrlC);
    
    // Initialize Wiring Pi, using the pin numbering scheme shown on the 
    // robot itself.
	wiringPiSetupGpio();
    
    // Connect all three sensors.
	connectSensors(&lSensor, &fSensor, &rSensor);

	VL53L0X_RangingMeasurementData_t lMeasurement;
	VL53L0X_RangingMeasurementData_t fMeasurement;
	VL53L0X_RangingMeasurementData_t rMeasurement;
	for (int i = 0; i < 100; i++)
	{
        // Get a measurement from each sensor.
		getMeasurement(&lSensor, &lMeasurement);
		getMeasurement(&fSensor, &fMeasurement);
		getMeasurement(&rSensor, &rMeasurement);
        
        // Print each measurement, but only if it's a valid measurement.
		cout << "\tLeft: ";
		if (lMeasurement.RangeStatus == 0)
			cout << lMeasurement.RangeMilliMeter << "mm";
		else
			cout << "N/A";
			
		cout << "\tFront: ";
		if (fMeasurement.RangeStatus == 0)
			cout << fMeasurement.RangeMilliMeter << "mm";
		else
			cout << "N/A";
			
		cout << "\tRight: ";
		if (rMeasurement.RangeStatus == 0)
			cout << rMeasurement.RangeMilliMeter << "mm";
		else
			cout << "N/A";
		
		cout << endl;
	}
}
