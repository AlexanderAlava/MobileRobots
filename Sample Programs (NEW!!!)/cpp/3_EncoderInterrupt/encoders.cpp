// This program demonstrates usage of the digital encoders.
// After executing the program, manually spin the wheels and observe the output.
// See http://wiringpi.com/reference/priority-interrupts-and-threads/ for more details.

#include <iostream>
#include <wiringPi.h>

// Pins that the encoders are connected to
#define LENCODER 17
#define RENCODER 18

using namespace std;

// This function is called when the left encoder detects a rising edge signal.
void onLeftEncode()
{
	cout << "Left encoder ticked!" << endl;
}

// This function is called when the right encoder detects a rising edge signal.
void onRightEncode()
{
	cout << "Right encoder ticked!" << endl;
}

int main()
{
    // Initialize Wiring Pi, using the pin numbering scheme shown on the 
    // robot itself.
	wiringPiSetupGpio();
	
    // Set encoder pins as input
	pinMode(LENCODER, INPUT);
	pinMode(RENCODER, INPUT);
	
    // Enable pull-up resistors on the encoder pins
    // This ensures a clean 0V and 3.3V is always outputted from the encoders.
	pullUpDnControl(LENCODER, PUD_UP);
	pullUpDnControl(RENCODER, PUD_UP);
	
    // Attach a rising edge interrupt to the encoder pins
	wiringPiISR(LENCODER, INT_EDGE_RISING, onLeftEncode);
	wiringPiISR(RENCODER, INT_EDGE_RISING, onRightEncode);
	
    // Prevent the program from exiting by adding a looping delay.
	while (true)
	{
		delay(1000);
	}
}
