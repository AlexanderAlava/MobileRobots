// This program demonstrates usage of some timing functions.
// See http://wiringpi.com/reference/timing/ for more details.

#include <iostream>
#include <wiringPi.h>

using namespace std;

int main()
{
	unsigned int ms1, ms2, ms3;
	unsigned int us1, us2, us3;
	
    // Initializes Wiring Pi and resets the time counter.
    wiringPiSetupGpio();
    
    // Get current time (since Wiring Pi setup was called) in ms and us.
	ms1 = millis();
	us1 = micros();
    
	cout << "Current time:" << endl;
	cout << ms1 << " ms, " << us1 << " us" << endl << endl;
    
    // Wait 500 ms.
	delay(500);
	
    // Get current time again.
	ms2 = millis();
	us2 = micros();
    
	cout << "500 ms later:" << endl;
	cout << ms2 << " ms, " << us2 << " us";
    cout << " (" << (ms2 - ms1) << "ms elapsed, " << (us2 - us1) << "us elapsed)";
    cout << endl << endl;
	
    // Wait 100 us. The Pi will most likely end up waiting significantly longer.
	delayMicroseconds(100);
	
    // Get current time again.
	ms3 = millis();
	us3 = micros();
    
	cout << "100 us later:" << endl;
	cout << ms3 << " ms, " << us3 << " us";
    cout << " (" << (ms3 - ms2) << "ms elapsed, " << (us3 - us2) << "us elapsed)";
    cout << endl << endl;
}
