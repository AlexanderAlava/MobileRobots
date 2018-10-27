// This program demonstrates usage of the servos.
// Keep the robot in a safe location before running this program,
// as it will immediately begin moving.
// See https://github.com/Reinbert/pca9685 for more details.
// Also see https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/

#include <iostream>
#include <wiringPi.h>
#include <pca9685.h>
#include <cassert>
#include <csignal>

// The servo hat uses a different numbering scheme within Wiring Pi.
// 100 represents the first servo, 101 for the second, and so on.
#define HATPINBASE 100
#define LSERVO HATPINBASE + 0
#define RSERVO HATPINBASE + 1

int servoHat;

// This function is called when Ctrl+C is pressed.
// It's intended for properly exiting the program.
void ctrlC(int sig)
{
	// Stop the servos
	pca9685PWMReset(servoHat);
	
	exit(0);
}

int main()
{
    // Attach the Ctrl+C signal interrupt
	signal(SIGINT, ctrlC);
	
    // Initialize Wiring Pi, using the pin numbering scheme shown on the 
    // robot itself.
	wiringPiSetupGpio();
	
    // Initialize the servo hat, using the specified numbering scheme.
    // Address 0x40 is used for I2C communication between the Pi and the servo hat.
    // 50Hz is used for the frequency of the servos.
    servoHat = pca9685Setup(HATPINBASE, 0x40, 50);
	assert(servoHat > 0);
	
    // Write an initial value of 1.5, which keeps the servos stopped.
    // Due to how servos work, and the design of the Wiring Pi plugin, 
    // the value must be divided by 20 and then multiplied by 4096.
	pwmWrite(LSERVO, 1.5 / 20 * 4096);
	pwmWrite(RSERVO, 1.5 / 20 * 4096);
	
	while (true)
	{
        // Write a value of 1.6 for each servo.
        // Since the servos are oriented in opposite directions,
        // the robot will end up spinning in one direction.
		// Values between 1.3 and 1.7 should be used.
		pwmWrite(LSERVO, 1.55 / 20 * 4096);
		pwmWrite(RSERVO, 1.55 / 20 * 4096);
		delay(4000);
		
        // Write a value of 1.4 for each servo.
        // The robot will end up spinning in the other direction.
		pwmWrite(LSERVO, 1.45 / 20 * 4096);
		pwmWrite(RSERVO, 1.45 / 20 * 4096);
		delay(4000);
	}
}
