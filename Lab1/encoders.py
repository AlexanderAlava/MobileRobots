<<<<<<< HEAD
# This program demonstrates usage of the digital encoders.
# After executing the program, manually spin the wheels and observe the output.
# See https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/ for more details.

import time
import RPi.GPIO as GPIO
import signal

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

#Values
LTickCount = 0
RTickCount = 0
totalCountTuple = (0, 0)

#Function that resets the total count of ticks
def resetCounts():
	totalCountTuple[0] = totalCountTuple[0] + LTickCount
	totalCountTuple[1] = totalCountTuple[1] + RTickCount
	LTickCount = 0
	RTickCount = 0

#Function that gets previous tick counts 
def getCounts():
	return (LTickCount, RTickCount)

#Function that returns instantaneous left and right wheel speeds
def getSpeeds():
	
# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    print("Left encoder ticked!")
	LTickCount = LTickCount + 1

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    print("Right encoder ticked!")
	RTickCount = RTickCount + 1
	
# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Set encoder pins as input
# Also enable pull-up resistors on the encoder pins
# This ensures a clean 0V and 3.3V is always outputted from the encoders.
GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach a rising edge interrupt to the encoder pins
GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

# Prevent the program from exiting by adding a looping delay.
while True:
    time.sleep(1)
	function = input("Press 'G' for getCounts, Press 'R' to resetCounts")
	if function.lower() == "g":
        getCounts()
	if function.lower() == "r":
        resetCounts()
		getCounts()
=======
# This program demonstrates usage of the digital encoders.
# After executing the program, manually spin the wheels and observe the output.
# See https://sourceforge.net/p/raspberry-gpio-python/wiki/Inputs/ for more details.

import time
import RPi.GPIO as GPIO
import signal

# Pins that the encoders are connected to
LENCODER = 17
RENCODER = 18

# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    print("Left encoder ticked!")

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    print("Right encoder ticked!")

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Set encoder pins as input
# Also enable pull-up resistors on the encoder pins
# This ensures a clean 0V and 3.3V is always outputted from the encoders.
GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Attach a rising edge interrupt to the encoder pins
GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)

# Prevent the program from exiting by adding a looping delay.
while True:
    time.sleep(1)
>>>>>>> 91e207eca849da5641876bc6c6ca10e2b6e945e6
