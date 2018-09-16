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
lTickCount = 0
rTickCount = 0
totalCountTuple = (0, 0)
lSpeed = 0
rSpeed = 0
currentTime = 0
lRevolutions = 0
rRevolutions = 0
startTime = time.time()

#Function that resets the total count of ticks
def resetCounts():
    global totalCountTuple, lTickCount, rTickCount
    totalCountTuple[0] = totalCountTuple[0] + lTickCount
    totalCountTuple[1] = totalCountTuple[1] + rTickCount
    lTickCount = 0
    rTickCount = 0

#Function that gets previous tick counts 
def getCounts():
    return (lTickCount, rTickCount)

#Function that returns instantaneous left and right wheel speeds
def getSpeeds():
    global lTickCount, rTickCount, CurrentTime, lSpeed, rSpeed
    lSpeed = (lTickCount / 32) / CurrentTime
    rSpeed = (rTickCount / 32) / CurrentTime
	
# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    global lTickCount, lRevolutions, lSpeed, currentTime
    print("Left encoder ticked!")
    lTickCount = lTickCount + 1
    lRevolutions = float(lTickCount / 32)
    currentTime = time.time() - startTime
    lSpeed = lRevolutions / currentTime
    print ("LTicks ", lTickCount)
    print ("LRevolutions ", lRevolutions)
    print ("LTime ", currentTime)
    print ("LSpeed ", lSpeed)

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global rTickCount, rRevolutions, rSpeed, currentTime
    print("Right encoder ticked!")
    rTickCount = rTickCount + 1
    rRevolutions = float(rTickCount / 32)
    currentTime = time.time() - startTime
    rSpeed = rRevolutions / currentTime
    print ("RTicks ", rTickCount)
    print ("RRevolutions ", rRevolutions)
    print ("RTime ", currentTime)
    print ("RSpeed ", rSpeed)
	
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
	#function = input("Press 'G' for getCounts, Press 'R' to resetCounts")
	#if function.lower() == "g":
        #getCounts()
	#if function.lower() == "r":
        #resetCounts()ss
		#getCounts()	
    
