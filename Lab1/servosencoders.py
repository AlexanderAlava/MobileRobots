import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import signal
import math

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    
    # Stop the servos
    pwm.set_pwm(LSERVO, 0, 0);
    pwm.set_pwm(RSERVO, 0, 0);
    
    exit()

# Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library, 
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
#################################################################################
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
	
def servoFlip(speed):
	difference = speed - 1.5
	return 1.5 - difference

def initEncoders():
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
	
# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlC(signum, frame):
    print("Exiting")
    GPIO.cleanup()
    # Write an initial value of 1.5, which keeps the servos stopped.
    # Due to how servos work, and the design of the Adafruit library, 
    # the value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
    time.sleep(3)
    exit()

# This function is called when Ctrl+C is pressed.
# It's intended for properly exiting the program.
def ctrlD(signum, frame):
    print("Printing Speeds:", getSpeeds())

## Attach the Ctrl+C signal interrupt
#signal.signal(signal.SIGINT, ctrlC)
    
## Set the pin numbering scheme to the numbering shown on the robot itself.
#GPIO.setmode(GPIO.BCM)

## Set encoder pins as input
## Also enable pull-up resistors on the encoder pins
## This ensures a clean 0V and 3.3V is always outputted from the encoders.
#GPIO.setup(LENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#GPIO.setup(RENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)

## Attach a rising edge interrupt to the encoder pins
#GPIO.add_event_detect(LENCODER, GPIO.RISING, onLeftEncode)
#GPIO.add_event_detect(RENCODER, GPIO.RISING, onRightEncode)
############################################################################
initEncoders()
time.sleep(5)

while True:
    # Write a maximum value of 1.7 for each servo.
    # Since the servos are oriented in opposite directions,
    # the robot will end up spinning in one direction.
    # Values between 1.3 and 1.7 should be used.
   # pwm.set_pwm(LSERVO, 0, math.floor(1.7 / 20 * 4096));
    #pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(1.7) / 20 * 4096));
    
    #time.sleep(0.5)
    
    # Write a minimum value of 1.4 for each servo.
    # The robot will end up spinning in the other direction.
   # pwm.set_pwm(LSERVO, 0, math.floor(1.7 / 20 * 4096));
   
   # pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(1.7) / 20 * 4096));
    #time.sleep(0.5)
	time.sleep(1)
