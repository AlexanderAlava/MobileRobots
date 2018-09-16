import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import signal
import math

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1
    
# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

# Write an initial value of 1.5, which keeps the servos stopped.
# Due to how servos work, and the design of the Adafruit library, 
# the value must be divided by 20 and multiplied by 4096.
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

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
    global totalCountTuple, lTickCount, rTickCount, startTime
    #totalCountTuple[0] = totalCountTuple[0] + lTickCount
    #totalCountTuple[1] = totalCountTuple[1] + rTickCount
    lTickCount = 0
    rTickCount = 0
    startTime = time.time()

#Function that gets previous tick counts 
def getCounts():
    return (lTickCount, rTickCount)

#Function that returns instantaneous left and right wheel speeds
def getSpeeds():
    global lTickCount, rTickCount, currentTime, lSpeed, rSpeed
    currentTime = time.time() - startTime
    lSpeed = (lTickCount / 32) / currentTime
    rSpeed = (rTickCount / 32) / currentTime
    return (lSpeed, rSpeed)
	
# This function is called when the left encoder detects a rising edge signal.
def onLeftEncode(pin):
    global lTickCount, lRevolutions, lSpeed, currentTime
    #print("Left encoder ticked!")
    lTickCount = lTickCount + 1
    lRevolutions = float(lTickCount / 32)
    currentTime = time.time() - startTime
    lSpeed = lRevolutions / currentTime
    #print ("LTicks ", lTickCount)
    #print ("LRevolutions ", lRevolutions)
    #print ("LTime ", currentTime)
    #print ("LSpeed ", lSpeed)

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global rTickCount, rRevolutions, rSpeed, currentTime
    #print("Right encoder ticked!")
    rTickCount = rTickCount + 1
    rRevolutions = float(rTickCount / 32)
    currentTime = time.time() - startTime
    rSpeed = rRevolutions / currentTime
    #print ("RTicks ", rTickCount)
    #print ("RRevolutions ", rRevolutions)
    #print ("RTime ", currentTime)
    #print ("RSpeed ", rSpeed)
	
def servoFlip(speed):
	difference = speed - 1.5
	return 1.5 - difference

def initEncoders(): 
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
    ## Write an initial value of 1.5, which keeps the servos stopped.
    ## Due to how servos work, and the design of the Adafruit library, 
    ## the value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    print("Speed", getSpeeds())
    time.sleep(3)
    exit()

## Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)	
initEncoders()
time.sleep(5)

possibleInputs = (1.30,1.31, 1.32, 1.33, 1.34, 1.35, 1.36, 1.37, 1.38, 1.39,
1.40, 1.41, 1.42, 1.43, 1.44, 1.45, 1.46, 1.47, 1.48, 1.49,
1.50, 1.51, 1.52, 1.53, 1.54, 1.55, 1.56, 1.57, 1.58, 1.59, 
1.60, 1.61, 1.62, 1.63, 1.64, 1.65, 1.66, 1.67, 1.68, 1.69,
1.70)

testVar = 1.6

while True:
    # Write a maximum value of 1.7 for each servo.
    # Since the servos are oriented in opposite directions,
    # the robot will end up spinning in one direction.
    # Values between 1.3 and 1.7 should be used.
    pwm.set_pwm(LSERVO, 0, math.floor(testVar / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(testVar) / 20 * 4096))
    
    # Write a minimum value of 1.4 for each servo.
    # The robot will end up spinning in the other direction.
    pwm.set_pwm(LSERVO, 0, math.floor(testVar / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(testVar) / 20 * 4096))
    time.sleep(10)
    print (testVar,getSpeeds())
    time.sleep(5)
    testVar = testVar + 0.01
    resetCounts()
    
