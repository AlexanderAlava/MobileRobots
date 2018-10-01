import time
import Adafruit_PCA9685
import RPi.GPIO as GPIO
import signal
import math
import decimal

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

# Declaring and initializing global variables
lTickCount = 0
rTickCount = 0
totalCountTuple = (0, 0)
lSpeed = 0
rSpeed = 0
currentTime = 0
lRevolutions = 0
rRevolutions = 0
distanceTravel = 0
startTime = time.time()

# Declaring and defining the left and right servos maps constructed with data generated from calibrateSpeeds()
lPwmTranslation = {
                    0.00: 1.50, 0.01: 1.505, 0.02: 1.505, 0.03: 1.51, 0.04: 1.51,
                    0.05: 1.51, 0.06: 1.51, 0.07: 1.51, 0.08: 1.515, 0.09: 1.515,
                    0.10: 1.515, 0.11: 1.515, 0.12: 1.515, 0.13: 1.52, 0.14: 1.52,
                    0.15: 1.52, 0.16: 1.52, 0.17: 1.52, 0.18: 1.525, 0.19: 1.525,
                    0.20: 1.525, 0.21: 1.525, 0.22: 1.525, 0.23: 1.525, 0.24: 1.53,
                    0.25: 1.53, 0.26: 1.53, 0.27: 1.53, 0.28: 1.53, 0.29: 1.535,
                    0.30: 1.535, 0.31: 1.535, 0.32: 1.535, 0.33: 1.54, 0.34: 1.54,
                    0.35: 1.54, 0.36: 1.54, 0.37: 1.54, 0.38: 1.545, 0.39: 1.545,
                    0.40: 1.545, 0.41: 1.545, 0.42: 1.545, 0.43: 1.55, 0.44: 1.55,
                    0.45: 1.55, 0.46: 1.55, 0.47: 1.56, 0.48: 1.555, 0.49: 1.555,
                    0.50: 1.555, 0.51: 1.555, 0.52: 1.555, 0.53: 1.56, 0.54: 1.56,
                    0.55: 1.56, 0.56: 1.56, 0.57: 1.56, 0.58: 1.565, 0.59: 1.565,
                    0.60: 1.565, 0.61: 1.565, 0.62: 1.565, 0.63: 1.57, 0.64: 1.57,
                    0.65: 1.57, 0.66: 1.57, 0.67: 1.57, 0.68: 1.575, 0.69: 1.575,
                    0.70: 1.58, 0.71: 1.58, 0.72: 1.585, 0.73: 1.585, 0.74: 1.59,
                    0.75: 1.59, 0.76: 1.59, 0.77: 1.60, 0.78: 1.60, 0.79: 1.61,
                    0.80: 1.61, 0.81: 1.61, 0.82: 1.61, 0.83: 1.62, 0.84: 1.63,
                    0.85: 1.64, 0.86: 1.65, 0.87: 1.70
                    }
rPwmTranslation = {
                    0.00: 1.50, 0.01: 1.50, 0.02: 1.50, 0.03: 1.505, 0.04: 1.505,
                    0.05: 1.505, 0.06: 1.505, 0.07: 1.505, 0.08: 1.51, 0.09: 1.51,
                    0.10: 1.51, 0.11: 1.51, 0.12: 1.51, 0.13: 1.515, 0.14: 1.515,
                    0.15: 1.515, 0.16: 1.515, 0.17: 1.515, 0.18: 1.52, 0.19: 1.52,
                    0.20: 1.52, 0.21: 1.52, 0.22: 1.52, 0.23: 1.525, 0.24: 1.525,
                    0.25: 1.525, 0.26: 1.525, 0.27: 1.52, 0.28: 1.53, 0.29: 1.53,
                    0.30: 1.53, 0.31: 1.53, 0.32: 1.53, 0.33: 1.535, 0.34: 1.535,
                    0.35: 1.535, 0.36: 1.535, 0.37: 1.54, 0.38: 1.54, 0.39: 1.54,
                    0.40: 1.54, 0.41: 1.54, 0.42: 1.54, 0.43: 1.5425, 0.44: 1.5425,
                    0.45: 1.5425, 0.46: 1.545, 0.47: 1.545, 0.48: 1.5475, 0.49: 1.5475,
                    0.50: 1.5475, 0.51: 1.5475, 0.52: 1.55, 0.53: 1.55, 0.54: 1.55,
                    0.55: 1.55, 0.56: 1.555, 0.57: 1.555, 0.58: 1.555, 0.59: 1.56,
                    0.60: 1.56, 0.61: 1.56, 0.62: 1.56, 0.63: 1.56, 0.64: 1.565,
                    0.65: 1.565, 0.66: 1.565, 0.67: 1.565, 0.68: 1.57, 0.69: 1.57,
                    0.70: 1.57, 0.71: 1.57, 0.72: 1.575, 0.73: 1.575, 0.74: 1.58,
                    0.75: 1.58, 0.76: 1.58, 0.77: 1.59, 0.78: 1.59, 0.79: 1.60,
                    0.80: 1.60, 0.81: 1.60, 0.82: 1.61, 0.83: 1.62, 0.84: 1.62,
                    0.85: 1.62, 0.86: 1.65, 0.87: 1.70
                    }

#Function that resets the total count of ticks
def resetCounts():
    global totalCountTuple, lTickCount, rTickCount, startTime, distanceTravel
    lTickCount = 0
    rTickCount = 0
    distanceTravel = 0
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
    global lTickCount, lRevolutions, lSpeed, currentTime, distanceTravel
    #print("Left encoder ticked!")
    lTickCount = lTickCount + 1
    lRevolutions = float(lTickCount / 32)
    currentTime = time.time() - startTime
    lSpeed = lRevolutions / currentTime
    print("Distance: ", distanceTravel)

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global rTickCount, rRevolutions, rSpeed, currentTime, distanceTravel
    #print("Right encoder ticked!")
    rTickCount = rTickCount + 1
    rRevolutions = float(rTickCount / 32)
    currentTime = time.time() - startTime
    rSpeed = rRevolutions / currentTime
    print("Distance: ", distanceTravel)

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

def calibrateSpeeds():
    startVar = 1.5

    # Looping until it reaches the maximum required value
    while startVar < 1.71:
        # Setting values for both servos
        pwm.set_pwm(LSERVO, 0, math.floor(startVar / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(startVar) / 20 * 4096))
        time.sleep(10)

        # Printing speeds to produce respective dictionaries
        print (startVar,getSpeeds())
        time.sleep(5)

        # Increasing pwm value
        startVar = startVar + 0.005

        # Resetting start time and tick counts
        resetCounts()

# Defining the function that sets speed to revolutions per second
def setSpeedsRPS(rpsLeft, rpsRight):
    lPwmValue = float(lPwmTranslation[rpsLeft])
    rPwmValue = float(rPwmTranslation[rpsRight])
    pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))


# Defining the function that sets speed to inches per second
def setSpeedsIPS(ipsLeft, ipsRight):
    rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)
    lPwmValue = float(lPwmTranslation[rpsLeft])
    rPwmValue = float(rPwmTranslation[rpsRight])
    pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))
   

# Defining the speed function for the first arc
def setSpeedsvw1(v, w):
    leftSpeed1 = (v + (w*daxis))
    rightSpeed1 = (v - (w*daxis))
    #print(leftSpeed1, rightSpeed1)
    setSpeedsIPS(leftSpeed1, rightSpeed1)

# Defining the speed function for the second arc
def setSpeedsvw2(v, w):
    leftSpeed2 = (v - (w*daxis))
    rightSpeed2 = (v + (w*daxis))
    #print(leftSpeed2, rightSpeed2)
    setSpeedsIPS(leftSpeed2, rightSpeed2)

## Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)	
initEncoders()	
	
# Prompting for and reading in user input
circleRadius1 = 0
circleRadius2 = 0
circleTime = 0
circleRadius1 = input("Enter radius R for circle 1: ")
if(circleRadius1 < 2.0):
    print("Entered radius is too small and can not be completed.")
	exit()
circleRadius2 = input("Enter radius R for circle 2: ")
if(circleRadius2 < 2.0):
    print("Entered radius is too small and can not be completed.")
    exit()	
circleTime = input("Enter time to complete circles: ")

# Computing required values
daxis = float(3.95)
arcPath1 = float(3.14159)*(float(circleRadius1))
arcPath2 = float(3.14159)*(float(circleRadius2))
if(circleTime > 0):
    linearSpeed = (float(arcPath1) + float(arcPath2))/float(circleTime)   
else:
    print("The time entered to complete S shape is imposible")
    exit()
if((linearSpeed > 7.134) or (linearSpeed < 0)):
    print("The computed linear speed for the S shape is greater than the robot's max speed of 7.134 ips")
    exit()
	
omega1 = float(linearSpeed)/float(circleRadius1)
omega2 = float(linearSpeed)/float(circleRadius2)

#Computeing linear velocities for each wheel to ensure that it is possible to perform S shape.
leftLinearSpeed1 = (omega1 * (circleRadius1 + 1.985))
rightLinearSpeed1 = (omega1 * (circleRadius1 - 1.985))
leftLinearSpeed2 = (omega2 * (circleRadius2 + 1.985))
rightLinearSpeed2 = (omega2 * (circleRadius2 - 1.985))
if((leftLinearSpeed1 > 7.134) or (leftLinearSpeed2 > 7.134) or (rightLinearSpeed1 > 7.134) or (rightLinearSpeed2 > 7.134)):
    print("Computed speed for individual wheel exceeds the maximum")
	exit()

firstFlag = True

while firstFlag == True:
    # Setting speed for first arc
    setSpeedsvw1(linearSpeed, omega1)
    resetCounts()
    distanceTravel = (8.20 * ((lRevolutions + rRevolutions) / 2))

    # Checking if the distance of the first arch has been traveled
    if (float(arcPath1) - float(distanceTravel)) <= 0.00:
        firstFlag = False
        print("Flag was set to false")
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));

resetCounts()
newInput = ''

# Waiting for user input to proceed
while newInput != 'm':
	newInput = input("Please enter \'m\' to continue the movement: ")

secondFlag = True
while secondFlag == True:
    # Setting speed for second arc
    setSpeedsvw2(linearSpeed, omega2)
    resetCounts()
    distanceTravel = (8.20 * ((lRevolutions + rRevolutions) / 2))

    # Checking if the distance of the fi arch has been traveled
    if (float(arcPath2) - float(distanceTravel)) <= 0.00:
        secondFlag = False
        print("Second flag was set to false")
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
