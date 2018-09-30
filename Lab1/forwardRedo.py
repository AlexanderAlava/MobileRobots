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
lSpeed = 0
rSpeed = 0
currentTime = 0
lRevolutions = 0
rRevolutions = 0
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
    global totalCountTuple, lTickCount, rTickCount, startTime
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

    # Increasing tickcount and computing instantaneous values
    lTickCount = lTickCount + 1
    lRevolutions = float(lTickCount / 32)
    currentTime = time.time() - startTime
    lSpeed = lRevolutions / currentTime

# This function is called when the right encoder detects a rising edge signal.
def onRightEncode(pin):
    global rTickCount, rRevolutions, rSpeed, currentTime

    # Increasing tickcount and computing instantaneous values
    rTickCount = rTickCount + 1
    rRevolutions = float(rTickCount / 32)
    currentTime = time.time() - startTime
    rSpeed = rRevolutions / currentTime

# Defining function that flips the speed for the servo that's looking backwards
def servoFlip(speed):
	difference = speed - 1.5
	return 1.5 - difference

# Defining function that initializes the encoders
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

# Function used to produce the traslation dictionaries
def calibrateSpeeds():
    # Starting at full stop
    startVar = 1.7
    endVar = 1.3

    l = open("LeftSpeedCalibration.txt", "w+")
    r = open("RightSpeedCalibration.txt", "w+")

    # Looping until it reaches the maximum required value
    while endVar <= startVar:

        # Setting values for both servos
        pwm.set_pwm(LSERVO, 0, math.floor(startVar / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(startVar) / 20 * 4096))

        # Resetting start time and tick counts
        resetCounts()

        time.sleep(3)

        # Printing speeds to produce respective dictionaries
        print (startVar,getSpeeds())
        time.sleep(3)

        currentSpeeds = getSpeeds()
        currentLeftSpeed = float(math.ceil((currentSpeeds[0]) * 100) / 100)
        currentRightSpeed = float(math.ceil((currentSpeeds[1]) * 100) / 100)
        l.write(str(currentLeftSpeed) + " " + str(startVar) + "\n")
        r.write(str(currentRightSpeed) + " " + str(startVar) + "\n")
        time.sleep(1)


        pwm.set_pwm(LSERVO, 0, math.floor(endVar / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(endVar) / 20 * 4096))

        # Resetting start time and tick counts
        resetCounts()

        time.sleep(3)

        # Printing speeds to produce respective dictionaries
        print (endVar,getSpeeds())
        time.sleep(3)

        currentSpeeds = getSpeeds()
        currentLeftSpeed = float(math.ceil((currentSpeeds[0]) * 100) / 100)
        currentRightSpeed = float(math.ceil((currentSpeeds[1]) * 100) / 100)
        l.write("-" + str(currentLeftSpeed) + " " + str(endVar) + "\n")
        r.write("-" + str(currentRightSpeed) + " " + str(endVar) + "\n")
        time.sleep(1)

        # Increasing pwm value
        startVar = startVar - 0.005

        endVar = endVar + 0.005

        # Resetting start time and tick counts
        resetCounts()

    l.close()
    r.close()

def setSpeedsRPS(rpsLeft, rpsRight):
	
    print("RPS")
	
    # Calculating pwm values from the respective dictionaries

    l = open("LeftSpeedCalibration.txt", "r")
    r = open("RightSpeedCalibration.txt", "r")
    left = rpsLeft
    right = rpsRight
    flag = True

    print("RPS2")

    while flag:

        #print("RPS3")

        for line in l:
            currentLine = line.split()
            rpsValue = float(currentLine[0])
            pwmValue = float(currentLine[1])

            print(left)

            if left == rpsValue:
                print("SUP")
                lPwmValue = pwmValue
                flag = False
                break
            elif left > 0.87:
                lPwmValue = 0
                flag = False
                break

        print(left, " hey")
        left = float((math.ceil(left + 0.01) * 100) / 100)
        time.sleep(3)

    flag = True

    while flag:

        for line in r:
            currentLine = line.split()
            rpsValue = float(currentLine[0])
            pwmValue = float(currentLine[1])

            if right == rpsValue:
                rPwmValue = pwmValue
                flag = False
                break
            elif right > 0.87:
                rPwmValue = 0
                flag = False
                break

        right = float((math.ceil(right + 0.01) * 100) / 100)
        print (right)

    # Setting appropiate speeds to the servos
    pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))

    return 0

def setSpeedsIPS(ipsLeft, ipsRight):
    # Converting inches per second into revolutions per second
    rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

    print("IPS")

    # Calculating pwm values from the respective dictionaries
    setSpeedsRPS(rpsLeft, rpsRight)

    return 0

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

# Initializing encoders
initEncoders()

# Declaring variable to keep track of distance traveled
distanceTravel = 0


#calibrateSpeeds()


############################### Main code ##################################

# Prompting for user input for distance and time
xInches = input("Enter number of inches to travel: ")
yTime = input("Enter time to complete set distance: ")

# Setting boolean flag to False initially
goodValue = False

# Establishing the maximum value in inches per second
maxValue = 7.134

# Calculating the inches per second
ipsValue = float(math.ceil((float(xInches) / float(yTime)) * 100) / 100)
print (ipsValue)

# Checking if the entered input is possible and asking for new input if needed
while goodValue != True:
    if ipsValue > maxValue:
        print("Sorry but that request can not be completed. Please try again")
        xInches = input("Enter number of inches to travel: ")
        yTime = input("Enter time to complete set distance: ")
        ipsValue = float(math.ceil((float(xInches) / float(yTime)) * 100) / 100)
        print(ipsValue)
    elif ipsValue < 0:
        print("Sorry but I am only supposed to move forwards, not backwards. Please try again")
        xInches = input("Enter number of inches to travel: ")
        yTime = input("Enter time to complete set distance: ")
        ipsValue = float(math.ceil((float(xInches) / float(yTime)) * 100) / 100)
        print(ipsValue)
    else:
	    goodValue = True

newInput = ''

while newInput != 's':
    newInput = input("Please enter \'s\' to have the robot start it's movement: ")

while True:
    setSpeedsIPS(ipsValue, ipsValue)

    # Checking if the robot has already traveled the required distance
    distanceTravel = (8.20 * ((lRevolutions + rRevolutions) / 2))
    if (float(xInches) - float(distanceTravel)) <= 0.00:
	    # Stopping both servos and exiting the loop
        pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096));
        pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096));
        exit()

    # Printing final numbers
    print("Number of Revolutions: ",(lRevolutions + rRevolutions) / 2)
    print("Distance Traveled: ", distanceTravel)
