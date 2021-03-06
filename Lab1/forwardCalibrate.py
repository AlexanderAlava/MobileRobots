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
lPwmTranslation = {}
rPwmTranslation = {}

# Declaring list for the dictionaries keys
keylistLeft = []
keylistRight = []

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

def setSpeedsRPS(rpsLeft, rpsRight):
    # Calculating pwm values from the respective dictionaries
    pwmValues = findPwmValues(rpsLeft, rpsRight)

    lPwmValue = float(pwmValues[0])
    rPwmValue = float(pwmValues[1])

    # Setting appropiate speeds to the servos
    pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))

    return 0

def setSpeedsIPS(ipsLeft, ipsRight):
    # Converting inches per second into revolutions per second
    rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

    # Calling the setSpeedsRPS function with the appropiate parameters
    setSpeedsRPS(rpsLeft, rpsRight)

    return 0

# Function used to produce the traslation dictionaries
def calibrateSpeeds():
    # Starting at full stop
    startVar = 1.7
    endVar = 1.3

    # Opening files for storing calibration values
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

        # Reading the current speeds for both wheels and storing them in their respective file
        currentSpeeds = getSpeeds()
        currentLeftSpeed = currentSpeeds[0]
        currentRightSpeed = currentSpeeds[1]
        l.write(str(currentLeftSpeed) + " " + str(startVar) + "\n")
        r.write(str(currentRightSpeed) + " " + str(startVar) + "\n")
        time.sleep(1)

        # Setting values for both servos
        pwm.set_pwm(LSERVO, 0, math.floor(endVar / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(endVar) / 20 * 4096))

        # Resetting start time and tick counts
        resetCounts()

        time.sleep(3)

        # Printing speeds to produce respective dictionaries
        print (endVar,getSpeeds())
        time.sleep(3)

        # Reading the current speeds for both wheels and storing them in their respective file
        currentSpeeds = getSpeeds()
        currentLeftSpeed = currentSpeeds[0]
        currentRightSpeed = currentSpeeds[1]
        l.write("-" + str(currentLeftSpeed) + " " + str(endVar) + "\n")
        r.write("-" + str(currentRightSpeed) + " " + str(endVar) + "\n")
        time.sleep(1)

        # Increasing pwm value
        startVar = startVar - 0.005

        endVar = endVar + 0.005

        # Resetting start time and tick counts
        resetCounts()

    # Closing both files
    l.close()
    r.close()

# Function used to read the files with the calibration values and creating dictionaries storing the needed values
def readCalibratedSpeeds():
    global lPwmTranslation, rPwmTranslation, keylistLeft, keylistRight

    # Reading all calibration values for the left wheel and storing them in their respective dictonary
    l = open("LeftSpeedCalibration.txt", "r")
    for line in l:
        currentLine = line.split()
        rpsValue = float(currentLine[0])
        pwmValue = float(currentLine[1])
        lPwmTranslation[rpsValue] = pwmValue
    l.close()

    # Reading all calibration values for the right wheel and storing them in their respective dictonary
    r = open("RightSpeedCalibration.txt", "r")
    for line in r:
        currentLine = line.split()
        rpsValue = float(currentLine[0])
        pwmValue = float(currentLine[1])
        rPwmTranslation[rpsValue] = pwmValue
    r.close()

    # Creating a list of the keys of the left wheel dictionary and sorting it
    keylistLeft = list(lPwmTranslation.keys())
    keylistLeft.sort()

    # Creating a list of the keys of the right wheel dictionary and sorting it
    keylistRight = list(rPwmTranslation.keys())
    keylistRight.sort()

# Function that translates rps values into pwm values
def findPwmValues(rpsLeft, rpsRight):
    # Checking if the rps value exists exactly on our calibrated values
    if rpsLeft in keylistLeft:
        pwmLeft = lPwmTranslation[rpsLeft]
    # Checking if the rps value is less than the possible minimum
    elif rpsLeft < min(keylistLeft):
        pwmLeft = lPwmTranslation[keylistLeft[0]]
    else:
        # Tracker for the next element
        nextElem = 1
        # Iterating through all ordered rpm values in order to find the closest one possible
        for elem in keylistLeft:
            # Checking if the list has reached its maximum value and therefore returning it
            if elem == max(keylistLeft):
                pwmLeft = lPwmTranslation[elem]
                break
            elif rpsLeft > elem:
                # Checking if the element falls within two values and returning the highest one
                if rpsLeft < keylistLeft[nextElem]:
                    pwmLeft = lPwmTranslation[keylistLeft[nextElem]]
                    break
            # Increasing tracker
            nextElem += 1

    # Checking if the rps value exists exactly on our calibrated values
    if rpsRight in keylistRight:
        pwmRight = rPwmTranslation[rpsRight]
    # Checking if the rps value is less than the possible minimum
    elif rpsRight < min(keylistRight):
        pwmRight = rPwmTranslation[keylistRight[0]]
    else:
        # Tracker for the next element
        nextElem = 1
        # Iterating through all ordered rpm values in order to find the closest one possible
        for elem in keylistRight:
            # Checking if the list has reached its maximum value and therefore returning it
            if elem == max(keylistRight):
                pwmRight = rPwmTranslation[elem]
                break
            elif rpsRight > elem:
                # Checking if the element falls within two values and returning the highest one
                if rpsRight < keylistRight[nextElem]:
                    pwmRight = rPwmTranslation[keylistRight[nextElem]]
                    break
            # Increasing tracker
            nextElem += 1

    return (pwmLeft, pwmRight)

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


############################### Main code ##################################

#calibrateSpeeds()
readCalibratedSpeeds()

# Prompting for user input for distance and time
xInches = input("Enter number of inches to travel: ")
yTime = input("Enter time to complete set distance: ")

# Setting boolean flag to False initially
goodValue = False

# Establishing the maximum value in inches per second
maxValue = 7.134

# Calculating the inches per second
ipsValue = (float(xInches) / float(yTime))

# Checking if the entered input is possible and asking for new input if needed
while goodValue != True:
    if ipsValue > maxValue:
        print("Sorry but that request can not be completed. Please try again")
        xInches = input("Enter number of inches to travel: ")
        yTime = input("Enter time to complete set distance: ")
        ipsValue = (float(xInches) / float(yTime))
    elif ipsValue < 0:
        print("Sorry but I am only supposed to move forwards, not backwards. Please try again")
        xInches = input("Enter number of inches to travel: ")
        yTime = input("Enter time to complete set distance: ")
        ipsValue = (float(xInches) / float(yTime))
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
