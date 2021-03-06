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

#Values
lTickCount = 0
rTickCount = 0
lSpeed = 0
rSpeed = 0
currentTime = 0
lRevolutions = 0
rRevolutions = 0
startTime = time.time()

leftset = False
rightset = False
leftflag = False
rightflag = False

# Declaring and defining the left and right servos maps constructed with data generated from calibrateSpeeds()
lPwmTranslation = {}
rPwmTranslation = {}

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
	difference = decimal.Decimal(speed) - decimal.Decimal(1.50)
	return decimal.Decimal(1.50) - decimal.Decimal(difference)

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
        currentLeftSpeed = currentSpeeds[0]
        currentRightSpeed = currentSpeeds[1]
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

def readCalibratedSpeeds():
    global lPwmTranslation, rPwmTranslation

    l = open("LeftSpeedCalibration.txt", "r")
    for line in l:
        currentLine = line.split()
        rpsValue = float(currentLine[0])
        pwmValue = float(currentLine[1])
        lPwmTranslation[rpsValue] = pwmValue
    l.close()

    r = open("RightSpeedCalibration.txt", "r")
    for line in r:
        currentLine = line.split()
        rpsValue = float(currentLine[0])
        pwmValue = float(currentLine[1])
        rPwmTranslation[rpsValue] = pwmValue
    r.close()

    keylistLeft = lPwmTranslation.keys()
    keylistLeft.sort()
    keylistRight = rPwmTranslation.keys()
    keylistRight.sort()

def findPwmValues(rpsLeft, rpsRight):
    if rpsLeft in keylistLeft:
        pwmLeft = lPwmTranslation[rpsLeft]
    elif rpsLeft < min(keylistLeft):
        pwmLeft = lPwmTranslation[keylistLeft[0]]
    else:
        nextElem = 1
        for elem in keylistLeft:
            if elem == max(keylistLeft):
                pwmLeft = lPwmTranslation[elem]
                break
            elif rpsLeft > elem:
                if rpsLeft < keylistLeft[nextElem]:
                    pwmLeft = lPwmTranslation[keylistLeft[nextElem]]
                    break
            nextElem += 1

    if rpsRight in keylistRight:
        pwmRight = rPwmTranslation[rpsRight]
    elif rpsRight < min(keylistRight):
        pwmRight = rPwmTranslation[keylistRight[0]]
    else:
        nextElem = 1
        for elem in keylistRight:
            if elem == max(keylistRight):
                pwmRight = rPwmTranslation[elem]
                break
            elif rpsRight > elem:
                if rpsRight < keylistRight[nextElem]:
                    pwmRight = rPwmTranslation[keylistRight[nextElem]]
                    break
            nextElem += 1



def setSpeedsRPS(rpsLeft, rpsRight):
    decimal.getcontext().prec=2
    print("RPS")
    global leftflag, rightflag, leftset, rightset
    # Calculating pwm values from the respective dictionaries
    left = decimal.Decimal(rpsLeft) + decimal.Decimal(0.00)
    right = decimal.Decimal(rpsRight) + decimal.Decimal(0.00)
    print("left: ", left)
    print("right: ", right)
    leftflag = True
    rightflag = False
    print("RPS2")
    decimal.getcontext().prec=2
    testValue = decimal.Decimal(0.45)
    print("test value", testValue)
    testValue = testValue + decimal.Decimal(0.01)
    #testValue = math.ceil(testValue*100) / 100
    #format(testValue, '.2f')
    print("Test value fixed: ", testValue)

    while leftflag != False:

        #print("RPS3")
        l = open("LeftSpeedCalibration.txt", "r")
        for line in l:
            currentLine = line.split()
            rpsValue = decimal.Decimal(currentLine[0])
            pwmValue = decimal.Decimal(currentLine[1])
            print("rpsValue: ",rpsValue)
            print("pwmValue: ",pwmValue)

            print(left)

            if left == rpsValue:
                print("SUP")
                lPwmValue = decimal.Decimal(pwmValue)
                leftset = True
                leftflag = False
                print("leftflag is set to: ", leftflag)
                break
            elif left > 0.87:
                lPwmValue = 0
                leftset = True
                leftflag = False
                print("leftflag is set to: ", leftflag)
                break
        l.close()
        print(left, " hey")
        left = left + decimal.Decimal(0.01)
        time.sleep(3)

    #rightflag = True

    while rightflag == True:

        r = open("RightSpeedCalibration.txt", "r")
        for line in r:
            currentLine = line.split()
            rpsValue = decimal.Decimal(currentLine[0])
            pwmValue = decimal.Decimal(currentLine[1])
            print("rpsValue: ",rpsValue)
            print("pwmValue: ",pwmValue)

            if right == rpsValue:
                print("SUP")
                rPwmValue = decimal.Decimal(pwmValue)
                rightflag = False
                rightset = True
                break
            elif right > 0.87:
                rPwmValue = 0
                rightflag = False
                break
        r.close()
        print(right, " hey")
        right = right + decimal.Decimal(0.01)
        print (right)

    if rightset == True and leftset == True:
        # Setting appropiate speeds to the servos
        pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))

def setSpeedsIPS(ipsLeft, ipsRight):
    # Converting inches per second into revolutions per second
    decimal.getcontext().prec=2
    rpsLeft = decimal.Decimal(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = decimal.Decimal(math.ceil((ipsRight / 8.20) * 100) / 100)

    print("IPS")

    # Calculating pwm values from the respective dictionaries
    setSpeedsRPS(rpsLeft, rpsRight)

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
