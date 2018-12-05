import time
import Adafruit_PCA9685
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import math
import random

# Declaring and defining the left and right servos maps constructed with data generated from calibrateSpeeds(
lPwmTranslation = {
                    0.00: 1.50, 0.01: 1.50, 0.02: 1.50, 0.03: 1.50, 0.04: 1.50,
                    0.05: 1.50, 0.06: 1.51, 0.07: 1.50, 0.08: 1.515, 0.09: 1.515,
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
                    0.00: 1.50, 0.01: 1.50, 0.02: 1.50, 0.03: 1.50, 0.04: 1.50,
                    0.05: 1.50, 0.06: 1.50, 0.07: 1.50, 0.08: 1.51, 0.09: 1.51,
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

# The servo hat uses its own numbering scheme within the Adafruit library.
# 0 represents the first servo, 1 for the second, and so on.
LSERVO = 0
RSERVO = 1

# Initialize the servo hat library.
pwm = Adafruit_PCA9685.PCA9685()

# 50Hz is used for the frequency of the servos.
pwm.set_pwm_freq(50)

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

# Pins that the sensors are connected to
LSHDN = 27
FSHDN = 22
RSHDN = 23

DEFAULTADDR = 0x29 # All sensors use this address by default, don't change this
LADDR = 0x2a
RADDR = 0x2b

# Set the pin numbering scheme to the numbering shown on the robot itself.
GPIO.setmode(GPIO.BCM)

# Setup pins
GPIO.setup(LSHDN, GPIO.OUT)
GPIO.setup(FSHDN, GPIO.OUT)
GPIO.setup(RSHDN, GPIO.OUT)

# Shutdown all sensors
GPIO.output(LSHDN, GPIO.LOW)
GPIO.output(FSHDN, GPIO.LOW)
GPIO.output(RSHDN, GPIO.LOW)

time.sleep(0.01)

# Initialize all sensors
lSensor = VL53L0X.VL53L0X(address=LADDR)
fSensor = VL53L0X.VL53L0X(address=DEFAULTADDR)
rSensor = VL53L0X.VL53L0X(address=RADDR)

# Connect the left sensor and start measurement
GPIO.output(LSHDN, GPIO.HIGH)
time.sleep(0.01)
lSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the right sensor and start measurement
GPIO.output(RSHDN, GPIO.HIGH)
time.sleep(0.01)
rSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

# Connect the front sensor and start measurement
GPIO.output(FSHDN, GPIO.HIGH)
time.sleep(0.01)
fSensor.start_ranging(VL53L0X.VL53L0X_GOOD_ACCURACY_MODE)

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

def ctrlC(signum, frame):
    print("DONE")
    GPIO.cleanup()

    ## Write an initial value of 1.5, which keeps the servos stopped.
    ## Due to how servos work, and the design of the Adafruit library,
    ## the value must be divided by 20 and multiplied by 4096.
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    # Stop measurement for all sensors
    lSensor.stop_ranging()
    fSensor.stop_ranging()
    rSensor.stop_ranging()
    #
    # while(True):
    #     startInput = input("Press 'm' to print out final completed map of maze.")
    #     if startInput == "m":
    #         print("\nMAP OF MAZE:\n")
    #         printMaze(maze)
    #         exit()
    #     else:
	#         exit()
    exit()

# Function that flips pwm values since servos are in opposite directions
def servoFlip(speed):
	difference = speed - 1.5
	return 1.5 - difference

# Function that translates speeds from ips to pwm
def setSpeedsIPS(ipsLeft, ipsRight):
    # Converting inches per second into revolutions per second
    rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

    # Flipping RPS values when negative in order to use the appropiate pwm values
    if rpsLeft < 0:
        rpsLeft = 0 - rpsLeft
    if rpsRight < 0:
        rpsRight = 0 - rpsRight

    # Calculating pwm values from the respective dictionaries
    lPwmValue = float(lPwmTranslation[rpsLeft])
    rPwmValue = float(rPwmTranslation[rpsRight])

    if ipsLeft < 0 and ipsRight < 0:
        # Setting appropiate speeds to the servos when going forwards
        pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))
    elif ipsLeft >= 0 and ipsRight >= 0:
        # Setting apporpiate speeds to the servos when going backwards
        pwm.set_pwm(LSERVO, 0, math.floor(servoFlip(lPwmValue) / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))
    elif ipsLeft >= 0 and ipsRight < 0:
		# Setting apporpiate speeds to the servos when turning
        pwm.set_pwm(LSERVO, 0, math.floor(servoFlip(lPwmValue) / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))

# Function to set appropiate boundaries for front sensor
def saturationFunction(ips):
    controlSignal = ips

    # Value of 0.5 is used to limit the range of speeds possible and was reached through trial and error
    if controlSignal > 0.5:
        controlSignal = 0.5
    elif controlSignal < -0.5:
        controlSignal = -0.5
    return controlSignal

# Function to set appropiate boundaries for right sensor
def saturationFunctionRight(inches):
    controlSignal = inches

    #Value of 0.5 is used to limit the range of speeds possible and was reached through trial and error
    if controlSignal > 0.5:
        controlSignal = 0.5
    elif controlSignal < -0.5:
        controlSignal = -0.5
    return controlSignal

# Function to make a left turn when needed
def turnLeft():
	# Reading in from sensor
    fDistance = fSensor.get_distance()

    # Transforming readings to inches
    inchesDistance = fDistance * 0.0393700787

    while inchesDistance < 10:
        if inchesDistance < 3:
            pwm.set_pwm(LSERVO, 0, math.floor(1.47 / 20 * 4096))
            pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(1.47) / 20 * 4096))

            time.sleep(1.2)
        pwm.set_pwm(LSERVO, 0, math.floor(1.49 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.42 / 20 * 4096))
        #setSpeedsIPS(1.3, -2)

        # Reading in from sensor
        fDistance = fSensor.get_distance()

        # Transforming readings to inches
        inchesDistance = fDistance * 0.0393700787
    time.sleep(0.25)
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))

# Function to set appropiate boundaries for front sensor
def saturationFunctionFront(ips):
    controlSignal = ips
    if controlSignal > 7.1:
        controlSignal = 7.1
    elif controlSignal < -7.1:
        controlSignal = -7.1
    return controlSignal

def adjustFront():
    global lRevolutions, rRevolutions

    frontCount = 0

    while True:
        # Reading in from sensor
        fDistance = fSensor.get_distance()

        # Transforming readings to inches
        inchesDistance = fDistance * 0.0393700787

        # Calculating respective error
        errorFrontAd = 8 - inchesDistance

        # Computing the control signal
        controlSignal = kpValue * errorFrontAd

        # Running control signals through saturation function
        newSignal = saturationFunctionFront(controlSignal)

        # Setting speed of the robot with the newly computed values
        setSpeedsIPS(newSignal, newSignal)

        if errorFrontAd < 0.5 and errorFrontAd > -0.5:
            break

        frontCount = frontCount + 1
        #print(frontCount)

        if frontCount > 80:
            #print("I AM BREEEEEAKIIIIIIIIING FREEEEEEEEEEEEEEEEE")
            break

    lRevolutions = 1.1
    rRevolutions = 1.1



def setSpeedsvw(v, w):
    leftSpeed1 = (v + (w*3.95))
    rightSpeed1 = (v - (w*3.95))
    setSpeedsIPS(-leftSpeed1, -rightSpeed1)

def stop():
    pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
    time.sleep(1)

#Check if specific flags are set to decide action.
def whatToDo(leftWallOpen, frontWallOpen, rightWallOpen):
    option = 0

    if inchesDFront < 18:
        adjustFront()

    if leftWallOpen and frontWallOpen and rightWallOpen:
        choices = [1, 2, 3]
        option = random.choice(choices)
        #stop()

    elif frontWallOpen and rightWallOpen:
        choices = [2, 3]
        option = random.choice(choices)
        #stop()

    elif leftWallOpen and rightWallOpen:
        choices = [1, 3]
        option = random.choice(choices)
        #stop()

    elif leftWallOpen and frontWallOpen:
        choices = [1, 2]
        option = random.choice(choices)
        #stop()

    elif rightWallOpen:
        option = 3
        #stop()

    elif frontWallOpen:
        option = 2
        #stop()

    elif leftWallOpen:
        option = 1
        #stop()

    else:
        option = 0
        #stop()

    #CHECK WHAT MOVE I MUST DO!
    if option == 1:
        #turn left
        print("LEFT TURN")
        inPlaceLeftTurn()
    elif option == 2:
        #keep moving forward
        print("MOVE FORWARD")
    elif option == 3:
        #right turn
        print("RIGHT TURN")
        inPlaceRightTurn()
    else:
        #360 turn and move forward
        print("WALL IN ALL THREE PLACES")
        inPlaceTurnAround()

def moveForward():
    global sensorCount

	# Reading in from sensors
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    lDistance = lSensor.get_distance()

    # Transforming readings to inches
    inchesDistanceFront = fDistance * 0.0393700787
    inchesDistanceRight = rDistance * 0.0393700787
    inchesDistanceLeft = lDistance * 0.0393700787

    # Calculating respective errors
    errorf = 7.0 - inchesDistanceFront
    errorr = 8.0 - inchesDistanceRight
    errorl = 8.0 - inchesDistanceLeft

    # Computing the control signals
    controlSignalf = kpValue * errorf
    controlSignalr = kpValue * errorr
    controlSignall = kpValue * errorl

    # Running control signals through saturation functions
    newSignalf = saturationFunction(controlSignalf)
    newSignalr = saturationFunctionRight(controlSignalr)
    newSignall = saturationFunctionRight(controlSignall)


    if inchesDistanceRight > inchesDistanceLeft and inchesDistanceRight < 12.0:
        # Setting speed of the robot, angular speed will be zero when moving straight
        #setSpeedsvw(linearSpeed,-newSignalr)
        setSpeedsvw(linearSpeed,newSignall/3)
    elif inchesDistanceRight > inchesDistanceLeft and inchesDistanceRight > 12.0:
        # Setting speed of the robot, angular speed will be zero when moving straight
        #setSpeedsvw(linearSpeed,-newSignalr)
        setSpeedsvw(linearSpeed,newSignall/3)
    elif inchesDistanceRight < inchesDistanceLeft and inchesDistanceLeft < 12.0:
        setSpeedsvw(linearSpeed,-newSignalr/3)
        #setSpeedsvw(linearSpeed,newSignall)
    elif inchesDistanceRight < inchesDistanceLeft and inchesDistanceLeft > 12.0:
        setSpeedsvw(linearSpeed,-newSignalr/3)
        #setSpeedsvw(linearSpeed,newSignall)
    else:
        setSpeedsvw(linearSpeed,0)

    # Checking if there is an object approaching from the front
    if inchesDistanceFront < 5.0:
        # Increasing reading count
	    sensorCount += 1

        # Checking if the front small reading happens continously to avoid a fake trigger
	    if sensorCount > 4:
            # Turning left
		    adjustFront()

    # Clearing sensor count for continous small front readings
    else:
        sensorCount = 0

def inPlaceLeftTurn():
    global distanceTravel, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.46 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.42 / 20 * 4096))
    time.sleep(0.9)
    lRevolutions = 1.1
    rRevolutions = 1.1

    # if direction == 'W':
    #     direction = 'S'
    # elif direction == 'S':
    #     direction = 'E'
    # elif direction == 'E':
    #     direction = 'N'
    # elif direction == 'N':
    #     direction = 'W'

def inPlaceRightTurn():
    global distanceTravel, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.58 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.54 / 20 * 4096))
    time.sleep(0.9)
    lRevolutions = 1.1
    rRevolutions = 1.1

    # if direction == 'W':
    #     direction = 'N'
    # elif direction == 'N':
    #     direction = 'E'
    # elif direction == 'E':
    #     direction = 'S'
    # elif direction == 'S':
    #     direction = 'W'

def inPlaceTurnAround():
    global distanceTravel, lRevolutions, rRevolutions, direction
    stop()
    pwm.set_pwm(LSERVO, 0, math.floor(1.55 / 20 * 4096))
    pwm.set_pwm(RSERVO, 0, math.floor(1.55 / 20 * 4096))
    time.sleep(2.05)
    lRevolutions = 1.1
    rRevolutions = 1.1

    # if direction == 'W':
    #     direction = 'E'
    # elif direction == 'E':
    #     direction = 'W'
    # elif direction == 'N':
    #     direction = 'S'
    # elif direction == 'S':
    #     direction = 'N'

##############################################MAZEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE###########################################################################################################
# class Cell:
# 	def __init__(self, west, north, east, south, visited = False):
# 		# There are 4 walls per cell
# 		# Wall values can be 'W', 'O', or '?' (wall, open, or unknown)
# 		self.west = west
# 		self.north = north
# 		self.east = east
# 		self.south = south
#
# 		# Store whether or not the cell has been visited before
# 		self.visited = visited
#
# # Helper function that verifies all the walls of the maze
# def detectMazeInconsistencies(maze):
# 	# Check horizontal walls
# 	for i in range(3):
# 		for j in range(4):
# 			pos1 = i * 4 + j
# 			pos2 = i * 4 + j + 4
# 			hWall1 = maze[pos1].south
# 			hWall2 = maze[pos2].north
# 			assert hWall1 == hWall2, " Cell " + str(pos1) + "'s south wall doesn't equal cell " + str(pos2) + "'s north wall! ('" + str(hWall1) + "' != '" + str(hWall2) + "')"
#
# 	# Check vertical walls
# 	for i in range(4):
# 		for j in range(3):
# 			pos1 = i * 4 + j
# 			pos2 = i * 4 + j + 1
# 			vWall1 = maze[pos1].east
# 			vWall2 = maze[pos2].west
# 			assert vWall1 == vWall2, " Cell " + str(pos1) + "'s east wall doesn't equal cell " + str(pos2) + "'s west wall! ('" + str(vWall1) + "' != '" + str(vWall2) + "')"
#
# # You don't have to understand how this function works
# def printMaze(maze, hRes = 4, vRes = 2):
# 	assert hRes > 0, "Invalid horizontal resolution"
# 	assert vRes > 0, "Invalid vertical resolution"
#
# 	# Get the dimensions of the maze drawing
# 	hChars = 4 * (hRes + 1) + 2
# 	vChars = 4 * (vRes + 1) + 1
#
# 	# Store drawing into a list
# 	output = [" "] * (hChars * vChars - 1)
#
# 	# Draw top border
# 	for i in range(1, hChars - 2):
# 		output[i] = "_"
#
# 	# Draw bottom border
# 	for i in range(hChars * (vChars - 1) + 1, hChars * (vChars - 1) + hChars - 2):
# 		output[i] = "¯"
#
# 	# Draw left border
# 	for i in range(hChars, hChars * (vChars - 1), hChars):
# 		output[i] = "|"
#
# 	# Draw right border
# 	for i in range(2 * hChars - 2, hChars * (vChars - 1), hChars):
# 		output[i] = "|"
#
# 	# Draw newline characters
# 	for i in range(hChars - 1, hChars * vChars - 1, hChars):
# 		output[i] = "\n"
#
# 	# Draw dots inside maze
# 	for i in range((vRes + 1) * hChars, hChars * (vChars - 1), (vRes + 1) * hChars):
# 		for j in range(hRes + 1, hChars - 2, hRes + 1):
# 			output[i + j] = "·"
#
# 	# Draw question marks if cell is unvisited
# 	for i in range(4):
# 		for j in range(4):
# 			cellNum = i * 4 + j
# 			if maze[cellNum].visited:
# 				continue
# 			origin = (i * hChars * (vRes + 1) + hChars + 1) + (j * (hRes + 1))
# 			for k in range(vRes):
# 				for l in range(hRes):
# 					output[origin + k * hChars + l] = "?"
#
# 	# Draw horizontal walls
# 	for i in range(3):
# 		for j in range(4):
# 			cellNum = i * 4 + j
# 			origin = ((i + 1) * hChars * (vRes + 1) + 1) + (j * (hRes + 1))
# 			hWall = maze[cellNum].south
# 			for k in range(hRes):
# 				output[origin + k] = "-" if hWall == 'W' else " " if hWall == 'O' else "?"
#
# 	# Draw vertical walls
# 	for i in range(4):
# 		for j in range(3):
# 			cellNum = i * 4 + j
# 			origin = hChars + (hRes + 1) * (j + 1) + i * hChars * (vRes + 1)
# 			vWall = maze[cellNum].east
# 			for k in range(vRes):
# 				output[origin + k * hChars] = "|" if vWall == 'W' else " " if vWall == 'O' else "?"
#
# 	# Print drawing
# 	print(''.join(output))
#
#
# #########################################################################################################################################################
#
# #Function that is called to check the sensors and then update the maze walls based on current cell position.
# def updateMaze(left, front, right, direction, currentCell):
#     global maze
#
#     currentCellIndex = currentCell - 1
#     maze[currentCellIndex].visited = True
#
#     leftIndex = -1
#     upIndex = -1
#     rightIndex = -1
#     downIndex = -1
#
#     if currentCellIndex % 4 > 0:
#         leftIndex = currentCellIndex - 1
#
#     if (currentCellIndex + 1) % 4 > 0:
#         rightIndex = currentCellIndex + 1
#
#     if currentCellIndex >= 4:
#         upIndex = currentCellIndex - 4
#
#     if currentCellIndex < 12:
#         downIndex = currentCellIndex + 4
#
#
# 	    ###########################WEST ORIENTATION#########################################
#     if direction == 'W':
#         if left == True:
#             maze[currentCellIndex].south = 'O'
#         else:
#             maze[currentCellIndex].south = 'W'
#
#         if front == True:
#             maze[currentCellIndex].west = 'O'
#         else:
#             maze[currentCellIndex].west = 'W'
#
#         if right == True:
#             maze[currentCellIndex].north = 'O'
#         else:
#             maze[currentCellIndex].north = 'W'
#
#         maze[currentCellIndex].east = 'O'
#
#
#         #if leftIndex >= 0:
#             #maze[leftIndex].east = maze[currentCellIndex].west
#
#         #if upIndex >= 0:
#             #maze[upIndex].south = maze[currentCellIndex].north
#
#
# 		###########################NORTH ORIENTATION#########################################
#     elif direction == 'N':
#         if left == True:
#             maze[currentCellIndex].west = 'O'
#         else:
#             maze[currentCellIndex].west = 'W'
#
#         if front == True:
#             maze[currentCellIndex].north = 'O'
#         else:
#             maze[currentCellIndex].north = 'W'
#
#         if right == True:
#             maze[currentCellIndex].east = 'O'
#         else:
#             maze[currentCellIndex].east = 'W'
#
#         maze[currentCellIndex].south = 'O'
#
#
#
#         #if leftIndex >= 0:
#             #maze[leftIndex].east = maze[currentCellIndex].west
#
#         #if upIndex >= 0:
#             #maze[upIndex].south = maze[currentCellIndex].north
#
#
# 		###########################EAST ORIENTATION#########################################
#     elif direction == 'E':
#         if left == True:
#             maze[currentCellIndex].north = 'O'
#         else:
#             maze[currentCellIndex].north = 'W'
#
#         if front == True:
#             maze[currentCellIndex].east = 'O'
#         else:
#             maze[currentCellIndex].east = 'W'
#
#         if right == True:
#             maze[currentCellIndex].south = 'O'
#         else:
#             maze[currentCellIndex].south = 'W'
#
#         maze[currentCellIndex].west = 'O'
#
#
#
#         #if leftIndex >= 0:
#             #maze[leftIndex].east = maze[currentCellIndex].south
#
#         #if upIndex >= 0:
#             #maze[upIndex].south = maze[currentCellIndex].west
#
# 			###########################SOUTH ORIENTATION#########################################
#     elif direction == 'S':
#         if left == True:
#             maze[currentCellIndex].east = 'O'
#         else:
#             maze[currentCellIndex].east = 'W'
#
#         if front == True:
#             maze[currentCellIndex].south = 'O'
#         else:
#             maze[currentCellIndex].south = 'W'
#
#         if right == True:
#             maze[currentCellIndex].west = 'O'
#         else:
#             maze[currentCellIndex].west = 'W'
#
#         maze[currentCellIndex].north = 'O'
#
#
#
#
#         #if leftIndex >= 0:
#             #maze[leftIndex].east = maze[currentCellIndex].east
#
#         #if upIndex >= 0:
#             #maze[upIndex].south = maze[currentCellIndex].south
#
#     if leftIndex >= 0:
#         maze[leftIndex].east = maze[currentCellIndex].west
#
#     if upIndex >= 0:
#         maze[upIndex].south = maze[currentCellIndex].north
#
# def updateCell(direction, cell):
#     global currentCell
#
#     if direction == 'W':
#         currentCell = cell - 1
#     elif direction == 'N':
#         currentCell = cell - 4
#     elif direction == 'E':
#         currentCell = cell + 1
#     elif direction == 'S':
# 	    currentCell = cell + 4
#     maze[currentCell - 1].visited = True


## Attach the Ctrl+C signal interrupt
signal.signal(signal.SIGINT, ctrlC)

# Initializing encoders
initEncoders()

# Declaring the disared distance to the wall
desiredDistance = 5.0

# Declaring variable to keep track of distance traveled
distanceTravel = 0

# Declaring the kp value to be used
kpValue = 0.7

# Sleeping the motors before starting the movement
pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
time.sleep(0.5)


# direction = input("Please enter the direction(orientation of the robot starting: ")
# currentCell = int(input("Please enter starting cell numbered from 1-16: "))


# Waiting for user to enter the required key in order to start the movement
flagStart = False
startInput = input("Press 'm' to start robot wall following.")
if startInput == "m":
	flagStart = True
else:
	print("Exiting program, re-run file wall following.")
	exit()

# Declaring constant linear speed that will be used during the movement
linearSpeed = 5

# Declaring a variable to keep track of front sensor big readings
sensorCount = 0

#Booleans to determine if walls are open?
frontWallOpen = False
leftWallOpen = False
rightWallOpen = False
newCell = True

# Initialize the maze with a set of walls and visited cells
# The bottom right cell is marked as unvisited and with unknown walls
# maze = [
# 	Cell('W','W','?','?', False), Cell('?','W','?','?', False), Cell('?','W','?','?', False), Cell('?','W','W','?', False),
#     Cell('W','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','W','?', False),
# 	Cell('W','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','?','?', False), Cell('?','?','W','?', False),
# 	Cell('W','?','?','W', False), Cell('?','?','?','W', False), Cell('?','?','?','W', False), Cell('?','?','W','W', False)
# ]




####################################### UPDATING INFORMATION FROM INITIAL CELL ########################################################


# Reading in from sensors
fDistance = fSensor.get_distance()
rDistance = rSensor.get_distance()
lDistance = lSensor.get_distance()

# Transforming readings to inches
inchesDFront = fDistance * 0.0393700787
inchesDRight = rDistance * 0.0393700787
inchesDLeft = lDistance * 0.0393700787

#Constantly updates the flags while reading the sensors.
if inchesDFront > 15:
    frontWallOpen = True

if inchesDRight > 15:
    rightWallOpen = True

if inchesDLeft > 15:
    leftWallOpen = True

# updateMaze(leftWallOpen, frontWallOpen, rightWallOpen, direction, currentCell)

#######################################################################################################################################

while True:
    #detectMazeInconsistencies(maze)
    #printMaze(maze)
    #updateMaze(True,True,False,'W', 6)

	# Reading in from sensors
    fDistance = fSensor.get_distance()
    rDistance = rSensor.get_distance()
    lDistance = lSensor.get_distance()

    # Transforming readings to inches
    inchesDFront = fDistance * 0.0393700787
    inchesDRight = rDistance * 0.0393700787
    inchesDLeft = lDistance * 0.0393700787

    #Constantly updates the flags while reading the sensors.
    if inchesDFront > 15:
        frontWallOpen = True

    if inchesDRight > 15:
        rightWallOpen = True

    if inchesDLeft > 15:
        leftWallOpen = True

    #Checking if the robot has already traveled the required distance
    distanceTravel = (8.20 * ((lRevolutions + rRevolutions) / 2))
    #print("Distance Traveled is: ", distanceTravel)
    #print(newCell)
    if distanceTravel > 9 and newCell:
        print("I have entered a new cell")
        newCell = False
        # updateCell(direction, currentCell)

    if distanceTravel > 18:
        #print("UPDATING CELL: ", currentCell)
        #print("\nCURRENT ORIENTATION: ", direction)
        # updateMaze(leftWallOpen, frontWallOpen, rightWallOpen, direction, currentCell)
        #print("I updated the maze!")
        # printMaze(maze)
        whatToDo(leftWallOpen, frontWallOpen, rightWallOpen)
        newCell = True

        distanceTravel = 0
        #stop()
        resetCounts()
        #print("TRAVELLED .75 FEET (9INCHES)")

    frontWallOpen = False
    leftWallOpen = False
    rightWallOpen = False

    moveForward()
