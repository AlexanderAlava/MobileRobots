# This program demonstrates advanced usage of the OpenCV library by
# using the SimpleBlobDetector feature along with camera threading.
# The program displays two windows: one for adjusting the mask,
# and one that displays the detected blobs in the (masked) image.
# Adjust the HSV values until blobs are detected from the camera feed.
# There's also a params file in the same folder that can be adjusted.

# Helpful links:
# https://www.learnopencv.com/blob-detection-using-opencv-python-c/
# https://docs.opencv.org/3.4.1/da/d97/tutorial_threshold_inRange.html
# https://docs.opencv.org/3.4.1/d0/d7a/classcv_1_1SimpleBlobDetector.html
# https://docs.opencv.org/3.4.1/d2/d29/classcv_1_1KeyPoint.html
# https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/

import cv2 as cv
import time
from ThreadedWebcam import ThreadedWebcam
from UnthreadedWebcam import UnthreadedWebcam

FPS_SMOOTHING = 0.9
x_position = 0
y_position = 0
circle_diameter = 0
keypoint_angle = 0.0
noBlob = True

# Window names
WINDOW1 = "Adjustable Mask - Press Esc to quit"
WINDOW2 = "Detected Blobs - Press Esc to quit"

# Default HSV ranges
# Note: the range for hue is 0-180, not 0-255
minH =   120; minS = 100; minV =   100;
maxH = 180; maxS = 255; maxV = 255;


# These functions are called when the user moves a trackbar
def onMinHTrackbar(val):
    # Calculate a valid minimum red value and re-set the trackbar.
    global minH
    global maxH
    minH = min(val, maxH - 1)
    cv.setTrackbarPos("Min Hue", WINDOW1, minH)

def onMinSTrackbar(val):
    global minS
    global maxS
    minS = min(val, maxS - 1)
    cv.setTrackbarPos("Min Sat", WINDOW1, minS)

def onMinVTrackbar(val):
    global minV
    global maxV
    minV = min(val, maxV - 1)
    cv.setTrackbarPos("Min Val", WINDOW1, minV)

def onMaxHTrackbar(val):
    global minH
    global maxH
    maxH = max(val, minH + 1)
    cv.setTrackbarPos("Max Hue", WINDOW1, maxH)

def onMaxSTrackbar(val):
    global minS
    global maxS
    maxS = max(val, minS + 1)
    cv.setTrackbarPos("Max Sat", WINDOW1, maxS)

def onMaxVTrackbar(val):
    global minV
    global maxV
    maxV = max(val, minV + 1)
    cv.setTrackbarPos("Max Val", WINDOW1, maxV)


# Initialize the threaded camera
# You can run the unthreaded camera instead by changing the line below.
# Look for any differences in frame rate and latency.
camera = ThreadedWebcam() # UnthreadedWebcam()
#camera = UnthreadedWebcam()
camera.start()

# Initialize the SimpleBlobDetector
params = cv.SimpleBlobDetector_Params()
detector = cv.SimpleBlobDetector_create(params)

# Attempt to open a SimpleBlobDetector parameters file if it exists,
# Otherwise, one will be generated.
# These values WILL need to be adjusted for accurate and fast blob detection.
fs = cv.FileStorage("params.yaml", cv.FILE_STORAGE_READ); #yaml, xml, or json
if fs.isOpened():
    detector.read(fs.root())
else:
    print("WARNING: params file not found! Creating default file.")

    fs2 = cv.FileStorage("params.yaml", cv.FILE_STORAGE_WRITE)
    detector.write(fs2)
    fs2.release()

fs.release()

# Create windows
cv.namedWindow(WINDOW1)
cv.namedWindow(WINDOW2)

# Create trackbars
cv.createTrackbar("Min Hue", WINDOW1, minH, 180, onMinHTrackbar)
cv.createTrackbar("Max Hue", WINDOW1, maxH, 180, onMaxHTrackbar)
cv.createTrackbar("Min Sat", WINDOW1, minS, 255, onMinSTrackbar)
cv.createTrackbar("Max Sat", WINDOW1, maxS, 255, onMaxSTrackbar)
cv.createTrackbar("Min Val", WINDOW1, minV, 255, onMinVTrackbar)
cv.createTrackbar("Max Val", WINDOW1, maxV, 255, onMaxVTrackbar)

fps, prev = 0.0, 0.0
####################################################################################
import time
import Adafruit_PCA9685
import sys
sys.path.append('/home/pi/VL53L0X_rasp_python/python')
import VL53L0X
import RPi.GPIO as GPIO
import signal
import math

# Declaring and defining the left and right servos maps constructed with data generated from calibrateSpeeds(
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
def saturationFunctionGoalFace(ips):
    controlSignal = ips
    #if controlSignal > 1.0:
        #controlSignal = 1.0
    #elif controlSignal < -1.0:
        #controlSignal = -1.0
    #return controlSignal
    if controlSignal > 1.0:
        controlSignal = 1.0
    elif controlSignal < -1.0:
        controlSignal = -1.0
    return controlSignal

# Function to set appropiate boundaries for front sensor
def saturationFunction(ips):
    controlSignal = ips
    if controlSignal > 7.1:
        controlSignal = 7.1
    elif controlSignal < -7.1:
        controlSignal = -7.1
    return controlSignal

def setSpeedsvw(v, w):
    leftSpeed1 = (v + (w*3.95))
    rightSpeed1 = (v - (w*3.95))
    setSpeedsIPS(-leftSpeed1, -rightSpeed1)

# Function that translates speeds from ips to pwm
def spinOnSelfIPS(ipsLeft, ipsRight):
    # Converting inches per second into revolutions per second
    rpsLeft = float(math.ceil((ipsLeft / 8.20) * 100) / 100)
    rpsRight = float(math.ceil((ipsRight / 8.20) * 100) / 100)

    if rpsLeft < 0:
        rpsLeft = 0 - rpsLeft
    if rpsRight < 0:
        rpsRight = 0 - rpsRight

    # Calculating pwm values from the respective dictionaries
    lPwmValue = float(lPwmTranslation[rpsLeft])
    rPwmValue = float(rPwmTranslation[rpsRight])

    if ipsLeft < 0 or ipsRight < 0:
        # Setting appropiate speeds to the servos when going forwards
        pwm.set_pwm(LSERVO, 0, math.floor(lPwmValue / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(rPwmValue / 20 * 4096))
    elif ipsLeft >= 0 or ipsRight >= 0:
        # Setting apporpiate speeds to the servos when going backwards
        pwm.set_pwm(LSERVO, 0, math.floor(servoFlip(lPwmValue) / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(servoFlip(rPwmValue) / 20 * 4096))

#When in front of object move forward
def moveToGoal():
    ### Calculating respective error
    errorc = 320 - x_position

    ### Computing the control signal
    controlSignalc = kpValue * errorc

    ### Running control signals through saturation function
    newSignalc = saturationFunctionGoalFace(controlSignalc)

    # Reading in from sensors
    fDistance = fSensor.get_distance()

    # Transforming readings to inches
    inchesDistanceFront = fDistance * 0.0393700787

    # Calculating respective errors
    errorf = 5.0 - inchesDistanceFront

    # Computing the control signals
    controlSignalf = kpValue * errorf

    # Running control signals through saturation functions
    newSignalf = saturationFunction(controlSignalf)

    # Setting speed of the robot, angular speed will be zero when moving straight
    setSpeedsvw(linearSpeed,-newSignalc)


def spinForGoal():
    if len(keypoints) >= 1:
        ### Calculating respective error
        error = 320 - x_position

        ### Computing the control signal
        controlSignal = kpValue * error

        ### Running control signals through saturation function
        newSignal = saturationFunctionGoalFace(controlSignal)

        ### Setting speed of the robot with the newly computed values
        spinOnSelfIPS(newSignal, newSignal)
    else:
        pwm.set_pwm(LSERVO, 0, math.floor(1.6 / 20 * 4096))
        pwm.set_pwm(RSERVO, 0, math.floor(1.6 / 20 * 4096))

# Declaring the disared distance to the wall
desiredDistance = 5.0

# Declaring the kp value to be used
kpValue = 0.9

pwm.set_pwm(LSERVO, 0, math.floor(1.5 / 20 * 4096))
pwm.set_pwm(RSERVO, 0, math.floor(1.5 / 20 * 4096))
time.sleep(10)

#while True:
    ## Reading in from sensor
    #fDistance = fSensor.get_distance()

    ## Transforming readings to inches
    #inchesDistance = fDistance * 0.0393700787

    ## Calculating respective error
    #error = desiredDistance - inchesDistance

    ## Computing the control signal
    #controlSignal = kpValue * error

    ## Running control signals through saturation function
    #newSignal = saturationFunction(controlSignal)

    ## Setting speed of the robot with the newly computed values
    #setSpeedsIPS(newSignal, newSignal)

## Stop measurement for all sensors
#lSensor.stop_ranging()
#fSensor.stop_ranging()
#rSensor.stop_ranging()
####################################################################################

while True:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    frame = camera.read()

    # Blob detection works better in the HSV color space
    # (than the RGB color space) so the frame is converted to HSV.
    frame_hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)

    # Create a mask using the given HSV range
    mask = cv.inRange(frame_hsv, (minH, minS, minV), (maxH, maxS, maxV))

    # Run the SimpleBlobDetector on the mask.
    # The results are stored in a vector of 'KeyPoint' objects,
    # which describe the location and size of the blobs.
    keypoints = detector.detect(mask)

    # For each detected blob, draw a circle on the frame
    frame_with_keypoints = cv.drawKeypoints(frame, keypoints, None, color = (0, 255, 0), flags = cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # Write text onto the frame
    cv.putText(frame_with_keypoints, "FPS: {:.1f}".format(fps), (5, 15), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    cv.putText(frame_with_keypoints, "{} blobs".format(len(keypoints)), (5, 35), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))

    #Print FPS on screen and number of blobs.
    print("Camera FPS: ", fps)
    print("Number of blobs detected: ", len(keypoints))
    for keypoint in keypoints:
        x_position = keypoint.pt[0]
        y_position = keypoint.pt[1]
        circle_diameter = keypoint.size # diameter of circle
        keypoint_angle = keypoint.angle # angle

        print("x: ", x_position)
        print("y: ", y_position)
        print("size: ", circle_diameter)

    if x_position <= 315 or x_position >= 325:
        spinForGoal()
    else:
        moveToGoal()

    #spinForGoal()
    #pwm.set_pwm(LSERVO, 0, math.floor(1.52 / 20 * 4096))
    #pwm.set_pwm(RSERVO, 0, math.floor(1.52 / 20 * 4096))

    #if len(keypoints) >= 1 and 319.50 <= float(x_position) <= 320.50:
        #moveToGoal()
    #elif len(keypoints) >= 1:
        #### Calculating respective error
        #error = 320 - x_position

        #### Computing the control signal
        #controlSignal = kpValue * error

        #### Running control signals through saturation function
        #newSignal = saturationFunctionGoalFace(controlSignal)

        #### Setting speed of the robot with the newly computed values
        #spinOnSelfIPS(newSignal, newSignal)
    #else:
        #pwm.set_pwm(LSERVO, 0, math.floor(1.52 / 20 * 4096))
        #pwm.set_pwm(RSERVO, 0, math.floor(1.52 / 20 * 4096))

    ### Calculating respective error
    #error = 320 - x_position

    ### Computing the control signal
    #controlSignal = kpValue * error

    ### Running control signals through saturation function
    #newSignal = saturationFunction(controlSignal)

    ### Setting speed of the robot with the newly computed values
    #setSpeedsIPS(newSignal, newSignal)

    ## Display the frame
    ##cv.imshow(WINDOW1, mask)
    ##cv.imshow(WINDOW2, frame_with_keypoints)

    # Check for user input
    c = cv.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        camera.stop()
        break

camera.stop()
## Stop measurement for all sensors
lSensor.stop_ranging()
fSensor.stop_ranging()
rSensor.stop_ranging()
