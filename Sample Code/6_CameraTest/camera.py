# This program demonstrates the usage of the camera through the OpenCV library.
# A simple camera feed is displayed on screen, with the current frames per second.
# See https://www.learnopencv.com/read-write-and-display-a-video-using-opencv-cpp-python/ for more details.

import cv2
import time

FPS_SMOOTHING = 0.9

# Initialize camera with a specified resolution.
# It may take some experimenting to find other valid resolutions,
# as the camera may end up displaying an incorrect image.
# Alternatively, frames can be resized afterwards using the resize() function.
capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not capture.isOpened():
    print("Failed to open camera!")
    exit()

fps, prev = 0.0, 0.0
while True:
    # Calculate FPS
    now = time.time()
    fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING))
    prev = now

    # Get a frame
    ret, frame = capture.read()
    if not ret:
        break

    # Write text onto the frame
    cv2.putText(frame, "FPS: {:.1f}".format(fps), (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0))
    
    # Display the frame
    cv2.imshow("Preview - Press esc to exit", frame)
    
    # Check for user input
    c = cv2.waitKey(1)
    if c == 27 or c == ord('q') or c == ord('Q'): # Esc or Q
        break
