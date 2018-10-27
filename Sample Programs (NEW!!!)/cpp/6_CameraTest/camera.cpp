// This program demonstrates the usage of the camera through the OpenCV library.
// A simple camera feed is displayed on screen, with the current frames per second.
// See https://www.learnopencv.com/read-write-and-display-a-video-using-opencv-cpp-python/ for more details.

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <iostream>
#include <iomanip>

using namespace std;
using namespace cv;

#define FPS_SMOOTHING 0.9

int main(int argc, char** argv)
{
	// Initialize camera with a specified resolution.
	// It may take some experimenting to find other valid resolutions,
	// as the camera may end up displaying an incorrect image.
	// Alternatively, frames can be resized afterwards using the resize() function.
    VideoCapture capture(0);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	
	if (!capture.isOpened())
	{
		cout << "Failed to open camera!" << endl;
		return 1;
	}
	
	Mat frame;
    float fps = 0, prev = 0;
    while (true)
    {
		// Calculate FPS
        float now = (clock()/(float)CLOCKS_PER_SEC);
        fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING));
        prev = now;
        
		// Get a frame
        capture >> frame;
        if (frame.empty())
			break;
        
		// Write text onto the frame
        putText(frame, "FPS: " + to_string(fps), Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0));
        
		// Display the frame
        imshow("Preview - Press esc to exit", frame);
        
		// Check for user input
        char c = (char)waitKey(1);
        if (c == 27 | c == 'q' | c == 'Q') // Esc or Q
            break;
    }
}
