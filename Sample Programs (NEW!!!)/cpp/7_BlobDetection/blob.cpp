// This program demonstrates more advanced usage of the OpenCV library
// by using the SimpleBlobDetector feature along with camera threading.
// The program displays two windows: one for adjusting the mask, 
// and one that displays the detected blobs in the (masked) image.
// Adjust the HSV values until blobs are detected from the camera feed.
// There's also a params file in the same folder that can be adjusted.

// Helpful links:
// https://www.learnopencv.com/blob-detection-using-opencv-python-c/
// https://docs.opencv.org/3.4.1/da/d97/tutorial_threshold_inRange.html
// https://docs.opencv.org/3.4.1/d0/d7a/classcv_1_1SimpleBlobDetector.html
// https://docs.opencv.org/3.4.1/d2/d29/classcv_1_1KeyPoint.html
// https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/

#include <opencv2/opencv.hpp>
#include <stdint.h>
#include <iostream>
#include "ThreadedWebcam.h"
#include "UnthreadedWebcam.h"

#define FPS_SMOOTHING 0.9

using namespace std;
using namespace cv;

// Window names
const string WINDOW1 = "Adjustable Mask - Esc to quit";
const string WINDOW2 = "Detected Blobs - Esc to quit";

// Default HSV ranges
// Note: the range for hue is 0-180, not 0-255
int minH = 0,   minS = 127, minV =   0;
int maxH = 180, maxS = 255, maxV = 255;


// These functions are called when the user moves a trackbar
void onMinHTrackbar(int, void *)
{
	// Calculate a valid minimum red value and re-set the trackbar.
	minH = min(minH, maxH - 1);
	setTrackbarPos("Min H", WINDOW1, minH);
}

void onMinSTrackbar(int, void *)
{
	minS = min(minS, maxS - 1);
	setTrackbarPos("Min S", WINDOW1, minS);
}

void onMinVTrackbar(int, void *)
{
	minV = min(minV, maxV - 1);
	setTrackbarPos("Min V", WINDOW1, minV);
}

void onMaxHTrackbar(int, void *)
{
	maxH = max(maxH, minH + 1);
	setTrackbarPos("Max H", WINDOW1, maxH);
}

void onMaxSTrackbar(int, void *)
{
	maxS = max(maxS, minS + 1);
	setTrackbarPos("Max S", WINDOW1, maxS);
}

void onMaxVTrackbar(int, void *)
{
	maxV = max(maxV, minV + 1);
	setTrackbarPos("Max V", WINDOW1, maxV);
}

int main(int argc, char** argv)
{
	// Initialize the threaded camera
	// You can run the unthreaded camera instead by changing the line below.
	// Look for any differences in frame rate and latency.
	ThreadedWebcam camera(0); // UnthreadedWebcam camera(0);
	camera.Start();
	
	// Initialize the SimpleBlobDetector
    SimpleBlobDetector::Params params;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    
    // Attempt to open a SimpleBlobDetector parameters file if it exists,
    // Otherwise, one will be generated.
    // These values WILL need to be adjusted for accurate and fast blob detection.
    FileStorage fs("params.yaml", FileStorage::READ); //yaml, xml, or json
    if (fs.isOpened())
    {
		detector->read(fs.root());
	}
	else
	{
		cout << "WARNING: params file not found! Creating default file." << endl;
		
		FileStorage fs2("params.yaml", FileStorage::WRITE);
		detector->write(fs2);
		fs2.release();
    }
    fs.release();
    
	// Create windows
    namedWindow(WINDOW1);
    namedWindow(WINDOW2);
    
	// Create trackbars
    createTrackbar("Min H", WINDOW1, &minH, 180, onMinHTrackbar);
    createTrackbar("Max H", WINDOW1, &maxH, 180, onMaxHTrackbar);
    createTrackbar("Min S", WINDOW1, &minS, 255, onMinSTrackbar);
    createTrackbar("Max S", WINDOW1, &maxS, 255, onMaxSTrackbar);
    createTrackbar("Min V", WINDOW1, &minV, 255, onMinVTrackbar);
    createTrackbar("Max V", WINDOW1, &maxV, 255, onMaxVTrackbar);
    
    float fps = 0.0;
    double prev = 0;
    Mat frame, frame_hsv, mask, frame_with_keypoints;
    vector<KeyPoint> keypoints;
    while (true)
    {
		// Calculate FPS
        float now = (clock()/(float)CLOCKS_PER_SEC);
        fps = (fps*FPS_SMOOTHING + (1/(now - prev))*(1.0 - FPS_SMOOTHING));
        prev = now;
        
        // Get a frame
        frame = camera.Read();
        
		// Blob detection works better in the HSV color space 
		// (than the RGB color space) so the frame is converted to HSV.
		cvtColor(frame, frame_hsv, COLOR_BGR2HSV);
		
		// Create a mask using the given HSV range
		inRange(frame_hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
		
		// Run the SimpleBlobDetector on the masked frame.
		// The results are stored in a vector of 'KeyPoint' objects,
		// which describe the location and size of the blobs.
		detector->detect(mask, keypoints);
		
		// For each detected blob, draw a circle on the frame
		drawKeypoints(frame, keypoints, frame_with_keypoints, Scalar(0, 255, 0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		
		// Write text onto the frame
        putText(frame_with_keypoints, "FPS: " + to_string(fps), Point(5, 15), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0));
        putText(frame_with_keypoints, to_string(keypoints.size()) + " blobs", Point(5, 35), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0));

		// Display the frames
        imshow(WINDOW1, mask);
        imshow(WINDOW2, frame_with_keypoints);
        
		// Check for user input
        char c = (char)waitKey(1);
        if (c == 27 | c == 'q' | c == 'Q') // Esc or Q
        {
			camera.Stop();
            break;
		}
    }
}
