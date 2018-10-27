// This is a modified version of the ThreadedWebcam class which removes threading.

#include <opencv2/opencv.hpp>
#include "UnthreadedWebcam.h"
#include <string>

using namespace std;
using namespace cv;


UnthreadedWebcam::UnthreadedWebcam(int src)
	: src(src)
{
	// Initialize camera with a specified resolution.
	// It may take some experimenting to find other valid resolutions,
	// as the camera may end up displaying an incorrect image.
	// Alternatively, frames can be resized afterwards using the resize() function.
    capture = new VideoCapture(src);
    capture->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	capture->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	
	if (!capture->isOpened())
	{
		cout << "Failed to open camera!" << endl;
		exit(1);
	}
	else
	{
		cout << "Unthreaded webcam started." << endl;
	}
	
	capture->read(frame);
}

UnthreadedWebcam::~UnthreadedWebcam()
{
	delete capture;
}

void UnthreadedWebcam::Start()
{
	// Do nothing
}

void UnthreadedWebcam::Stop()
{
	// Do nothing
}

// Returns the latest camera frame.
Mat UnthreadedWebcam::Read()
{
	capture->read(frame);
	return frame;
}
