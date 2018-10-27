// This class is a wrapper for the camera which adds threading functionality.
// See https://www.pyimagesearch.com/2015/12/21/increasing-webcam-fps-with-python-and-opencv/ for more details.

#include <opencv2/opencv.hpp>
#include "ThreadedWebcam.h"
#include <string>
#include <thread>
#include <iostream>

using namespace std;
using namespace cv;


// Private function that constantly reads the camera stream.
// Do not call this function externally.
void Update(ThreadedWebcam * camera)
{	
	while (!camera->stopped)
	{
		camera->capture->read(camera->frame);
	}
}

ThreadedWebcam::ThreadedWebcam(int src)
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
		cout << "Threaded webcam started." << endl;
	}
	
	capture->read(frame);
}

ThreadedWebcam::~ThreadedWebcam()
{
	if (myThread != NULL)
		Stop();
	
	delete capture;
}

// Starts the camera thread.
void ThreadedWebcam::Start()
{
	if (myThread != NULL)
		return;	
	
	stopped = false;
	myThread = new thread(Update, this);
}

// Stops the camera thread.
void ThreadedWebcam::Stop()
{
	if (myThread == NULL)
		return;
	
	stopped = true;
	myThread->join();
	delete myThread;
	myThread = NULL;
}

// Returns the latest camera frame.
Mat ThreadedWebcam::Read()
{
	return frame;
}
