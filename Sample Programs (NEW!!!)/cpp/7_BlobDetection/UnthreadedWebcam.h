#pragma once
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class UnthreadedWebcam
{
public:
	int src;

	UnthreadedWebcam(int src = 0);
	~UnthreadedWebcam();
	
	void Start();
	Mat Read();
	void Stop();

private:
	VideoCapture * capture;
	Mat frame;
};
