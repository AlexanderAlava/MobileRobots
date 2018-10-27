#pragma once
#include <opencv2/opencv.hpp>
#include <thread>

using namespace cv;
using namespace std;

class ThreadedWebcam
{
public:
	int src;

	ThreadedWebcam(int src = 0);
	~ThreadedWebcam();
	
	void Start();
	Mat Read();
	void Stop();

private:
	VideoCapture * capture;
	thread * myThread = NULL;
	Mat frame;
	bool stopped;
	
	friend void Update(ThreadedWebcam * camera);
};
