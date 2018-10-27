all: blob

blob: blob.cpp ThreadedWebcam.o UnthreadedWebcam.o
	g++ -o blob blob.cpp `pkg-config --cflags --libs opencv` ThreadedWebcam.o UnthreadedWebcam.o -lpthread

ThreadedWebcam.o: ThreadedWebcam.cpp
	g++ -c ThreadedWebcam.cpp `pkg-config --cflags --libs opencv`

UnthreadedWebcam.o: UnthreadedWebcam.cpp
	g++ -c UnthreadedWebcam.cpp `pkg-config --cflags --libs opencv`

clean:
	rm -f *.o blob
