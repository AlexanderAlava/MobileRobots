all: camera

camera: camera.cpp
	g++ -o camera camera.cpp `pkg-config --cflags --libs opencv`

clean:
	rm -f *.o camera
