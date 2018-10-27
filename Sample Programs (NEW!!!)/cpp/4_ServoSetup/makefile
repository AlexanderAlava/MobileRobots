all: servos

servos: servos.cpp
	g++ -Wall -o servos servos.cpp -lwiringPi -lwiringPiPca9685

clean:
	rm -f *.o servos
