all: encoders

encoders: encoders.cpp
	g++ -Wall -o encoders encoders.cpp -lwiringPi

clean:
	rm -f *.o encoders
