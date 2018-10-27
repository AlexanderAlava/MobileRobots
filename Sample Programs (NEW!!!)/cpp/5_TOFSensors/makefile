TOF_DIR = /home/pi/VL53L0X_rasp
API_DIR = /home/pi/VL53L0X_rasp/VL53L0X_1.0.2/Api

all: tof

tof: tof.cpp tofHelper.o
	g++ -Wall -o tof tof.cpp tofHelper.o -lwiringPi -I$(API_DIR)/core/inc -I$(API_DIR)/platform/inc -I$(TOF_DIR)/platform/inc -L$(TOF_DIR)/bin -lVL53L0X_Rasp

tofHelper.o: tofHelper.h tofHelper.cpp
	g++ -Wall -c tofHelper.cpp -I$(API_DIR)/core/inc -I$(TOF_DIR)/platform/inc -I$(API_DIR)/platform/inc

clean:
	rm -f *.o tof
