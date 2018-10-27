#include "tofHelper.h"
#include <assert.h>
#include <wiringPi.h>
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

// Pins that the sensors are connected to
#define LSHDNPIN 27
#define FSHDNPIN 22
#define RSHDNPIN 23

#define DEFAULTADDR 0x29 // All sensors use this address by default, don't change this
#define LADDR       0x2a
#define RADDR       0x2b
 
void checkStatus(VL53L0X_Error status)
{
	if (status != VL53L0X_ERROR_NONE)
	{
		char buf[VL53L0X_MAX_STRING_LENGTH];
		VL53L0X_GetPalErrorString(status, buf);
		printf("API Status: %i : %s\n", status, buf);
		
		assert(false);
	}    
}

VL53L0X_Error waitForMeasurementDataReady(VL53L0X_Dev_t * device)
{
	VL53L0X_Error status;
	uint8_t isDataReady;
	
	// Wait up to 1 second (200 * 5ms = 1s)
	for (int i = 0; i < 200; i++)
	{
		status = VL53L0X_GetMeasurementDataReady(device, &isDataReady);
		
		if ((isDataReady == 0x01) || status != VL53L0X_ERROR_NONE)
			return status;
			
		VL53L0X_PollingDelay(device); // (sleeps for 5ms)
	}
	
	return VL53L0X_ERROR_TIME_OUT;
}

VL53L0X_Error waitForStopCompleted(VL53L0X_Dev_t * device)
{
	VL53L0X_Error status;
	uint32_t isStopCompleted;
	
	// Wait up to 1 second (200 * 5ms = 1s)
	for (int i = 0; i < 200; i++)
	{
		status = VL53L0X_GetStopCompletedStatus(device, &isStopCompleted);
		
		if ((isStopCompleted == 0x00) || status != VL53L0X_ERROR_NONE)
			return status;
		
		VL53L0X_PollingDelay(device); // (sleeps for 5ms)
	}
	
	return VL53L0X_ERROR_TIME_OUT;
}

void startSensor(VL53L0X_Dev_t * device)
{
	VL53L0X_Error status;
	uint8_t vhvSettings;
    uint8_t phaseCal;
    uint8_t isApertureSpads;
    uint32_t refSpadCount;
    
	status = VL53L0X_DataInit(device);
	checkStatus(status);

	status = VL53L0X_StaticInit(device);
	checkStatus(status);
	
	status = VL53L0X_PerformRefCalibration(device, &vhvSettings, &phaseCal);
	checkStatus(status);

	status = VL53L0X_PerformRefSpadManagement(device, &refSpadCount, &isApertureSpads);
	checkStatus(status);

	status = VL53L0X_SetDeviceMode(device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
	checkStatus(status);

	status = VL53L0X_StartMeasurement(device);
	checkStatus(status);
}

void stopSensor(VL53L0X_Dev_t * device)
{
	VL53L0X_Error status;
	
	status = VL53L0X_StopMeasurement(device);
	checkStatus(status);
	
	status = waitForStopCompleted(device);
	checkStatus(status);
	
	status = VL53L0X_ClearInterruptMask(device, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	checkStatus(status);
}

void getMeasurement(VL53L0X_Dev_t * device, VL53L0X_RangingMeasurementData_t * measurementData)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	
	status = waitForMeasurementDataReady(device);
	checkStatus(status);
	
	status = VL53L0X_GetRangingMeasurementData(device, measurementData);
	checkStatus(status);
	
	status = VL53L0X_ClearInterruptMask(device, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
	checkStatus(status);
}

void connectSensors(VL53L0X_Dev_t * lDevice, VL53L0X_Dev_t * fDevice, VL53L0X_Dev_t * rDevice)
{
	VL53L0X_Error status;
	
	// Setup pins
    pinMode(LSHDNPIN, OUTPUT);
    pinMode(FSHDNPIN, OUTPUT);
    pinMode(RSHDNPIN, OUTPUT);
	
	// Store sensor addresses
    lDevice->I2cDevAddr = LADDR;
	fDevice->I2cDevAddr = DEFAULTADDR;
	rDevice->I2cDevAddr = RADDR;
    
    // Shutdown all sensors
    digitalWrite(LSHDNPIN, LOW);
    digitalWrite(FSHDNPIN, LOW);
    digitalWrite(RSHDNPIN, LOW);
    delay(10);
    
    // Wake up left sensor
	digitalWrite(LSHDNPIN, HIGH);
    delay(10);
	
	// Connect to left sensor
	lDevice->fd = VL53L0X_i2c_init((char*)"/dev/i2c-1", DEFAULTADDR);
	assert(lDevice->fd > 0);
	
	// Reassign left sensor's address
    status = VL53L0X_SetDeviceAddress(lDevice, LADDR * 2);
	checkStatus(status);

	// Reconnect to left sensor
	lDevice->fd = VL53L0X_i2c_init((char*)"/dev/i2c-1", LADDR);
	assert(lDevice->fd > 0);
	
	// Wake up right sensor
	digitalWrite(RSHDNPIN, HIGH);
    delay(10);
    
    // Connect to right sensor
    rDevice->fd = VL53L0X_i2c_init((char*)"/dev/i2c-1", DEFAULTADDR);
	assert(rDevice->fd > 0);
	
	// Reassign right sensor's address
    status = VL53L0X_SetDeviceAddress(rDevice, RADDR * 2);
	checkStatus(status);

	// Reconnect to right sensor
	rDevice->fd = VL53L0X_i2c_init((char*)"/dev/i2c-1", RADDR);
	assert(rDevice->fd > 0);
	
	// Wake up front sensor
	digitalWrite(FSHDNPIN, HIGH);
    delay(10);
    
    // Connect to front sensor
    fDevice->fd = VL53L0X_i2c_init((char*)"/dev/i2c-1", DEFAULTADDR);
	assert(fDevice->fd > 0);
    
    // Initialize sensors
	startSensor(lDevice);
	startSensor(fDevice);
	startSensor(rDevice);
}

void disconnectSensors(VL53L0X_Dev_t * lDevice, VL53L0X_Dev_t * fDevice, VL53L0X_Dev_t * rDevice)
{
	stopSensor(lDevice);
	stopSensor(fDevice);
	stopSensor(rDevice);
}
