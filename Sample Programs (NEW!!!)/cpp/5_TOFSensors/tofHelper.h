#pragma once
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

void getMeasurement   (VL53L0X_Dev_t *  device, VL53L0X_RangingMeasurementData_t * measurementData);
void connectSensors   (VL53L0X_Dev_t * lDevice, VL53L0X_Dev_t * fDevice, VL53L0X_Dev_t * rDevice);
void disconnectSensors(VL53L0X_Dev_t * lDevice, VL53L0X_Dev_t * fDevice, VL53L0X_Dev_t * rDevice);
