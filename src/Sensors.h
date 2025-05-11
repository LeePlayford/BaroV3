//-------------------------------------
// Sensors
//-------------------------------------

#ifndef SENSORS_H
#define SENSORS_H

#include <Adafruit_Sensor.h>

static bool bBaroSensorValid = false;

#ifdef BMP
#include <Adafruit_BMP280.h>
static Adafruit_BMP280 bmp; // I2C
#elif BME
#include <Adafruit_BME280.h>
static Adafruit_BME280 bme; // I2C
#endif

// baro filter
#define FILTER_SIZE 8
static uint16_t m_baroFilter[FILTER_SIZE]= {0};


//
// Function Prototypes
int16_t ReadPressure();
bool InitSensors ();
uint16_t FilterBaro (uint16_t baro);


#endif