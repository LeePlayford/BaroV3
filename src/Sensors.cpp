//-------------------------------------
//
//-------------------------------------
#include "Sensors.h"
#include <Arduino.h>

//-------------------------------------
//
//-------------------------------------
bool InitSensors ()
{
    bool result = false;
    // Init the sensors
#ifdef BMP
    if (!bmp.begin(0x76))
    {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    }
    else
    {
    /* Default settings from datasheet. */
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
        bBaroSensorValid = true;
    }
#elif BME
    if (!bme.begin(0x76))
    {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    }
    else
    {
        bBaroSensorValid = true;
    }
#endif
    
    return bBaroSensorValid;
}

//----------------------------------------
//
//----------------------------------------
int16_t ReadPressure ()
{
    int16_t int_pressure = 0;
    if (bBaroSensorValid)
    {
#ifdef BMP
        int_pressure = (int16_t)(bmp.readPressure() / 10.0);
#elif BME
        int_pressure = (int16_t)(bme.readPressure() / 10.0);
#else
        int_pressure = 10134;
#endif
    }
    return FilterBaro (int_pressure);
}

//----------------------------------------
//
//----------------------------------------
uint16_t FilterBaro (uint16_t baro)
{
    static int head = 0;
    // init the filter
    if (m_baroFilter[0] == 0)
    {
        for (int i = 0 ; i < FILTER_SIZE ; i++)
        {
            m_baroFilter[i] = baro;
        }
        return baro;
    }
    else 
    {
        m_baroFilter[head++] = baro;
        if (head == FILTER_SIZE)
            head = 0;
    }
    uint32_t total = 0; 
    for (int i = 0 ; i < FILTER_SIZE ; i++)
    {
        total += m_baroFilter[i];
    }
    return total / FILTER_SIZE;
}
