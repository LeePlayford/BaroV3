
//-------------------------------------------------
//
// Project BaroGraph
// Author Lee Playford
// Date May 2025    
//
//-------------------------------------------------

#include <Wire.h>
#include <stdio.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// ESP32 Can Setup
#define ESP32_CAN_TX_PIN GPIO_NUM_16
#define ESP32_CAN_RX_PIN GPIO_NUM_17

#include <Arduino.h>

#define USE_N2K_CAN 7
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>

#include "Free_Fonts.h"

#include "eepromManager.h"
#include "lcdScreen.h"
#include "boardTests.h" 
#include "Sensors.h"


//led defines
#define LED_1 GPIO_NUM_4
#define LED_2 GPIO_NUM_33
#define LED_3 GPIO_NUM_32
#define LED_4 GPIO_NUM_25

bool LED_1_State = 0;
bool LED_2_State = 0;
bool LED_3_State = 0;
bool LED_4_State = 0;


// Data Defines
uint32_t SAMPLE_TIME = 86400/4*10;
#define POINTS_PER_DAY 400
#define MAX_DAYS 1
#define BARO_ARRAY_SIZE (POINTS_PER_DAY * MAX_DAYS) // 7 days data

// Baro Array Define
uint16_t m_baroDataArray[BARO_ARRAY_SIZE];
uint16_t m_baroDataHead = 0;

// create the eeprom object
EepromManager eepromManager;

// create the LCD Screen object
LcdScreen lcdScreen;


// Define READ_STREAM to port, where you write data from PC e.g. with NMEA Simulator.
#define READ_STREAM Serial       
// Define ForwardStream to port, what you listen on PC side. On Arduino Due you can use e.g. SerialUSB
#define FORWARD_STREAM Serial    

Stream *ReadStream=&READ_STREAM;
Stream *ForwardStream=&FORWARD_STREAM;

// WIFI network defines
//const char* ssid = "TALKTALK4AD3F0";
//const char* password = "C3AKAC4R";
const char* ssid = "Wireless 2.4G_08C4E8_Sh3d";
const char* password = "M00n5hineSh3d!";
bool wifiConnected = false;
 
// Function prototypes
void GetHighLowRange (uint16_t& high , uint16_t &low , uint16_t &range);
 
//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void setup() 
{
    Serial.begin(115200);
    Serial.println("Booting");
    Serial.print ("Version ");
    Serial.println (version);
    
    // Setup the LEDS
    pinMode (LED_1 , OUTPUT);
    pinMode (LED_2 , OUTPUT);
    pinMode (LED_3 , OUTPUT);
    pinMode (LED_4 , OUTPUT);

    digitalWrite (LED_1 , LOW);
    digitalWrite (LED_2 , LOW);
    digitalWrite (LED_3 , LOW);
    digitalWrite (LED_4 , LOW);

#ifdef OTA
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) 
    {
        Serial.println("Connection Failed! Rebooting...");
        //delay(5000);
        //ESP.restart();
    }
    else
    {
        wifiConnected = true;
    

    
// Over the Air Updates
      ArduinoOTA
        .onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
        })
        .onEnd([]() {
            Serial.println("\nEnd");
        })
        .onProgress([](unsigned int progress, unsigned int total) 
        {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        })
        .onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

      ArduinoOTA.begin();

      Serial.println("Wifi Ready");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      Serial.print("MAC Address: ");
      Serial.println(WiFi.macAddress());

    }

    digitalWrite (LED_4 , HIGH);// wifi started
    digitalWrite (LED_3 , LOW);// wifi started

#endif
    // Start the TFT
    lcdScreen.Init();
    
    // Initialise the Sensors
    InitSensors();
   
    // Start the I2C wire
    Wire.begin();
    //Wire.setClock(400000);
    Wire.setClock(100000);

    // make a unique number from the mac address
    uint8_t mac[6];
    WiFi.macAddress (mac);
    unsigned long uniqueId = (mac[3] << 16) | (mac[4] <<  8) | mac[5];

    // setup pwm
    // channel , freq , resolution
    //ledcSetup (0,5000,8);

    // Can Bus Set up
    // Reserve enough buffer for sending all messages. 
    NMEA2000.SetN2kCANSendFrameBufSize(250);
    // Set Product information
    NMEA2000.SetProductInformation("00000001", // Manufacturer's Model serial code
                                 100, // Manufacturer's product code
                                 "Barograph ESP32",  // Manufacturer's Model ID
                                 "1.0.0.0 (2020-08-15)",  // Manufacturer's Software version code
                                 "1.0.2.0 (2019-08-15)" // Manufacturer's Model version
                                 );
    // Set device information
    NMEA2000.SetDeviceInformation(uniqueId, // Unique number. Use e.g. Serial number.
                                132, // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                25, // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046 // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf                               
                               );
    
    // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
    NMEA2000.SetForwardStream(&Serial);
    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly , 23);
    NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
    NMEA2000.SendProductInformation (0xff);
    NMEA2000.Open();
   
}

//---------------------------------------------------------------------
// Send the Baro over the N2K bus
//---------------------------------------------------------------------
void SendPressure (uint16_t baro)
{
    static bool bSentBaro = false;
    if (bSentBaro == false)
    {
        Serial.println("Sending Baro");
        bSentBaro = true;
    }
    tN2kMsg N2kMsg;
    SetN2kPressure(N2kMsg,0,2,N2kps_Atmospheric,baro*10);
    NMEA2000.SendMsg(N2kMsg);
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void GetHighLowRange (uint16_t& high , uint16_t &low , uint16_t &range)
{
    // loop the data and find the high and low
    range = 0;    
    
    for (uint16_t i = 0 ; i < BARO_ARRAY_SIZE ; i++)
    {
        uint16_t value = m_baroDataArray[i];
        if (value > MIN_BARO && value < MAX_BARO)
        {
            if (value < low ) 
                low = value;
            else if (value > high) 
                high = value;
        }        
    }   
    range = high - low;
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
uint16_t GetRange (uint16_t range)
{
    if (range < 10) range = 10;
    else if (range < 20) range = 20;
    else if (range < 50) range = 50;
    else if (range < 100) range = 100;
    else if (range < 200) range = 200;
    else if (range < 300) range = 300;
    else if (range < 400) range = 400;
    else if (range < 500) range = 500;
    else range =  1000; 
    return range;
    
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void ScaleHighLowRange (uint16_t& high , uint16_t &low , uint16_t &range)
{
    range = GetRange (range);
    high = uint16_t((high / 10)+1)*10;

    if (low < high - range)
    {
        range = GetRange (range + 10);
    }
    low = high - range;
}

//----------------------------------------
//
//----------------------------------------
/*void PWMSetup (int led ,int channel, int duty)
{
    ledcSetup (channel , 5000 , 8);
    ledcAttachPin (led , channel);
    ledcWrite (channel , duty);
}*/


//-------------------------------------
//
//-------------------------------------
void UpdateBaro(int16_t baro )
{
   // get the high low and range
   uint16_t high , low , range;
   static uint16_t lastHigh = 0 , lastLow = 0, lastRange = 0;
   high = baro;
   low = baro;
   GetHighLowRange (high , low , range);
   ScaleHighLowRange(high, low, range);

   // Draw the Baro Scale
   if (lastHigh != high || lastLow != low || lastRange != range)
   {
       lastHigh = high;
       lastLow = low;
       lastRange = range;
       lcdScreen.AddScale (low , range / 5);
   }

   int16_t offset = m_baroDataHead - BARO_ARRAY_SIZE + 1;
   if (offset < 0) 
       offset += BARO_ARRAY_SIZE;

    lcdScreen.DrawBaro (m_baroDataArray,  BARO_ARRAY_SIZE, offset , high , low , range);
}

//----------------------------------------
//
//----------------------------------------
void loop()
{
    lcdScreen.SplashScreen(wifiConnected , WiFi.localIP() , WiFi.macAddress());

    #ifdef TEST_EEPROM
    RunBoardTests(lcdScreen , eepromManager);
#endif

    // Blink the LEDS to show we are alive
    LED_1_State = HIGH;
    LED_2_State = LOW;
    
    for (int i = 0 ; i < 6 ; i++)
    {
        digitalWrite (LED_1 , LED_1_State);
        digitalWrite (LED_2 , LED_2_State);
        LED_1_State = !LED_1_State;
        LED_2_State = !LED_2_State;
        delay (500);
    }
    digitalWrite (LED_1 , LOW);
    digitalWrite (LED_2 , LOW);
    digitalWrite (LED_3 , HIGH);
    digitalWrite (LED_4 , LOW);

     // draw the screen
    lcdScreen.DrawInitScreen ();

    // Read the eeprom
    eepromManager.ReadData(m_baroDataArray , BARO_ARRAY_SIZE , m_baroDataHead);
    // Local variables

    uint32_t lastReadTime = 0;
    uint32_t lastUpdateTime = 0;
    uint32_t lastSendTime = 0;
    int16_t lastPressure = 0;

    int16_t int_pressure = ReadPressure();
 
    int16_t counter = 0;
    
    while (true)
    {
        if (lastSendTime == 0 || millis() > lastSendTime + 1000)
        {
            SendPressure(int_pressure);
            lastSendTime = millis();
            
            //digitalWrite (LED_3 , LED_1_State);
            //LED_1_State = !LED_1_State; save current
            
        }
        if (lastReadTime == 0 || millis() - lastReadTime > SAMPLE_TIME / 8)
        {
            lastReadTime = millis();
            int_pressure = (int16_t)(ReadPressure());
        }        
      
        if (lastUpdateTime == 0 || millis() - lastUpdateTime > SAMPLE_TIME )
        {
            lastUpdateTime = millis();
            

            // Update the value only if its changed
            if (lastPressure != int_pressure)
            {
                lastPressure = int_pressure;
                uint16_t high = int_pressure;
                uint16_t low = int_pressure;
                uint16_t range = 0;;
                GetHighLowRange(high , low , range);
                lcdScreen.UpdateBaro (int_pressure , high , low , range);
                
                // work out the three hours ago value
                int16_t offset = m_baroDataHead - POINTS_PER_DAY / 8;
                if (offset < 0) offset += POINTS_PER_DAY; 
                
                int16_t threeHours = m_baroDataArray[offset] ;
                if (threeHours < MIN_BARO || threeHours > MAX_BARO)
                    threeHours = int_pressure;
            
                lcdScreen.UpdateTrend (int_pressure , threeHours);
            }

            // update the graph
            // Add the new value
            m_baroDataArray[m_baroDataHead] = int_pressure;
            
            UpdateBaro (int_pressure);

            // reset the array head if necessary
            if (++m_baroDataHead >= BARO_ARRAY_SIZE)
                m_baroDataHead = 0;

            // Store the data every 20 minutes
            if (++counter%20 == 0)  // store every hour
                eepromManager.StoreData(m_baroDataArray , BARO_ARRAY_SIZE , m_baroDataHead);
        }      
        
        delay(10);
#ifdef OTA
        if (wifiConnected)
          ArduinoOTA.handle();
#endif
        NMEA2000.ParseMessages();

    }
}
