//-------------------------------------
//
//-------------------------------------


#include "boardTests.h"
#include "Free_Fonts.h"
#include "Sensors.h"


//-------------------------------------
// Private Function Headers
//-------------------------------------
void getBootReasonMessage(char *buffer, int bufferlength) ;
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);

//-------------------------------------
// Consts and defines
//-------------------------------------
#define TEST_RESULTS_X              132
#define TEST_RESULTS_Y              10
#define TEST_INITIAL_X              10
#define TEST_RESULTS_LINE_HEIGHT    20


//----------------------------------------
// Run Board Tests
//----------------------------------------
bool RunBoardTests(LcdScreen & lcdScreen , EepromManager &eepromManager)
{
    TFT_eSPI &tft = lcdScreen.GetTft();
    int testNumber = 0;
    // Run Tests and place on TFT screen
    tft.setTextColor (TFT_GREEN , cBackground);
    tft.setFreeFont (FSS9);
    tft.drawString ("Running Board Tests" , 10 , 10 ,1);
 
    char buf [10];  // temp buffer

    // Test 1 Print Restart Reason
    testNumber = 1;
    char bootReasonMessage[150]; // Allocate a buffer for the message
    getBootReasonMessage(bootReasonMessage, sizeof(bootReasonMessage)); // Get the message
    tft.drawString (bootReasonMessage , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) , 1);
    
    // Test 2 Check Baro
    testNumber = 2;
    int16_t baro = ReadPressure();
    if (baro > 0)
    {
        tft.drawString ("Baro Pressure" , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) , 1);
        sprintf (buf , "%0.1f" , (float)baro / 10.f);
        tft.setTextColor (TFT_YELLOW , cBackground);
        tft.drawString (buf , TEST_RESULTS_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
        tft.setTextColor (TFT_GREEN , cBackground);
    }
    else
    {
        tft.setTextColor (TFT_RED , cBackground);
        tft.drawString ("Baro Failed" , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
        tft.setTextColor (TFT_GREEN , cBackground);

    }

    // Test 3 Get Input Voltage
    testNumber = 3;
    int inVal = analogRead(33);
    const float vDrop = 0.76f;

    float voltage = mapfloat ((float)inVal , 863.f , 1318.f , 9.24f , 13.24f);
    voltage += vDrop;

    tft.drawString ("Input Voltage" , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
    sprintf (buf , "%2.1fv" , voltage);
    tft.setTextColor (TFT_YELLOW , cBackground);
    tft.drawString (buf , TEST_RESULTS_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
    tft.setTextColor (TFT_GREEN , cBackground);


    // Test 4 Check Memory
    testNumber = 4;
    tft.drawString ("Memory Test " , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
    int location; // used for the error checking
    bool result = eepromManager.TestEeprom(400 , false , location);

    if (result)
    {
        tft.setTextColor (TFT_YELLOW , cBackground);
        tft.drawString (" =EEPROM Test OK= " , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
        tft.setTextColor (TFT_GREEN , cBackground);

    }
    else
    {
        tft.setTextColor(TFT_RED , cBackground);
        tft.drawString (" = EEPROM Test FAILED =" , TEST_INITIAL_X , TEST_RESULTS_Y + (testNumber*TEST_RESULTS_LINE_HEIGHT) ,1);
        tft.setTextColor (TFT_GREEN , cBackground);
    }
    return result;
}


//----------------------------------------
// Function to get a text message for the reset reason
//----------------------------------------
void getBootReasonMessage(char *buffer, int bufferlength) 
{
  esp_reset_reason_t reset_reason = esp_reset_reason();
  switch (reset_reason) {
    case ESP_RST_UNKNOWN:
      snprintf(buffer, bufferlength, "Reset reason can not be determined");
      break;
    case ESP_RST_POWERON:
      snprintf(buffer, bufferlength, "Reset due to power-on event");
      break;
    case ESP_RST_EXT:
      snprintf(buffer, bufferlength, "Reset by external pin (not applicable for ESP32)");
      break;
    case ESP_RST_SW:
      snprintf(buffer, bufferlength, "Software reset via esp_restart");
      break;
    case ESP_RST_PANIC:
      snprintf(buffer, bufferlength, "Software reset due to exception/panic");
      break;
    case ESP_RST_INT_WDT:
      snprintf(buffer, bufferlength, "Reset (software or hardware) due to interrupt watchdog");
      break;
    case ESP_RST_TASK_WDT:
      snprintf(buffer, bufferlength, "Reset due to task watchdog");
      break;
    case ESP_RST_WDT:
      snprintf(buffer, bufferlength, "Reset due to other watchdogs");
      break;
    case ESP_RST_DEEPSLEEP:
      snprintf(buffer, bufferlength, "Reset after exiting deep sleep mode");
      break;
    case ESP_RST_BROWNOUT:
      snprintf(buffer, bufferlength, "Brownout reset (software or hardware)");
      break;
    case ESP_RST_SDIO:
      snprintf(buffer, bufferlength, "Reset over SDIO");
      break;
    default:
      snprintf(buffer, bufferlength, "Unknown reset reason %d", reset_reason);
      break;
  }
}

//----------------------------------------
// Float Mapping function
//----------------------------------------
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}