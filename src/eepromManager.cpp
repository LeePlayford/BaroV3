

#include "eepromManager.h"

//-------------------------------------
//
//-------------------------------------
EepromManager::EepromManager() :
    m_EEPROM_TYPE(1)
{
    // Constructor
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock speed
}

//----------------------------------------
//
//----------------------------------------
void EepromManager::WriteEEPROM(int deviceaddress, unsigned int eeaddress, byte data ) 
{
    if (m_EEPROM_TYPE == 0)
    {
        WriteEEPROM0 (deviceaddress , eeaddress , data);
    }
    else
    {
        WriteEEPROM1 (deviceaddress , eeaddress , data);
    }
}

//----------------------------------------
//
//----------------------------------------
byte EepromManager::ReadEEPROM(int deviceaddress, unsigned int eeaddress ) 
{
    if (m_EEPROM_TYPE == 0)
    {
        return ReadEEPROM0 (deviceaddress , eeaddress);
    }
    else
    {
        return ReadEEPROM1 (deviceaddress , eeaddress);
    }
}

//----------------------------------------
//
//----------------------------------------
void EepromManager::WriteEEPROM0(int deviceaddress, unsigned int eeaddress, byte data ) 
{
    int addrOffset = eeaddress >> 8;
    addrOffset <<= 1;
    deviceaddress = deviceaddress | addrOffset;
    
    Wire.beginTransmission(deviceaddress);// + addrOffset);
    //Wire.write((int)((eeaddress >> 8) & 0xff)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(data);
    Wire.endTransmission();
 
    delay(5);
}
 
//----------------------------------------
//
//----------------------------------------
byte EepromManager::ReadEEPROM0(int deviceaddress, unsigned int eeaddress ) 
{
    byte rdata = 0xFF;
    int addrOffset = eeaddress >> 8;
    addrOffset <<= 1;
    deviceaddress = deviceaddress | addrOffset;

    Wire.beginTransmission(deviceaddress);// + addrOffset);
    //Wire.write((int)((eeaddress>> 8) & 0xff)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
 
    Wire.requestFrom(deviceaddress,1);
 
    if (Wire.available()) 
        rdata = Wire.read();
    else
    {
        // ROM read failure switch to other type
        m_EEPROM_TYPE = 1;
        Serial.println ("Switched to Type 1 Eeprom");
        return ReadEEPROM1 (deviceaddress , eeaddress);
    }
 
    return rdata;
}


//----------------------------------------
//
//----------------------------------------
void EepromManager::WriteEEPROM1(int deviceaddress, unsigned int eeaddress, byte data ) 
{
    int addrOffset = eeaddress >> 8;
    addrOffset <<= 1;
    deviceaddress = deviceaddress;// | addrOffset;
    
    Wire.beginTransmission(deviceaddress);// + addrOffset);
    Wire.write((int)((eeaddress >> 8) & 0xff)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.write(data);
    Wire.endTransmission();
 
    delay(5);
}
 
//----------------------------------------
//
//----------------------------------------
byte EepromManager::ReadEEPROM1(int deviceaddress, unsigned int eeaddress ) 
{
    byte rdata = 0xFF;
    int addrOffset = eeaddress >> 8;
    addrOffset <<= 1;
    deviceaddress = deviceaddress;// | addrOffset;

    Wire.beginTransmission(deviceaddress);// + addrOffset);
    Wire.write((int)((eeaddress>> 8) & 0xff)); // MSB
    Wire.write((int)(eeaddress & 0xFF)); // LSB
    Wire.endTransmission();
 
    Wire.requestFrom(deviceaddress,1);
 
    if (Wire.available()) rdata = Wire.read();
 
    return rdata;
}

//----------------------------------------
// Test EEprom Code
//----------------------------------------
bool EepromManager::TestEeprom(int p_Size , bool printResult , int location , TFT_eSPI &tft, int textX , int textY)
{
   
    // Print out the results
    if (printResult)
    {
        Serial.println ("Testing EEprom");
    }
    bool result = true;

    ReadEEPROM (EepromAddr , 0);
    char buf [32];


    for (int i = 0 ; i < p_Size; i++)
    {
        // read out existing data
        unsigned char origData1 = ReadEEPROM (EepromAddr , i*2);
        unsigned char origData2 = ReadEEPROM (EepromAddr , (i*2) + 1);
                
        // Test Eeprom
        WriteEEPROM (EepromAddr , i*2 , 0xaa);
        WriteEEPROM (EepromAddr , (i*2)+1 , 0x55);
        unsigned char byte1 = ReadEEPROM (EepromAddr , i*2);
        unsigned char byte2 = ReadEEPROM (EepromAddr , (i*2) + 1);
        
        if ((i % 40) == 0) // put it on the tft
        {
            float percent = ((float)i / (float)p_Size) * 100.f;
            if (printResult)
            {
                sprintf(buf , "%d%%" , (int) percent);
                Serial.printf ("Eeprom test %d%%\r\n" , (int) percent);
            }
            sprintf (buf , "Eeprom test %d%%" , (int)percent);
            tft.setTextColor (TFT_YELLOW , 0);
            tft.drawString (buf , textX , textY , 1);
        }
        
        if (byte1 != 0xaa || byte2 != 0x55)
        {
            result = false;
            if (printResult)
            {
                Serial.printf ("Eeprom test Failure at %d %x %x\r\n" , i*2 , byte1 , byte2);
                location = byte1 << 8 | byte2;
                break;
            }
            sprintf (buf , "Eeprom Fail at %d" , i*2);
            tft.setTextColor (TFT_RED , 0);
            tft.drawString (buf , textX , textY , 1);
        }
        else
        {
            if (printResult)
                Serial.printf ("Eeprom test OK at %d %x %x\r\n" , i*2 , byte1 , byte2);
        }

        // Write back existing
        WriteEEPROM (EepromAddr , i*2 , origData1);
        WriteEEPROM (EepromAddr , (i*2)+1 , origData2);
    }
    return result;
}  


//----------------------------------------
//
//----------------------------------------
void EepromManager::StoreData (uint16_t * p_Data , int p_Size , uint16_t &p_DataHead)
{
    uint16_t offset = p_DataHead;
    
#ifdef DEBUG    
    Serial.println ("Storing Data");
#endif

    // Store the data
    for (int i = 0 ; i < p_Size ; i++)
    {
        WriteEEPROM (EepromAddr , i*2 , p_Data[i] & 0xff);
        WriteEEPROM (EepromAddr , (i*2)+1 , (p_Data[i] >> 8) & 0xff);
    }
    WriteEEPROM (EepromAddr , HeadOffset , p_DataHead &0xff);
    WriteEEPROM (EepromAddr , HeadOffset2 , (p_DataHead >> 8) &0xff);   
}

//----------------------------------------
//
//----------------------------------------
void EepromManager::ReadData (uint16_t * p_Data , int p_Size , uint16_t &p_DataHead)
{
    // Read in the data
    for (int i = 0 ; i < p_Size ; i++)
    {
        uint16_t value = ReadEEPROM (EepromAddr , i*2);
        value |= ReadEEPROM(EepromAddr , (i*2)+1) << 8;
        if (value == 0xffff || value < MIN_BARO || value > MAX_BARO)
        {
            TestFillEeprom(p_Data, p_Size);
            return;
        }
        else
            p_Data[i] = value;

    }
    // Read in the data write position
    p_DataHead = ReadEEPROM (EepromAddr , HeadOffset);
    p_DataHead |= (ReadEEPROM (EepromAddr , HeadOffset2) << 8);

#ifdef DEBUG
    Serial.printf ("Read Data Head %d\r\n" , p_DataHead);
#endif

// Check the data head
    if (p_DataHead < 0 || p_DataHead >= p_Size)
    {
        Serial.printf ("Data Head out of range %d\r\n" , p_DataHead);
        p_DataHead = 0;
    }
    else
        Serial.printf ("Data Head %d\r\n" , p_DataHead);
    
}

//-----------------------------------------------------
// Test Fill EEprom
//-----------------------------------------------------
void EepromManager::TestFillEeprom(uint16_t * p_Data, int p_Size)
{
    Serial.println ("Test Filling EEprom");
    for (int i = 0 ; i < p_Size ; i++)
    {
        // Put a sine wave in it
        // 400 points, each point is 1 degree
        const float DEG2RAD = 180.0f / 3.141;
        float sinValue = sin (i / DEG2RAD);
        sinValue *= 20.0f;    //amplify by 10
        // it to 1000mb
        sinValue += 10000.0f;
        int value = (int16_t) sinValue;
        WriteEEPROM(EepromAddr , i*2 , value & 0xff);
        WriteEEPROM(EepromAddr , (i*2)+1 , (value>>8) & 0xff);

        p_Data[i] = sinValue;
    }
    
    // Reset the data Head
    WriteEEPROM (EepromAddr , HeadOffset , 0);
    WriteEEPROM (EepromAddr , HeadOffset2 , 0);   

}

