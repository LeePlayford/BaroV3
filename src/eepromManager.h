//-------------------------------------
//
//-------------------------------------
#ifndef EEPROMMANAGER_H
#define EEPROMMANAGER_H
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

//-------------------------------------
//
//-------------------------------------
// eeprom data
const int EepromAddr = 0x50;  
const int HeadOffset = 0x3fe; // Head offset
const int HeadOffset2 = 0x3ff; // Head offset
const int MIN_BARO = 9000; // Minimum baro value
const int MAX_BARO = 11000; // Maximum baro value


//-------------------------------------
//
//-------------------------------------
class EepromManager
{
public:
    EepromManager();
    ~EepromManager(){}

    // Read Write functions
    void ReadData (uint16_t * p_Data , int p_Size , uint16_t &p_DataHead);
    void StoreData (uint16_t * p_Data , int p_Size , uint16_t &p_DataHead);
   

    // Test the eeprom
    bool TestEeprom(int p_Size , bool printResult , int location );


private:
    byte ReadEEPROM(int deviceaddress, unsigned int eeaddress );
    void WriteEEPROM(int deviceaddress, unsigned int eeaddress, byte data );

    // Type 0 EEprom - now deprectated
    void WriteEEPROM0(int deviceaddress, unsigned int eeaddress, byte data ) ;
    byte ReadEEPROM0(int deviceaddress, unsigned int eeaddress ) ;

    // Type 1 EEprom
    // 24LC32
    // 24LC64
    // 24LC128
    void WriteEEPROM1(int deviceaddress, unsigned int eeaddress, byte data ) ;
    byte ReadEEPROM1(int deviceaddress, unsigned int eeaddress ) ;

    void TestFillEeprom(uint16_t * p_Data, int p_Size);


    bool m_EEPROM_TYPE;

};
#endif