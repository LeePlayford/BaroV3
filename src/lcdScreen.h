//-------------------------------------
//
//-------------------------------------
#ifndef LCD_SCREEN_H
#define LCD_SCREEN_H
#include <TFT_eSPI.h>
#include <SPI.h>
#include <IPAddress.h>
//-------------------------------------
// Consts and defines
//-------------------------------------

constexpr char version[] = "2.00";

// Colour Defines
#define BLACK   0x0000
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0

#define RGB(r, g, b) (((r&0xF8)<<8)|((g&0xFC)<<3)|(b>>3))

#define DARKGREY  RGB(64, 64, 64)
#define DARKGREEN RGB(0,128,0)
#define LIGHTGREEN RGB(0,255,128)

#define Y_FILTER_SIZE 8

const uint16_t HEIGHT = 319;
const uint16_t WIDTH = 479;
const uint16_t GRAPH_WIDTH = 400;
const uint16_t GRAPH_HEIGHT = 245;
const uint16_t TOP_GRAPH = 70;
const uint16_t GRADULE = 66;
const uint16_t BOTTOM_GRAPH= 310;
const uint16_t LEFT_GRAPH = 70;


//-------------------------------------
// LCD SCREEN Class
//-------------------------------------
class LcdScreen
{
public:
    LcdScreen();
    ~LcdScreen(){}

    void Init (void);
    void DrawInitScreen (void);
    void UpdateHigh (uint16_t high);
    void UpdateLow (uint16_t low);
    void UpdateTrend (int16_t baro , int16_t threeHours);
    void UpdateBaro (int16_t baro , int16_t high , int16_t low , int16_t range);
    void UpdateBaroTrend (int16_t baro);
    void UpdateDelta (int16_t delta);  
    void AddScale (uint16_t baro , uint16_t stepVal);
    void SplashScreen (bool wifiConnected , IPAddress ip , String mac);
    void DrawBaro (uint16_t * baroData , int p_Size, uint16_t baroHead ,uint16_t high , uint16_t low , uint16_t range);

private:
    TFT_eSPI m_tft;
    int m_paddingBaro;
    uint16_t m_yPosFilter[Y_FILTER_SIZE];

    int16_t Interpolate (uint16_t value , uint16_t fromHigh , uint16_t fromLow , uint16_t toHigh, uint16_t toLow);
    uint16_t FilterDisplay (uint16_t yPos);



};

#endif