//-------------------------------------
//
//-------------------------------------

#include "lcdScreen.h"
#include "Free_Fonts.h"


//-------------------------------------
//
//-------------------------------------
LcdScreen::LcdScreen() :
    m_tft(TFT_eSPI())
{
    // Constructor
}
//-------------------------------------
//
//-------------------------------------
void LcdScreen::Init (void)
{
    m_tft.begin();
    m_tft.setRotation(3);
    m_tft.fillScreen(TFT_BLACK);
    m_tft.setTextDatum (TL_DATUM);
    m_tft.setFreeFont(FSS12);
    m_paddingBaro = m_tft.textWidth ("9999.9" , 1);

    m_tft.setFreeFont (FSS9);
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void LcdScreen::DrawInitScreen()
{
    // put a box on the screen
    m_tft.setFreeFont (FSS12);

    m_tft.fillScreen (BLACK);
    m_tft.fillRect (0, 0 , 479 , 4 , GREEN);      // Top Line
    m_tft.fillRect (0, 316 , 479 , 4 , GREEN);    // Bottom Line
    m_tft.fillRect (0 , 4 , 4 , 319 , GREEN);     // Left Line
    m_tft.fillRect (475 , 0 , 4 , 319 , GREEN);   // Right Line
    m_tft.fillRect (0, 66 , 475 , 4 , GREEN);     // Horiz Divider
    m_tft.fillRect (66 , 70 , 2 , 250 , GREEN);   // Vertical Divider

    m_tft.setTextColor(YELLOW);
    m_tft.setFreeFont (FSS9);
    m_tft.setTextSize(1);

    m_tft.setCursor (10, 30);
    m_tft.print("Baro");
    m_tft.setCursor (140, 30);
    m_tft.print("High");
    m_tft.setCursor (10, 60);
    m_tft.print("Trend");
    m_tft.setCursor (260, 30);
    m_tft.print("Low");
    m_tft.setCursor (410, 30);
    m_tft.print("24 Hrs");
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void LcdScreen::UpdateDelta (int16_t delta)
{
 
    int16_t x = 290 + m_paddingBaro;
    int16_t y = 42;

    m_tft.setFreeFont(FSS12);
    m_tft.setTextColor(CYAN , BLACK);
    m_tft.setTextDatum (TR_DATUM);
    m_tft.setTextPadding (m_paddingBaro);
    m_tft.drawFloat (delta / 10.0 , 1 , x , y , 1);
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void LcdScreen::UpdateLow (uint16_t low)
{
    m_tft.setFreeFont(FSS12);
    m_tft.setTextColor(CYAN , BLACK);
    m_tft.setTextDatum (TR_DATUM);
    int16_t x = 298 + m_paddingBaro;
    int16_t y = 12;
    m_tft.setTextPadding (m_paddingBaro);
    m_tft.drawFloat (low / 10.0 , 1 , x , y , 1);
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void LcdScreen::UpdateHigh (uint16_t high)
{   
    m_tft.setFreeFont(FSS12);
    m_tft.setTextColor(CYAN , BLACK);
    m_tft.setTextDatum (TR_DATUM);
    int16_t x = 180 + m_paddingBaro;
    int16_t y = 12;
    m_tft.setTextPadding (m_paddingBaro);
    m_tft.drawFloat (high / 10.0 , 1 , x , y , 1);
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void LcdScreen::UpdateTrend (int16_t baro , int16_t threeHours)
{

    m_tft.setFreeFont(FSS12);
    static int16_t padding = m_tft.textWidth (" Falling Rapidlyy ");
    // get last update and 3 hours before

    int16_t diff = baro - threeHours;
    
    UpdateDelta(diff);
    const int8_t BUF_SIZE = 18;
    char f_r [BUF_SIZE];
    memset (f_r , 0x0 , BUF_SIZE);
    if (abs(diff) < 1)
    {
        strcpy (f_r , "Steady ");
    }
    else
    {
        if (diff > 0)
            strcpy (f_r , "Rising ");
        else
            strcpy (f_r , "Falling ");
         
        if (abs(diff) < 15)
            strcat (f_r , "Slowly"); 
        else if (abs(diff) < 35)
            strcat (f_r , ""); 
        else if (abs(diff) < 60)
            strcat (f_r , "Quickly"); 
        else
            strcat (f_r , "Rapidly");
    } 

    int16_t x = 80;
    int16_t y = 42;
    
    m_tft.setTextColor(MAGENTA , BLACK);
    m_tft.setTextDatum (TL_DATUM);
    m_tft.setTextPadding (padding);
    m_tft.drawString (f_r , x , y , 1);
}

//---------------------------------------------------------------------
//
//---------------------------------------------------------------------
void LcdScreen::UpdateBaro (int16_t baro , int16_t high , int16_t low , int16_t range)
{
    UpdateHigh (high);
    UpdateLow (low);

    m_tft.setFreeFont(FSS12);
    int16_t x = 58 + m_paddingBaro;
    int16_t y = 12;
    m_tft.setTextDatum (TR_DATUM);
    m_tft.setTextColor(CYAN , BLACK);
    m_tft.setTextPadding (m_paddingBaro);
    m_tft.drawFloat (baro / 10.0 , 1 , x , y , 1);
}

//----------------------------------------
//
//----------------------------------------
void LcdScreen::AddScale (uint16_t baro , uint16_t stepVal)
{
    // Clear out box
    m_tft.fillRect (5,70,62,245 , BLACK);
    uint16_t yPos = 298;

    m_tft.setFreeFont(FSS9);
    m_tft.setTextColor(CYAN , BLACK);
    m_tft.setTextPadding (m_paddingBaro-20);
    m_tft.setTextDatum (TR_DATUM);

    for (int i = 0 ; i <= 5 ; i++)
    {
        m_tft.drawFloat (baro / 10.0 , 1 , m_paddingBaro-8 , yPos , 1);
        baro += stepVal;
        yPos -= 45;
    }
}

//----------------------------------------
//
//----------------------------------------
void LcdScreen::DrawBaro (uint16_t * p_pBaroData , int p_Size , uint16_t offset ,uint16_t high , uint16_t low , uint16_t range)
{
    uint16_t lastX = 0;
    uint16_t lastY = 0;

    // reset yPos filter
    for (int i=0; i < Y_FILTER_SIZE ; i++)
    {
        m_yPosFilter[i] = 0;
    }
    
    for (int i = 0 ; i < p_Size ; i++)
    {
        // need to draw the pixel
        // scale between 950 - 1050

        uint16_t baro = p_pBaroData[offset];
        uint16_t yPos = Interpolate (baro , high , low , TOP_GRAPH , BOTTOM_GRAPH);
        
        yPos = FilterDisplay(yPos);

        if (yPos < TOP_GRAPH) yPos = TOP_GRAPH;
        if (yPos > BOTTOM_GRAPH) yPos = BOTTOM_GRAPH;
        
        uint16_t xPos = LEFT_GRAPH + i;
        if (lastY != 0)
        {
            // Over draw the previous colours
            for (int x = lastX ; x <= xPos ; x++)
            {
                if (x == 420)
                    m_tft.drawFastVLine (x , TOP_GRAPH , GRAPH_HEIGHT , YELLOW);
                else if ((x-20)%50 == 0 )
                    m_tft.drawFastVLine (x , TOP_GRAPH , GRAPH_HEIGHT , DARKGREEN);
                else
                    m_tft.drawFastVLine (x , TOP_GRAPH , GRAPH_HEIGHT , BLACK);
            }

            // draw a line
            m_tft.drawLine (lastX , lastY , xPos , yPos , LIGHTGREEN);
            m_tft.drawLine (lastX , lastY+1 , xPos , yPos+1 , LIGHTGREEN);
        }

        // draw the Horiontal lines
        for (int y = low ; y <= high ; y+= range / 5)
        {
            uint16_t yPos = Interpolate (y , high , low , TOP_GRAPH , BOTTOM_GRAPH);
            m_tft.drawFastHLine (LEFT_GRAPH , yPos , GRAPH_WIDTH , DARKGREEN);           
        }

        // keep the last positions
        lastX = xPos;
        lastY = yPos;
        
        // check range  
        if (++offset >= p_Size) offset = 0;
    }
}

//----------------------------------------
//
//----------------------------------------
void LcdScreen::SplashScreen (bool wifiConnected , IPAddress ip , String mac)
{
    const int cBackground = TFT_BLACK;
    m_tft.fillScreen (cBackground);
    m_tft.setFreeFont (FSS24);
    m_tft.setTextColor(TFT_SKYBLUE , cBackground);
    m_tft.setTextDatum (TL_DATUM);
    m_tft.drawString ("Barograph" , 150 , 100 , 1);

    m_tft.setFreeFont (FSS12);
    m_tft.drawString ("Version" , 150 , 150 , 1);
    m_tft.drawString (version , 300 , 150 , 1);
    m_tft.setTextColor(TFT_YELLOW , cBackground);
    m_tft.drawString ("Lee Playford (c) 2025" , 150 , 200 , 1);

    if (wifiConnected)
    {
        m_tft.setTextColor(TFT_GREEN , cBackground);
        m_tft.drawString ("IPAddress" , 50 , 275 , 1);
        m_tft.drawString (ip.toString() , 250 , 275 , 1);
        m_tft.drawString ("MAC Address" , 50 , 300 , 1);
        m_tft.drawString (mac , 250 , 300 , 1);
    }
}



//----------------------------------------
//
//----------------------------------------
int16_t LcdScreen::Interpolate (uint16_t value , uint16_t fromHigh , uint16_t fromLow , uint16_t toHigh, uint16_t toLow)
{
    float ffromLow = fromLow;
    float ffromHigh = fromHigh;
    float ftoLow = toLow;
    float ftoHigh = toHigh;
    float fvalue = value;
    float result = (fvalue - ffromLow ) * (ftoHigh - ftoLow) / (ffromHigh - ffromLow) + ftoLow;
    return (int16_t) result;
}

//----------------------------------------
//
//----------------------------------------
uint16_t LcdScreen::FilterDisplay (uint16_t yPos)
{
    static int head = 0;
    // init the filter
    if (m_yPosFilter[0] == 0)
    {
        for (int i = 0 ; i < Y_FILTER_SIZE ; i++)
        {
            m_yPosFilter[i] = yPos;
        }
        return yPos;
    }
    else 
    {
        m_yPosFilter[head++] = yPos;
        if (head == Y_FILTER_SIZE)
            head = 0;
    }
    uint32_t total = 0; 
    for (int i = 0 ; i < Y_FILTER_SIZE ; i++)
    {
        total += m_yPosFilter[i];
    }
    return total / Y_FILTER_SIZE;
}


