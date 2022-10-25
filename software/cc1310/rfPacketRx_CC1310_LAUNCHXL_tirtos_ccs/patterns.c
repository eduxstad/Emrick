#include <stddef.h>
#include <string.h>

/* Example/Board Header files */
#include "Board.h"
#include "WS2812.h"

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

//TODO : ADD limitation on NB_PIXELS

const uint8_t HSVlights[61] =
{0, 4, 8, 13, 17, 21, 25, 30, 34, 38, 42, 47, 51, 55, 59, 64, 68, 72, 76,
81, 85, 89, 93, 98, 102, 106, 110, 115, 119, 123, 127, 132, 136, 140, 144,
149, 153, 157, 161, 166, 170, 174, 178, 183, 187, 191, 195, 200, 204, 208,
212, 217, 221, 225, 229, 234, 238, 242, 246, 251, 255};

uint16_t loc_u16_pixelIndex;

void trueHSV(int angle, int * red, int * green, int * blue)
{
  //uint8_t red, green, blue;

  if (angle<60) {*red = 255; *green = HSVlights[angle]; *blue = 0;} else
  if (angle<120) {*red = HSVlights[120-angle]; *green = 255; *blue = 0;} else
  if (angle<180) {*red = 0, *green = 255; *blue = HSVlights[angle-120];} else
  if (angle<240) {*red = 0, *green = HSVlights[240-angle]; *blue = 255;} else
  if (angle<300) {*red = HSVlights[angle-240], *green = 0; *blue = 255;} else
                 {*red = 255, *green = 0; *blue = HSVlights[360-angle];}
  //WS2812_setPixelColor(LED, red, green, blue);
}

void ResetLights(void)
{
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0x00, 0x00, 0x00);
    }

    WS2812_show();
}
void allWhite(void)
{
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0xFF, 0xFF, 0xFF);
    }

    WS2812_show();
}

void allRed(void)
{
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0xFF, 0x00, 0x00);
    }

    WS2812_show();
}

void allBlue(void)
{
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0x00, 0x00, 0xFF);
    }

    WS2812_show();
}

void allGreen(void)
{
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0x00, 0xFF, 0x00);
    }

    WS2812_show();
}

void chirstLights(void)
{
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex = loc_u16_pixelIndex + 2)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0xFF, 0x00, 0x00);
        WS2812_setPixelColor(loc_u16_pixelIndex + 1, 0x00, 0xFF, 0x00);
    }

    WS2812_show();
}

void rainbow(void)
{
    uint32_t i;
    int red, green, blue;
    for(i = 0; i < 0xFF; i++)
    {
        for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
        {
            //WS2812_setPixelColor(loc_u16_pixelIndex, i, i, i);
            trueHSV(i, &red, &green, &blue);
            WS2812_setPixelColor(loc_u16_pixelIndex, red, green, blue);
        }

        WS2812_show();
    }

    usleep(20000);
}
