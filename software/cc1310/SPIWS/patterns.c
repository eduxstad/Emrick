#include <stddef.h>
#include <string.h>
#include <stdlib.h>

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

uint8_t setArr[NB_PIXELS * 3] = {0};


uint16_t loc_u16_pixelIndex;
uint16_t arrIdx = 0;

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
    for(arrIdx = 0; arrIdx < NB_PIXELS * 3; arrIdx++)
    {
        setArr[arrIdx] = 0xFF;
    }

    arrIdx = 0;
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
        arrIdx = arrIdx + 3;
    }

    WS2812_show();
}

void allRed(void)
{
    for(arrIdx = 0; arrIdx < NB_PIXELS * 3;)
    {
        setArr[arrIdx + 1] = 0x0;
        setArr[arrIdx + 2] = 0x0;
        setArr[arrIdx] = 0xFF;
        arrIdx = arrIdx + 3;
    }

    arrIdx = 0;
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
        arrIdx = arrIdx + 3;
    }

    WS2812_show();
}

void allBlue(void)
{
    for(arrIdx = 2; arrIdx < NB_PIXELS * 3;)
    {
        setArr[arrIdx - 1] = 0x0;
        setArr[arrIdx - 2] = 0x0;
        setArr[arrIdx] = 0xFF;
        arrIdx = arrIdx + 3;
    }

    arrIdx = 0;
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
        arrIdx = arrIdx + 3;
    }

    WS2812_show();
}

void allGreen(void)
{
    for(arrIdx = 1; arrIdx < NB_PIXELS * 3;)
    {
        setArr[arrIdx - 1] = 0x0;
        setArr[arrIdx + 1] = 0x0;
        setArr[arrIdx] = 0xFF;
        arrIdx = arrIdx + 3;
    }

    arrIdx = 0;
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
        arrIdx = arrIdx + 3;
    }

    WS2812_show();
}

void chirstLights(void)
{
    int pick = 0;
    for(arrIdx = 0; arrIdx < NB_PIXELS * 3;)
    {
        if(pick == 0)
        {
            setArr[arrIdx] = 0xFF;
            setArr[arrIdx + 1] = 0x0;
            setArr[arrIdx + 2] = 0x0;
            pick = 1;
        }
        else
        {
            setArr[arrIdx] = 0x00;
            setArr[arrIdx + 1] = 0xFF;
            setArr[arrIdx + 2] = 0x0;
            pick = 0;
        }
        arrIdx = arrIdx + 3;
    }

    arrIdx = 0;
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
        arrIdx = arrIdx + 3;
    }

    WS2812_show();
}

void rainbow(void)
{
    int red, green, blue;
    int i, j;
    uint8_t *rainbowArr = (uint8_t *)malloc(sizeof(uint8_t) * NB_PIXELS * 3 * 12);


    for(j = 0; j < 12; j=j+1)
    {
        for(i = 0; i < NB_PIXELS; i = i + 1)
        {
            trueHSV(j*30, &red, &green, &blue);
            *(rainbowArr+3*i+(3*NB_PIXELS)*j) = (uint8_t) red;
            *(rainbowArr+(3*i + 1)+(3*NB_PIXELS)*j) = (uint8_t) green;
            *(rainbowArr+(3*i + 2)+(3*NB_PIXELS)*j) = (uint8_t) blue;
        }
    }

    for(i = 0; i < 12; i=i+1)
    {
        arrIdx = 0;
        for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
        {
            //WS2812_setPixelColor(loc_u16_pixelIndex, rainbowArr[i][arrIdx], rainbowArr[i][arrIdx + 1], rainbowArr[i][arrIdx + 2]);
            WS2812_setPixelColor(loc_u16_pixelIndex, *(rainbowArr+arrIdx+(3*NB_PIXELS)*i), *(rainbowArr+(arrIdx + 1)+(3*NB_PIXELS)*i), *(rainbowArr+(arrIdx + 2)+(3*NB_PIXELS)*i));
            arrIdx = arrIdx + 3;
        }

        WS2812_show();
        //usleep(137000);
        sleep(1);
    }

    free(rainbowArr);
}
