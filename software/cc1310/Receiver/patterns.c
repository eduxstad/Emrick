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
#include <ti/display/DisplayUart.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include "patterns.h"
#include <sched.h>
#include <math.h>


#include <ti/drivers/ADC.h>


//TODO : ADD limitation on NB_PIXELS

const uint8_t HSVlights[61] =
{0, 4, 8, 13, 17, 21, 25, 30, 34, 38, 42, 47, 51, 55, 59, 64, 68, 72, 76,
81, 85, 89, 93, 98, 102, 106, 110, 115, 119, 123, 127, 132, 136, 140, 144,
149, 153, 157, 161, 166, 170, 174, 178, 183, 187, 191, 195, 200, 204, 208,
212, 217, 221, 225, 229, 234, 238, 242, 246, 251, 255};

uint8_t setArr[NB_PIXELS * 3] = {0};


uint16_t loc_u16_pixelIndex;
uint16_t arrIdx = 0;


RGB hsvToRgb(float h, float s, float v) {
    RGB rgb;

    if (s == 0.0)
    {
        rgb.r = v;
        rgb.g = v;
        rgb.b = v;
        return rgb;
    }
    float M = 255 * v;
    float m = M * (1 - s);
    float z =((M - m) * (1-fabs(fmod(h/60.0,2.0)-1)));

    if (h < 60) {
        rgb.r = (uint8_t) M;
        rgb.g = (uint8_t) (z + m);
        rgb.b = (uint8_t) m;
    } else if (h < 120) {
        rgb.r = (uint8_t) (z + m);
        rgb.g = (uint8_t) M;
        rgb.b = (uint8_t) m;
    } else if (h < 180) {
        rgb.r = (uint8_t) m;
        rgb.g = (uint8_t) M;
        rgb.b = (uint8_t) (z + m);
    } else if (h < 240) {
        rgb.r = (uint8_t) m;
        rgb.g = (uint8_t) (z + m);
        rgb.b = (uint8_t) M;
    } else if (h < 300) {
        rgb.r = (uint8_t) (z + m);
        rgb.g = (uint8_t) m;
        rgb.b = (uint8_t) M;
    } else {
        rgb.r = (uint8_t) M;
        rgb.g = (uint8_t) m;
        rgb.b = (uint8_t) (z + m);
    }


    return rgb;
}






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

void animationBasic(void)
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
            else if (pick == 1)
            {
                setArr[arrIdx] = 0x00;
                setArr[arrIdx + 1] = 0xFF;
                setArr[arrIdx + 2] = 0x0;
                pick = 2;
            }
            else
            {
                setArr[arrIdx] = 0x00;
                setArr[arrIdx + 1] = 0x0;
               setArr[arrIdx + 2] = 0xFF;
               pick = 0;
            }
            arrIdx = arrIdx + 3;
        }
        arrIdx = 0;
        for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
        {
            WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
            arrIdx = arrIdx + 3;
            WS2812_show();
            usleep(30000);
        }



}

void rainbowAnimation(void)
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
            WS2812_show();
            usleep(10000);
        }
    }

    free(rainbowArr);
}

void bounce(void)
{
    for(arrIdx = 0; arrIdx < NB_PIXELS * 3;)
        {
            setArr[arrIdx + 1] = 0x00;
            setArr[arrIdx + 2] = 0xFF;
            setArr[arrIdx] = 0x00;
            arrIdx = arrIdx + 3;
        }

        arrIdx = 0;
        for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
        {
            WS2812_setPixelColor(loc_u16_pixelIndex, setArr[arrIdx], setArr[arrIdx + 1], setArr[arrIdx + 2]);
            arrIdx = arrIdx + 3;
            WS2812_show();
            usleep(100000);

        }


}

void rainbowGradient(void)
{
    int j = 0;
        WS2812_beginSPI();
    for (j = 0; j < 1000; j++) {
        //Start at red, go to green
        //pthread_mutex_lock(&LEDMutex);
        int i = 0;
        useconds_t time = 15000;
        for (i = 0; i < 255; i+=5) {
            for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++){
                WS2812_setPixelColor(loc_u16_pixelIndex, 255 - i, i, 0);
            }
            if (testFlag == 1) {
                WS2812_setPixelColor(NB_PIXELS / 2, 255, 255, 255);
            }
                WS2812_show();
            usleep(time); //Complete guesstimation on timing it with the old lights
        }


        //Start at green, go to blue
        for (i = 0; i < 255; i+=5) {

                for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
                        {
                            WS2812_setPixelColor(loc_u16_pixelIndex, 0, 255 - i, i);
                        }
                if (testFlag == 1) {
                    WS2812_setPixelColor(NB_PIXELS / 2, 255, 255, 255);
                }
                            WS2812_show();
                usleep(time);
            }

        //Start at blue, go to white
        for (i = 0; i < 255; i+=5) {

                for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
                        {
                            WS2812_setPixelColor(loc_u16_pixelIndex, i, i, 255);
                        }
                if (testFlag == 1) {
                    WS2812_setPixelColor(NB_PIXELS / 2, 255, 255, 255);
                }
                         WS2812_show();
                usleep(time);
        }

        //Start at white, go to red
        for (i = 0; i < 255; i+=5) {

                for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
                        {
                            WS2812_setPixelColor(loc_u16_pixelIndex, 255, 255-i, 255 - i);
                        }
                if (testFlag == 1) {
                    WS2812_setPixelColor(NB_PIXELS / 2, 255, 255, 255);
                }
                            WS2812_show();
                usleep(time);
            }
        //pthread_mutex_unlock(&LEDMutex);
    }
}


void rainbowGradientHSV(Display_Handle displayHandle) {
    int i = 0;
    useconds_t time = 20700;
    uint16_t test_led = 3;
    while (1) {
        int j = 0;
        for (j = 0; j < 300; j++) {
            RGB rgb;
            pthread_mutex_lock(&LEDMutex);
            for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
                rgb = hsvToRgb((float)j,1.0,1.0);
                WS2812_setPixelColor(loc_u16_pixelIndex, rgb.r, rgb.g, rgb.b);
            }
            if (testFlag == 1) {
                WS2812_setPixelColor(test_led, 255, 255, 255);
            }

            //Display_printf(displayHandle, DisplayUart_SCROLLING, 0,"%d, %d, %d, %d", j, rgb.r, rgb.g, rgb.b);
            WS2812_show();
            pthread_mutex_unlock(&LEDMutex);
            usleep(time);
        }
//        WS2812_close();
//        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,"Battery Voltage %d", battMicroVoltage());
//        WS2812_restartSPI();
        float k = 0;
        for (k = 0; k < 1.0; k += 0.011) {
            RGB rgb;
            pthread_mutex_lock(&LEDMutex);
            for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
                rgb = hsvToRgb((float)300,1.0 - k,1.0);
                WS2812_setPixelColor(loc_u16_pixelIndex, rgb.r, rgb.g, rgb.b);
            }
            if (testFlag == 1) {
                WS2812_setPixelColor(test_led, 255, 255, 255);
            }

            //Display_printf(displayHandle, DisplayUart_SCROLLING, 0,"%d, %d, %d, %d", j, rgb.r, rgb.g, rgb.b);
            WS2812_show();
            pthread_mutex_unlock(&LEDMutex);
            usleep(time);
        }
        for (j = 1; j < 60; j++) {
            RGB rgb;
            pthread_mutex_lock(&LEDMutex);
            for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
                rgb = hsvToRgb(0.0,(float)j / 60.0,1.0);
                WS2812_setPixelColor(loc_u16_pixelIndex, rgb.r, rgb.g, rgb.b);
            }
            if (testFlag == 1) {
                WS2812_setPixelColor(test_led, 255, 255, 255);
            }

            //Display_printf(displayHandle, DisplayUart_SCROLLING, 0,"%d, %d, %d, %d", j, rgb.r, rgb.g, rgb.b);
            WS2812_show();
            pthread_mutex_unlock(&LEDMutex);
            usleep(time);
        }
    }
}

uint32_t battMicroVoltage(Display_Handle displayHandle)
{
    ADC_Handle adc;
    ADC_Params adcparams;
    uint16_t adcValue0;
    uint32_t adcValue0MicroVolt;
    int_fast16_t res;

    ADC_Params_init(&adcparams);

    adc = ADC_open(Board_ADC0, &adcparams);

    if (adc != NULL)
    {
        res = ADC_convert(adc, &adcValue0);

        if (res == ADC_STATUS_SUCCESS)
        {

            adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
            ADC_close(adc);
            return adcValue0MicroVolt;

            //Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "ADC0 raw result: %d", adcValue0);

        }
        else
        {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                           "ADC0 convert failed\n");
        }

    }
    else
    {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                       "Error initializing ADC0\n");
    }

    ADC_close(adc);

}
