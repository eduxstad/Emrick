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
#include <ti/drivers/NVS.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include "patterns.h"
#include <sched.h>
#include <math.h>
#include <time.h>


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

void * status;

void *defaultLEDFunction(void *args);

Display_Handle displayHandle;

char *controlToChar(control control, char * ch) {
    ch[0] = control.size;
    ch[1] = control.light_show_flags >> 8 & 0xFF;
    ch[2] = control.light_show_flags & 0xFF;
    ch[3] = control.start_color.r;
    ch[4] = control.start_color.g;
    ch[5] = control.start_color.b;
    ch[6] = control.end_color.r;
    ch[7] = control.end_color.g;
    ch[8] = control.end_color.b;
    ch[9] = control.delay >> 24 & 0xFF;
    ch[10] = control.delay >> 16 & 0xFF;
    ch[11] = control.delay >> 8 & 0xFF;
    ch[12] = control.delay & 0xFF;
    ch[13] = control.duration >> 8 & 0xFF;
    ch[14] = control.duration & 0xFF;
    ch[15] = control.timeout;
    return ch;
}


 control charToControl(char * ch) {
    control control;
    control.size = ch[0];
    control.light_show_flags += ch[1];
    control.light_show_flags = control.light_show_flags << 8;
    control.light_show_flags += ch[2];
    control.start_color.r = ch[3];
    control.start_color.g = ch[4];
    control.start_color.b = ch[5];
    control.end_color.r = ch[6];
    control.end_color.g = ch[7];
    control.end_color.b = ch[8];
    control.delay = ch[9];
    control.delay = control.delay << 8;
    control.delay += ch[10];
    control.delay = control.delay << 8;
    control.delay += ch[11];
    control.delay = control.delay << 8;
    control.delay += ch[12];
    control.duration = ch[13];
    control.duration = control.duration << 8;
    control.duration += ch[14];
    control.timeout = ch[15];
    return control;
}

/* Contains control logic for running and switching LED patterns */
void runLED(Display_Handle dh) {

    // TODO: Check if the board is being charged before turning on the LED
    displayHandle = dh;
    // Turn on power by enabling the 5v supply
    GPIO_setConfig(Board_GPIO_BOOST_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    SPI_init();
    WS2812_beginSPI();

    // TODO: parse new packets and create new thread for new running pattern

    struct sched_param  priParam;
    int                 retc;
    int                 detachState;
    pthread_attr_t attrs;
    pthread_t           thread;
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_JOINABLE;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, 512);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create RX Listener thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    RGB red;
    red.r = 255;
    red.g = 0;
    red.b = 0;

    RGB blue;
    blue.r = 0;
    blue.g = 0;
    blue.b = 255;

    // Default pattern

    pattern.light_show_flags |= STALL_PATTERN;
    pattern.light_show_flags |= SET_TIMEOUT;
    pattern.light_show_flags |= COLOR_SHIFT;
    pattern.light_show_flags |= FINITE_DURATION;
    pattern.light_show_flags |= DEFAULT_FUNCTION;
    pattern.light_show_flags |= DO_DELAY;
    pattern.light_show_flags |= SHIFT_POST_DELAY;
    pattern.delay = 1000;
    pattern.duration = 5000;
    pattern.timeout = 20;
    pattern.start_color = red;
    pattern.end_color = blue;
    pattern.size = 15;


    NVS_Handle nvsRegion;
    NVS_Attrs regionAttrs;

    uint_fast16_t status;
    char buf[32];
    char pbuf[32];

    controlToChar(pattern, pbuf);


    nvsRegion = NVS_open(Board_NVSINTERNAL, NULL);

    NVS_getAttrs(nvsRegion, &regionAttrs);
    NVS_erase(nvsRegion, 0, regionAttrs.sectorSize);

    NVS_write(nvsRegion, 0, pbuf, 16, NVS_WRITE_POST_VERIFY);

    NVS_read(nvsRegion, 0, buf, 16);



    while(1) {
        pthread_create(&thread, &attrs, defaultLEDFunction, NULL);
        pthread_join(thread, &status);
    }

    // Turn off power
    GPIO_setConfig(Board_GPIO_BOOST_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    WS2812_close();
}

/*
 * Parameters: 2 unsigned 8-bit ints
 * Returns: Unsigned difference between the input ints
 * */

uint8_t diff(uint8_t x, uint8_t y) {
    if (x > y) {
        return x - y;
    } else {
        return y - x;
    }
}


/*
 * Parameters: 2 RGB structs representing RGB colors
 * Returns: Greatest difference between the R, G, or B values of the 2 colors
 * */

uint8_t maxColorDiff(RGB c1, RGB c2) {
    uint8_t max = 0;
    uint8_t rdiff = 0;
    uint8_t gdiff = 0;
    uint8_t bdiff = 0;

    rdiff = diff(c1.r, c2.r);
    gdiff = diff(c1.g, c2.g);
    bdiff = diff(c1.b, c2.b);


    max = rdiff;
    if (gdiff > max) {
        max = gdiff;
    }
    if (bdiff > max) {
        max = bdiff;
    }
    return max;
}

/*
 * returns a time value converted to milliseconds
 * */

uint32_t mstime(struct timespec t1) {
    return t1.tv_sec * 1000 + t1.tv_nsec / 1000000;
}

/*
 * Waits for a specified number of milliseconds then returns. While waiting, the function checks to see if a new
 * packet has arrived which would indicate the thread should exit
 * */

void busyWait(uint32_t wait_time) {
    struct timespec ts_start;
    clock_gettime(CLOCK_MONOTONIC, &ts_start);
    struct timespec ts_curr;
    clock_gettime(CLOCK_MONOTONIC, &ts_curr);
    while (mstime(ts_curr) < mstime(ts_start) + wait_time) {
        if (function_flag == 0xff) {
            function_flag == 0x00;
            pthread_exit(status);
        }
        clock_gettime(CLOCK_MONOTONIC, &ts_curr);
    }
}

/*
 * Runs LED pattern specified by receive_control structure
 * */

void *defaultLEDFunction(void *args) {
    // TODO: run light function then exit thread when execution is complete or perform waiting procedure if necessary

    uint32_t time_step;
    uint8_t steps;


    // determine timing for color shifting
    if (pattern.light_show_flags & FINITE_DURATION && pattern.light_show_flags & COLOR_SHIFT) {
        steps = maxColorDiff(pattern.start_color, pattern.end_color);
        time_step = pattern.duration / steps;

    }

    // change color if specified to change before delay is executed
    if (!(pattern.light_show_flags & SHIFT_POST_DELAY)) {
        for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
            WS2812_setPixelColor(loc_u16_pixelIndex, pattern.start_color.r, pattern.start_color.g, pattern.start_color.b);
        }
        WS2812_show();
    }

    // busy wait to execute delay
    if (pattern.light_show_flags & DO_DELAY) {
        busyWait(pattern.delay);
    }

    // execute color shift
    // TODO: convert to use HSV instead of RGB
    if (pattern.light_show_flags & COLOR_SHIFT) {
        uint8_t i;
        //Use HSV instead of RGB
        HSV startHSV = RGBtoHSV(pattern.start_color.r, pattern.start_color.g, pattern.start_color.b);
        HSV endHSV = RGBtoHSV(pattern.end_color.r, pattern.end_color.g, pattern.end_color.b);
        for (i = 0; i < steps; i++) {
            for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
                float h;
                float s;
                float v;

                if (startHSV.h > endHSV.h)
                    h = startHSV.h - (startHSV.h - endHSV.h) * i / steps;
                else {
                    h = startHSV.h - (endHSV.h - startHSV.h) * i / steps;
                }
                if (startHSV.s > endHSV.s)
                    s = startHSV.s - (startHSV.s - endHSV.s) * i / steps;
                else {
                    s = startHSV.s- (endHSV.s - startHSV.s) * i / steps;
                }
                if (startHSV.v > endHSV.v)
                    v = startHSV.v - (startHSV.v - endHSV.v) * i / steps;
                else {
                    v = startHSV.v - (endHSV.v - startHSV.v) * i / steps;
                }
                RGB ret = hsvToRgb(h, s, v);
                WS2812_setPixelColor(loc_u16_pixelIndex, ret.r, ret.g, ret.b);
            }
            WS2812_show();
            busyWait(time_step);
        }
    } else {
        for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
            WS2812_setPixelColor(loc_u16_pixelIndex, pattern.end_color.r, pattern.end_color.g, pattern.end_color.b);
        }
        WS2812_show();
    }

    //TODO: TIMEOUTS, FINITE DURATION, DO TIMEOUT, 
}


/*
 * Floating point conversion of hsv to rgb
 * */

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

/*
 * Floating point conversion of rgb to hsv
 * */
float max3(float a, float b, float c) {
   return ((a > b)? (a > c ? a : c) : (b > c ? b : c));
}
float min3(float a, float b, float c) {
   return ((a < b)? (a < c ? a : c) : (b < c ? b : c));
}
HSV RGBtoHSV(float r, float g, float b) {
   HSV hsv;
   float h, s, v;
   r /= 255.0;
   g /= 255.0;
   b /= 255.0;
   int cmax = max3(r, g, b); // maximum of r, g, b
   int cmin = min3(r, g, b); // minimum of r, g, b
   int diff = cmax-cmin; // diff of cmax and cmin.
   if (cmax == cmin)
      hsv.h = 0;
   else if (cmax == r)
      hsv.h = fmod((60 * ((g - b) / diff) + 360), 360.0);
   else if (cmax == g)
      hsv.h = fmod((60 * ((b - r) / diff) + 120), 360.0);
   else if (cmax == b)
      hsv.h = fmod((60 * ((r - g) / diff) + 240), 360.0);
   // if cmax equal zero
      if (cmax == 0)
         hsv.s = 0;
      else
         hsv.s = (diff / cmax) * 100;
   // compute v
   hsv.v = cmax * 100;
   
   return hsv;
}

