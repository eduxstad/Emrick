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

// TODO: write RGB to HSV function

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

    receive_control.light_show_flags |= STALL_PATTERN;
    receive_control.light_show_flags |= SET_TIMEOUT;
    receive_control.light_show_flags |= COLOR_SHIFT;
    receive_control.light_show_flags |= FINITE_DURATION;
    receive_control.light_show_flags |= DEFAULT_FUNCTION;
    receive_control.light_show_flags |= DO_DELAY;
    receive_control.light_show_flags |= SHIFT_POST_DELAY;
    receive_control.delay = 1000;
    receive_control.duration = 5000;
    receive_control.timeout = 20;
    receive_control.start_color = red;
    receive_control.end_color = blue;


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
 * Runs LED pattern specified by receive_control structure
 * */

void *defaultLEDFunction(void *args) {
    // TODO: run light function then exit thread when execution is complete or perform waiting procedure if necessary

    uint32_t time_step;
    uint8_t steps;


    // determine timing for color shifting
    if (receive_control.light_show_flags & FINITE_DURATION && receive_control.light_show_flags & COLOR_SHIFT) {
        steps = maxColorDiff(receive_control.start_color, receive_control.end_color);
        time_step = receive_control.duration * 1000 / steps;

    }

    // change color if specified to change before delay is executed
    if (!(receive_control.light_show_flags & SHIFT_POST_DELAY)) {
        for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
            WS2812_setPixelColor(loc_u16_pixelIndex, receive_control.start_color.r, receive_control.start_color.g, receive_control.start_color.b);
        }
        WS2812_show();
    }

    // busy wait to execute delay
    if (receive_control.light_show_flags & DO_DELAY) {
        struct timespec ts_start;
        clock_gettime(CLOCK_MONOTONIC, &ts_start);
        struct timespec ts_curr;
        clock_gettime(CLOCK_MONOTONIC, &ts_curr);
        while (mstime(ts_curr) < mstime(ts_start) + receive_control.delay) {
            if (function_flag == 0xff) {
                function_flag == 0x00;
                pthread_exit(status);
            }
            clock_gettime(CLOCK_MONOTONIC, &ts_curr);
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,"%d %d", ts_start.tv_nsec, ts_curr.tv_nsec);
        }
    }

    // execute color shift
    // TODO: convert to use HSV instead of RGB
    if (receive_control.light_show_flags & COLOR_SHIFT) {
        uint8_t i;
        for (i = 0; i < steps; i++) {
            for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
                uint8_t r;
                uint8_t g;
                uint8_t b;
                if (receive_control.start_color.r > receive_control.end_color.r)
                    r = receive_control.start_color.r - diff(receive_control.start_color.r, receive_control.end_color.r) * i / steps;
                else {
                    r = receive_control.start_color.r + diff(receive_control.start_color.r, receive_control.end_color.r) * i / steps;
                }
                if (receive_control.start_color.g > receive_control.end_color.g)
                    g = receive_control.start_color.g - diff(receive_control.start_color.g, receive_control.end_color.g) * i / steps;
                else {
                    g = receive_control.start_color.g + diff(receive_control.start_color.g, receive_control.end_color.g) * i / steps;
                }
                if (receive_control.start_color.b > receive_control.end_color.b)
                    b = receive_control.start_color.b - diff(receive_control.start_color.b, receive_control.end_color.b) * i / steps;
                else {
                    b = receive_control.start_color.b + diff(receive_control.start_color.b, receive_control.end_color.b) * i / steps;
                }
                WS2812_setPixelColor(loc_u16_pixelIndex, r, g, b);
            }
            WS2812_show();
            usleep(time_step);
        }
    }

    for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0, 0, 0);
    }
    WS2812_show();

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

