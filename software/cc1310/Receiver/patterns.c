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


void controlToChar(control control, char * ch) {
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

void metadataToChar(show_metadata sm, char * ch) {
    ch[0] = sm.total_size >> 24 & 0xFF;
    ch[1] = sm.total_size >> 16 & 0xFF;
    ch[2] = sm.total_size >> 8 & 0xFF;
    ch[3] = sm.total_size & 0xFF;
    ch[4] = sm.pattern_count >> 8 & 0xFF;
    ch[5] = sm.pattern_count & 0xFF;
    ch[6] = sm.set_count;
}

show_metadata charToMetadata(char * ch) {
    show_metadata sm;
    sm.total_size = ch[0];
    sm.total_size = sm.total_size << 8;
    sm.total_size += ch[1];
    sm.total_size = sm.total_size << 8;
    sm.total_size += ch[2];
    sm.total_size = sm.total_size << 8;
    sm.total_size += ch[3];
    sm.pattern_count = ch[4];
    sm.pattern_count = sm.pattern_count << 8;
    sm.pattern_count += ch[5];
    sm.set_count = ch[6];
    return sm;
}

/* Contains control logic for running and switching LED patterns */
void runLED(Display_Handle dh) {

    // TODO: Check if the board is being charged before turning on the LED
    displayHandle = dh;
    // Turn on power by enabling the 5v supply
    GPIO_setConfig(Board_GPIO_BOOST_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
    SPI_init();
    WS2812_beginSPI();

    programming_mode = 0;


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

    //default_pattern.light_show_flags |= STALL_PATTERN;
    //default_pattern.light_show_flags |= SET_TIMEOUT;
    default_pattern.light_show_flags |= COLOR_SHIFT;
    default_pattern.light_show_flags |= FINITE_DURATION;
    default_pattern.light_show_flags |= DEFAULT_FUNCTION;
    default_pattern.light_show_flags |= DO_DELAY;
    //default_pattern.light_show_flags |= SHIFT_POST_DELAY;
    default_pattern.delay = 5000;
    default_pattern.duration = 5000;
    //default_pattern.timeout = 20;
    default_pattern.start_color = red;
    default_pattern.end_color = blue;
    default_pattern.size = 16;


    NVS_Handle nvsRegion;
    NVS_Attrs regionAttrs;

    show_metadata show_metadata;
    char buf[32];
    current_set = 0;
    uint8_t last_set = 0;
    uint16_t pattern_index = 0;

    while(1) {
        if (programming_mode) {
            if (programming_mode == 0x01) {
                nvsRegion = NVS_open(Board_NVSINTERNAL, NULL);
                NVS_getAttrs(nvsRegion, &regionAttrs);
                NVS_erase(nvsRegion, 0, regionAttrs.regionSize);
                programming_mode = 0xFF;
                show_metadata.set_count = 0;
                show_metadata.pattern_count = 0;
                show_metadata.total_size = sizeof(show_metadata);
            }
            if (function_flag == 0xFF) {
                pthread_mutex_lock(&recMutex);
                show_metadata.total_size += PACKET_SIZE;
                size_t offset = regionAttrs.regionSize - (show_metadata.pattern_count) * PACKET_SIZE;
                if (current_set > last_set) {
                    last_set = current_set;
                    show_metadata.set_count++;
                    buf[0] = offset >> 24 & 0xFF;
                    buf[1] = offset >> 16 & 0xFF;
                    buf[2] = offset  >> 8 & 0xFF;
                    buf[3] = offset & 0xFF;
                    NVS_write(nvsRegion, sizeof(show_metadata) + (show_metadata.set_count - 1) * 4, buf, 4, NVS_WRITE_POST_VERIFY);
                }
                metadataToChar(show_metadata, buf);
                NVS_write(nvsRegion, 0, buf, sizeof(show_metadata), NVS_WRITE_POST_VERIFY);
                controlToChar(rec_pattern, buf);
                NVS_write(nvsRegion, offset, buf, PACKET_SIZE, NVS_WRITE_POST_VERIFY);
                rec_pattern.size = NULL;
                pthread_mutex_unlock(&recMutex);
            }

        } else {
            if (current_set == 0) {
                pattern = default_pattern;
            } else {
                if (current_set != last_set) {
                    last_set = current_set;
                    pattern_index = 0;
                }
                size_t offset = sizeof(show_metadata) + (current_set - 1) * 4;
                NVS_read(nvsRegion, offset, buf, 4);
                offset = buf[0];
                offset = offset << 8;
                offset += buf[1];
                offset = offset << 8;
                offset += buf[2];
                offset = offset << 8;
                offset += buf[3];
                offset -= pattern_index * PACKET_SIZE;
                NVS_read(nvsRegion, offset, buf, PACKET_SIZE);
                pattern = charToControl(buf);
                pattern_index++;
            }
            pthread_create(&thread, &attrs, defaultLEDFunction, NULL);
            pthread_join(thread, &status);
        }
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
        for (i = 0; i < steps; i++) {
            for (loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++) {
                uint8_t r;
                uint8_t g;
                uint8_t b;
                if (pattern.start_color.r > pattern.end_color.r)
                    r = pattern.start_color.r - diff(pattern.start_color.r, pattern.end_color.r) * i / steps;
                else {
                    r = pattern.start_color.r + diff(pattern.start_color.r, pattern.end_color.r) * i / steps;
                }
                if (pattern.start_color.g > pattern.end_color.g)
                    g = pattern.start_color.g - diff(pattern.start_color.g, pattern.end_color.g) * i / steps;
                else {
                    g = pattern.start_color.g + diff(pattern.start_color.g, pattern.end_color.g) * i / steps;
                }
                if (pattern.start_color.b > pattern.end_color.b)
                    b = pattern.start_color.b - diff(pattern.start_color.b, pattern.end_color.b) * i / steps;
                else {
                    b = pattern.start_color.b + diff(pattern.start_color.b, pattern.end_color.b) * i / steps;
                }
                WS2812_setPixelColor(loc_u16_pixelIndex, r, g, b);
            }
            WS2812_show();
            busyWait(time_step);
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

