/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

#define STRIP_ID                    0
#define UNIVERSAL_ID                0xffff  //used to match any light strip

/*
#define STALL_PATTERN               0x001
#define SET_TIMEOUT                 0x002
#define BLANK_ON_TIMEOUT            0x004 //Nope
#define COLOR_SHIFT                 0x008
#define DO_DELAY                    0x010
#define FINITE_DURATION             0X020
#define DEFAULT_FUNCTION            0x040
#define SHIFT_POST_DELAY            0x080
#define TOGGLE_PROGRAMMING_MODE     0X100
*/

#define DEFAULT_FUNCTION            0x001 //Use default light function
#define TIME_GRADIENT               0x040 //gradually shift color over set time
#define SET_TIMEOUT                 0x042 //set a timer to timeout after pattern
#define DO_DELAY                    0x044 //add a delay before execution
#define INSTANT_COLOR               0x054 //change color before delay is executed

/**************************************************************************
 * Type Definitions
 **************************************************************************/
typedef struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;

typedef struct HSV {
    float h;
    float s;
    float v;
} HSV;

typedef struct control {
    uint8_t         size;
    uint16_t        light_show_flags;
    RGB             start_color;
    RGB             end_color;
    uint32_t        delay;
    uint16_t        duration;
    uint8_t         timeout;
} control;
/**************************************************************************
 * Global variables
 **************************************************************************/
uint16_t testFlag;
uint16_t timer;
pthread_mutex_t LEDMutex;
control pattern;
control rec_pattern;

/* Function Selector */
uint8_t function_flag;

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Global Functions Declarations
 **************************************************************************/
/**
 * Initialize WS2812 driver with data line on arg_u8_pin
 * SPI used will be SPI at index arg_u8_spiId in your Board.c
 */

void runLED(Display_Handle displayHandle);
void trueHSV(int angle, int * red, int * green, int * blue);
void ResetLights(void);
void chirstLights(void);
void rainbow(void);
void allWhite(void);
void allBlue(void);
void allRed(void);
void allGreen(void);
void nextPattern(void);
void rainbowAnimation(void);
void rainbowGradient(void);
void CandyCane();
void XmasShift(int marcher);
void XmasPulse(int marcher);
void SinglePulse();
void rainbowGradientHSV(Display_Handle displayHandle);
void lightFunction(int marcher);
