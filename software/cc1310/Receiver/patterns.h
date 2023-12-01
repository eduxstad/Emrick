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

/**************************************************************************
 * Type Definitions
 **************************************************************************/
typedef struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;
/**************************************************************************
 * Global variables
 **************************************************************************/
uint16_t testFlag;
uint16_t timer;
pthread_mutex_t LEDMutex;


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
