/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/
#define NUM_PATTERNS 10

/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Global variables
 **************************************************************************/
 

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
