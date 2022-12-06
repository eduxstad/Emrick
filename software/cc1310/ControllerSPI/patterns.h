/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

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
void chirstLights(uint8_t sendArr[]);
void rainbow(void);
void allWhite(uint8_t sendArr[]);
void allBlue(uint8_t sendArr[]);
void allRed(uint8_t sendArr[]);
void allGreen(uint8_t sendArr[]);
void nextPattern(void);
void animationBasic(void);
void rainbowAnimation(void);
void bounce(void);

