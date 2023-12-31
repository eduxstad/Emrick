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

#define STALL_PATTERN               0x001
#define SET_TIMEOUT                 0x002
#define BLANK_ON_TIMEOUT            0x004
#define COLOR_SHIFT                 0x008
#define DO_DELAY                    0x010
#define FINITE_DURATION             0X020
#define DEFAULT_FUNCTION            0x040
#define SHIFT_POST_DELAY            0x080
#define TOGGLE_PROGRAMMING_MODE     0X100

/**************************************************************************
 * Type Definitions
 **************************************************************************/
typedef struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;

typedef struct control {
    uint16_t        light_show_flags;
    RGB             start_color;
    RGB             end_color;
    uint16_t        delay;
    uint16_t        duration;
    uint8_t         timeout;
} control;
/**************************************************************************
 * Global variables
 **************************************************************************/
uint16_t testFlag;
uint16_t timer;
pthread_mutex_t LEDMutex;
control receive_control;

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

void runLED(void);
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
