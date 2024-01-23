/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/drivers/NVS.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

#define STRIP_ID                    0
#define UNIVERSAL_ID                0xffff  //used to match any light strip

#define STALL_PATTERN               0x001
#define SET_TIMEOUT                 0x002
#define BLANK_ON_TIMEOUT            0x004
#define COLOR_SHIFT                 0x008
#define DO_DELAY                    0x010
#define FINITE_DURATION             0X020
#define DEFAULT_FUNCTION            0x040
#define SHIFT_POST_DELAY            0x080
#define TOGGLE_PROGRAMMING_MODE     0X100

#define PACKET_SIZE                 16

/**************************************************************************
 * Type Definitions
 **************************************************************************/
typedef struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGB;

typedef struct control {
    uint8_t         size;
    uint16_t        light_show_flags;
    RGB             start_color;
    RGB             end_color;
    uint32_t        delay;
    uint16_t        duration;
    uint8_t         timeout;
} control;

typedef struct show_metadata {
    uint32_t        total_size;
    uint16_t        pattern_count;
    uint8_t         set_count;
} show_metadata;
/**************************************************************************
 * Global variables
 **************************************************************************/
uint16_t testFlag;
uint16_t timer;
pthread_mutex_t recMutex;
control pattern;
control rec_pattern;
uint8_t programming_mode;
uint8_t current_set;

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
