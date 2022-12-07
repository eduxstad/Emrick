/******************************************************************************
 * @file    WS2812.h
 * @author  Rémi Pincent - INRIA
 * @date    10/04/2018
 *
 * @brief Neopixel WS2812 driver - http://cdn.sparkfun.com/datasheets/Components/LED/WS2812.pdf
 *
 * Project : cc26xx_neopixel
 * Contact:  Rémi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * refer https://github.com/Lahorde/cc26xx_neopixel.git
 *
 * LICENSE :
 * cc26xx_neopixel (c) by Rémi Pincent
 * cc26xx_neopixel is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

#ifndef WS2812__include
#define WS2812__include

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 * Include Files
 **************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/**************************************************************************
 * Manifest Constants
 **************************************************************************/

//TODO : ADD limitation on NB_PIXELS
#ifndef NB_PIXELS
#define NB_PIXELS 45U
#endif

#define NB_SPI_BYTES_PER_PIXEL 9U

/** Get SPI value corresponding to a bit at index n in a grb color on 24 bits
 *  1 bit is 0b110
 *  0 bit is 0b100
 */
#define GRB_BIT_TO_SPI_BITS(val, bitPos) ((1 << bitPos & val) ? 0x06 : 0x04)

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
extern void WS2812_beginSPI(void);
extern void WS2812_close(void);
extern bool WS2812_show(void);
extern void WS2812_setPixelColor(uint16_t arg_u16_ledIndex, uint8_t arg_u8_red, uint8_t arg_u8_green, uint8_t arg_u8_blue);


void sendControlPacket(); //Control Packet declaration

#ifdef __cplusplus
}
#endif

#endif /* WS2812__include */
