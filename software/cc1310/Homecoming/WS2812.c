/******************************************************************************
 * @file    WS2812.c
 * @author  R�mi Pincent - INRIA
 * @date    10/04/2016
 *
 * @brief WS2812 driver implementation on Texas Instruments CC26xx.
 * First implementation using UDMA + GPT configured as PWM was used
 * but timing was not precise / UDMA too long :
 *   * https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/541648
 *   * https://e2e.ti.com/support/wireless_connectivity/bluetooth_low_energy/f/538/t/542169
 *
 * So this implementation uses SPI configured at 3 x Neopixel Freq = 2.4MHz.
 * In this case a bit duration = 1/2.4MHz = 416ns. As 3 bits can be transferred in 1/NeopixelFreq
 * we can get a precise signal with :
 *   * 1 bit to 1 during 416ns and 2 other bits to 0. In this case a 1 bit is transferred to WS2812. We follow datasheet reqs
 *  350ns - 150ns < T0H = 416ns < 350ns + 150ns
 *  800ns - 150ns < T0L = 1250ns - T0H = 834 ns < 800ns + 150ns
 *
 *   * 2 bits to 1 during 833ns and 1 other bit to 0. In this case a 1 bit is transferred to WS2812. We do not follow datasheet reqs for T1L but
 *   after testing it works
 *  700ns - 150ns < T1H = 833ns < 700ns + 150ns
 *  600ns - 150ns < T1L = 1250ns - T1H = 417 ns < 600ns + 150ns
 *
 * 24 bit (GRB) color must be send for each neopixel => a buffer of 3x24bit = 72 bits = 9 bytes must be send to SPI for each neopixel.
 *
 * SPI SPH must be set in order to have back to back transfers like single word transfer.
 *
 * In Board.c you can assign only 1 pin to SPI MOSI (Neopixel data) and let other unsassigned.
 *
 * Project : WS2812_driver
 * Contact:  R�mi Pincent - remi.pincent@inria.fr
 *
 * Revision History:
 * refer https://github.com/Lahorde/cc26xx_neopixel.git
 *
 * LICENSE :
 * cc26xx_neopixel (c) by R�mi Pincent
 * cc26xx_neopixel is licensed under a
 * Creative Commons Attribution-NonCommercial 3.0 Unported License.
 *
 * You should have received a copy of the license along with this
 * work.  If not, see <http://creativecommons.org/licenses/by-nc/3.0/>.
 *****************************************************************************/

/**************************************************************************
 * Include Files
 **************************************************************************/
#include "WS2812.h"
#include "Board.h"

#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>


/**************************************************************************
 * Type Definitions
 **************************************************************************/

/**************************************************************************
 * Variables
 **************************************************************************/
/** buffer written to SPI */
static uint8_t _au8_spiLedBuffer[NB_SPI_BYTES_PER_PIXEL*NB_PIXELS] = {0};
SPI_Handle      masterSpi;
SPI_Params      spiParams;


/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Local Functions Declarations
 **************************************************************************/

/**************************************************************************
 * Global Functions Defintions
 **************************************************************************/

void WS2812_beginSPI(void)
{
    SPI_Params      spiParams;
    uint16_t loc_u16_pixelIndex;


    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate = 2420000;
    masterSpi = SPI_open(CC1310_LAUNCHXL_SPI1, &spiParams);


    //This is needed
    /** Put all led to 0 */
    for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
    {
        WS2812_setPixelColor(loc_u16_pixelIndex, 0, 0, 0);
    }
 //   WS2812_show();
}

void WS2812_close(void)
{
    SPI_close(masterSpi);
}

bool WS2812_show(void)
{
    /** Make SPI transfer */
    SPI_Transaction transaction;

    transaction.count = sizeof(_au8_spiLedBuffer);
    transaction.txBuf = _au8_spiLedBuffer;
    transaction.rxBuf = NULL;

    return SPI_transfer(masterSpi, &transaction);
}

void WS2812_setPin(uint8_t p)
{

}

void WS2812_setPixelColor(uint16_t arg_u16_ledIndex, uint8_t arg_u8_red, uint8_t arg_u8_green, uint8_t arg_u8_blue)
{
    uint8_t loc_u8_currIndex = 3;
    /* Divide by 3 to prevent over current */
    arg_u8_red = arg_u8_red/3;
    arg_u8_green = arg_u8_green/3;
    arg_u8_blue = arg_u8_blue/3;

    /** Position of current led data in SPI buffer */
    uint16_t loc_u16_ledOffset = arg_u16_ledIndex*9;

    /** Concatenate color on a 32bit word */
    uint32_t loc_u32_grb = arg_u8_green << 16 | arg_u8_red << 8 | arg_u8_blue;

    /** Concatenate two bytes of SPI buffer in order to always transfer blocks of 3 bits
     * to SPI buffer corresponding to a single grb bit*/
    uint16_t loc_u16_currVal = 0;

    int8_t loc_u8_bitIndex;

    for(loc_u8_bitIndex = 23; loc_u8_bitIndex >= 0; loc_u8_bitIndex--)
    {
        loc_u16_currVal |= GRB_BIT_TO_SPI_BITS(loc_u32_grb, loc_u8_bitIndex) << (16 + 8*((loc_u8_currIndex - 3)/8) - loc_u8_currIndex);

        if((loc_u8_currIndex)/8 > (loc_u8_currIndex-3)/8) /** some bits have been written to byte at index 1 in  loc_u16_currVal*/
        {
            /** it's time to shift buffers */
            _au8_spiLedBuffer[loc_u16_ledOffset + loc_u8_currIndex /8 - 1] = loc_u16_currVal >> 8;
            loc_u16_currVal = loc_u16_currVal << 8;
        }
        loc_u8_currIndex += 3;
    }
}
