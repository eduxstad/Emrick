/*
 * Copyright (c) 2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/display/Display.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"
#include "WS2812.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"

/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */

//TODO : ADD limitation on NB_PIXELS
#ifndef NB_PIXELS
#define NB_PIXELS 46U
#endif

#define NB_SPI_BYTES_PER_PIXEL 9U

/** Get SPI value corresponding to a bit at index n in a grb color on 24 bits
 *  1 bit is 0b110
 *  0 bit is 0b100
 */
#define GRB_BIT_TO_SPI_BITS(val, bitPos) ((1 << bitPos & val) ? 0x06 : 0x04)

static Display_Handle display;

SPI_Handle      masterSpi;
SPI_Params      spiParams;
uint16_t        colorIndex = 0;

static uint8_t _au8_spiLedBuffer[NB_SPI_BYTES_PER_PIXEL*NB_PIXELS] = {0};
const uint8_t HSVlights[61] =
{0, 4, 8, 13, 17, 21, 25, 30, 34, 38, 42, 47, 51, 55, 59, 64, 68, 72, 76,
81, 85, 89, 93, 98, 102, 106, 110, 115, 119, 123, 127, 132, 136, 140, 144,
149, 153, 157, 161, 166, 170, 174, 178, 183, 187, 191, 195, 200, 204, 208,
212, 217, 221, 225, 229, 234, 238, 242, 246, 251, 255};


#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>

/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)]
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;


static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

/***** Function definitions *****/
sem_t masterSem;

void delay(int number_of_seconds)
{
    // Converting time into milli_seconds
    int milli_seconds = 1000 * number_of_seconds;

    // Storing start time
    clock_t start_time = clock();

    // looping till required time is not achieved
    while (clock() < start_time + milli_seconds)
        ;
}

void trueHSV(int ind, int LED, int angle)
{
  uint8_t red, green, blue;

  if (angle<60) {red = 255; green = HSVlights[angle]; blue = 0;} else
  if (angle<120) {red = HSVlights[120-angle]; green = 255; blue = 0;} else
  if (angle<180) {red = 0, green = 255; blue = HSVlights[angle-120];} else
  if (angle<240) {red = 0, green = HSVlights[240-angle]; blue = 255;} else
  if (angle<300) {red = HSVlights[angle-240], green = 0; blue = 255;} else
                 {red = 255, green = 0; blue = HSVlights[360-angle];}
  WS2812_setPixelColor(ind, red, green, blue);
}



void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();



    GPIO_setConfig(Board_SPI_MASTER_READY, GPIO_CFG_OUTPUT | GPIO_CFG_OUT_LOW);
    GPIO_write(Board_SPI_MASTER_READY, 1);

    SPI_Params_init(&spiParams);
    spiParams.frameFormat = SPI_POL0_PHA1;
    spiParams.bitRate = 2400000;
    masterSpi = SPI_open(Board_SPI_MASTER, &spiParams);
    if (masterSpi == NULL) {
        Display_printf(display, 0, 0, "Error initializing master SPI\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "Master SPI initialized\n");
    }

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                               RF_PriorityNormal, &callback,
                                               RF_EventRxEntryDone);

//    switch(terminationReason)
//    {
//        case RF_EventLastCmdDone:
//            // A stand-alone radio operation command or the last radio
//            // operation command in a chain finished.
//            break;
//        case RF_EventCmdCancelled:
//            // Command cancelled before it was started; it can be caused
//            // by RF_cancelCmd() or RF_flushCmd().
//            break;
//        case RF_EventCmdAborted:
//            // Abrupt command termination caused by RF_cancelCmd() or
//            // RF_flushCmd().
//            break;
//        case RF_EventCmdStopped:
//            // Graceful command termination caused by RF_cancelCmd() or
//            // RF_flushCmd().
//            break;
//        default:
//            // Uncaught error event
//            while(1);
//    }
//
//    uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;
//    switch(cmdStatus)
//    {
//        case PROP_DONE_OK:
//            // Packet received with CRC OK
//            break;
//        case PROP_DONE_RXERR:
//            // Packet received with CRC error
//            break;
//        case PROP_DONE_RXTIMEOUT:
//            // Observed end trigger while in sync search
//            break;
//        case PROP_DONE_BREAK:
//            // Observed end trigger while receiving packet when the command is
//            // configured with endType set to 1
//            break;
//        case PROP_DONE_ENDED:
//            // Received packet after having observed the end trigger; if the
//            // command is configured with endType set to 0, the end trigger
//            // will not terminate an ongoing reception
//            break;
//        case PROP_DONE_STOPPED:
//            // received CMD_STOP after command started and, if sync found,
//            // packet is received
//            break;
//        case PROP_DONE_ABORT:
//            // Received CMD_ABORT after command started
//            break;
//        case PROP_ERROR_RXBUF:
//            // No RX buffer large enough for the received data available at
//            // the start of a packet
//            break;
//        case PROP_ERROR_RXFULL:
//            // Out of RX buffer space during reception in a partial read
//            break;
//        case PROP_ERROR_PAR:
//            // Observed illegal parameter
//            break;
//        case PROP_ERROR_NO_SETUP:
//            // Command sent without setting up the radio in a supported
//            // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
//            break;
//        case PROP_ERROR_NO_FS:
//            // Command sent without the synthesizer being programmed
//            break;
//        case PROP_ERROR_RXOVF:
//            // RX overflow observed during operation
//            break;
//        default:
//            // Uncaught error event - these could come from the
//            // pool of states defined in rf_mailbox.h
//            while(1);
//    }
//
//    while(1);
    return (NULL);
}


void WS2812_setPixelColor(uint16_t arg_u16_ledIndex, uint8_t arg_u8_red, uint8_t arg_u8_green, uint8_t arg_u8_blue)
{
    uint8_t loc_u8_currIndex = 3;

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


void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    SPI_Transaction transaction;
    uint16_t loc_u16_pixelIndex;

    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                           !PIN_getOutputValue(Board_PIN_LED2));



        switch(colorIndex)
        {
        case(0):
            for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
            {
                WS2812_setPixelColor(loc_u16_pixelIndex, 0, 0, 0XFF);
                //trueHSV(loc_u16_pixelIndex, loc_u16_pixelIndex, i);
            }
            transaction.count = sizeof(_au8_spiLedBuffer);
            transaction.txBuf = _au8_spiLedBuffer;
            transaction.rxBuf = NULL;

            SPI_transfer(masterSpi, &transaction);
            colorIndex++;
            break;
        case(1):
            for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
            {
                WS2812_setPixelColor(loc_u16_pixelIndex, 0, 0xFF, 0x00);
                //trueHSV(loc_u16_pixelIndex, loc_u16_pixelIndex, i);
            }
            transaction.count = sizeof(_au8_spiLedBuffer);
            transaction.txBuf = _au8_spiLedBuffer;
            transaction.rxBuf = NULL;

            SPI_transfer(masterSpi, &transaction);
            colorIndex++;
            break;
        case(2):
            for(loc_u16_pixelIndex = 0; loc_u16_pixelIndex < NB_PIXELS; loc_u16_pixelIndex++)
            {
                WS2812_setPixelColor(loc_u16_pixelIndex, 0xFF, 0x00, 0x00);
                //trueHSV(loc_u16_pixelIndex, loc_u16_pixelIndex, i);
            }
            transaction.count = sizeof(_au8_spiLedBuffer);
            transaction.txBuf = _au8_spiLedBuffer;
            transaction.rxBuf = NULL;

            SPI_transfer(masterSpi, &transaction);
            colorIndex = 0;
            break;
        }

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));

        RFQueue_nextEntry();
    }
}
