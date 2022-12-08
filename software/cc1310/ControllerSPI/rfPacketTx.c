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
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/UART.h>

/* Display Stuff */
#include <ti/display/Display.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"
#include "smartrf_settings/smartrf_settings.h"

/***** Defines *****/
#include "WS2812.h"
#include "patterns.h"

/* Do power measurement */
//#define POWER_MEASUREMENT

/* Packet TX Configuration */
#define PAYLOAD_LENGTH      135
#ifdef POWER_MEASUREMENT
#define PACKET_INTERVAL     5  /* For power measurement set packet interval to 5s */
#else
#define PACKET_INTERVAL     500000  /* Set packet interval to 500000us or 500ms */
#endif

/* Pattern List */
#define ARRAY_SIZE(A) (sizeof(A) / sizeof((A)[0]))
typedef void (*SimplePatternList[])();
SimplePatternList gPatterns = {allBlue, allRed, allGreen, chirstLights};
uint8_t gCurrentPatternNumber = 0; // Index number of which pattern is current

/* Pattern Array */
uint8_t SendArr[NB_PIXELS * 3] = {0};

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;

static uint8_t packet[PAYLOAD_LENGTH];

/* UART driver handles */
char        input = 0;
const char  echoPrompt[] = "Press any key to update pattern. \r\n\n";
const char  echoUpdate[] = "Update Received! Changing pattern.\r\n";

UART_Handle uart;
UART_Params uartParams;


/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#ifdef POWER_MEASUREMENT
#if defined(Board_CC1350_LAUNCHXL)
    Board_DIO30_SWPWR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
#endif
    PIN_TERMINATE
};

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }

    UART_init();

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }

#ifdef POWER_MEASUREMENT
#if defined(Board_CC1350_LAUNCHXL)
    /* Route out PA active pin to Board_DIO30_SWPWR */
    PINCC26XX_setMux(ledPinHandle, Board_DIO30_SWPWR, PINCC26XX_MUX_RFC_GPO1);
#endif
#endif

    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    UART_write(uart, echoPrompt, sizeof(echoPrompt));

    PerformanceMode();

    return NULL;
}

void sendControlPacket()
{
    gPatterns[gCurrentPatternNumber](SendArr);
    uint8_t i;
    for (i = 0; i < PAYLOAD_LENGTH; i++)
    {
        packet[i] = SendArr[i];
    }

    RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
    RF_cmdPropTx.pPkt = packet;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    /* Send packet */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*) &RF_cmdPropTx,
                                               RF_PriorityNormal, NULL, 0);

    switch (terminationReason)
    {
    case RF_EventLastCmdDone:
        // A stand-alone radio operation command or the last radio
        // operation command in a chain finished.
        break;
    case RF_EventCmdCancelled:
        // Command cancelled before it was started; it can be caused
        // by RF_cancelCmd() or RF_flushCmd().
        break;
    case RF_EventCmdAborted:
        // Abrupt command termination caused by RF_cancelCmd() or
        // RF_flushCmd().
        break;
    case RF_EventCmdStopped:
        // Graceful command termination caused by RF_cancelCmd() or
        // RF_flushCmd().
        break;
    default:
        // Uncaught error event
        while (1)
            ;
    }

    uint32_t cmdStatus = ((volatile RF_Op*) &RF_cmdPropTx)->status;
    switch (cmdStatus)
    {
    case PROP_DONE_OK:
        // Packet transmitted successfully
        break;
    case PROP_DONE_STOPPED:
        // received CMD_STOP while transmitting packet and finished
        // transmitting packet
        break;
    case PROP_DONE_ABORT:
        // Received CMD_ABORT while transmitting packet
        break;
    case PROP_ERROR_PAR:
        // Observed illegal parameter
        break;
    case PROP_ERROR_NO_SETUP:
        // Command sent without setting up the radio in a supported
        // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
        break;
    case PROP_ERROR_NO_FS:
        // Command sent without the synthesizer being programmed
        break;
    case PROP_ERROR_TXUNF:
        // TX underflow observed during operation
        break;
    default:
        // Uncaught error event - these could come from the
        // pool of states defined in rf_mailbox.h
        while (1);
    }
}

void nextPattern()
{
  // add one to the current pattern number, and wrap around at the end
  gCurrentPatternNumber = (gCurrentPatternNumber + 1) % ARRAY_SIZE( gPatterns);
}

void PerformanceMode()
{
    while(1)
    {

        sendControlPacket();

        UART_write(uart, echoUpdate, sizeof(echoUpdate));
        UART_read(uart, &input, 1);

        if(input){
            ProgrammingMode();
        }
    }
}

void ProgrammingMode()
{


#ifndef POWER_MEASUREMENT
            PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,!PIN_getOutputValue(Board_PIN_LED1));
#endif
            /* Power down the radio */
            RF_yield(rfHandle);

#ifdef POWER_MEASUREMENT
            /* Sleep for PACKET_INTERVAL s */
            sleep(PACKET_INTERVAL);
#else
            /* Sleep for PACKET_INTERVAL us */
//          usleep(PACKET_INTERVAL);
//          usleep(PACKET_INTERVAL);
#endif

            //Switch Patterns
            nextPattern();
            input = 0;
}

