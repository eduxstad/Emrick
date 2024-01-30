/*
 * Copyright (c) 2018-2019, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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


/*
 *  ======== spimaster.c ========
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <xdc/std.h>
#include <unistd.h>
#include <math.h>

/* POSIX Header files */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* Battery Monitor */
#include <ti/devices/cc13x0/driverlib/aon_batmon.h>

/* ADC */
#include <ti/drivers/ADC.h>

/* Display Header files */
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include <ti/display/DisplayExt.h>
#include <ti/display/AnsiColor.h>

/* RF */
#include <ti/drivers/rf/RF.h>
#include "smartrf_settings/smartrf_settings.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Example/Board Header files */
#include "Board.h"
#include "WS2812.h"
#include "patterns.h"

#define THREADSTACKSIZE (1024)

/***** Variable declarations *****/

/* RF */
#define PAYLOAD_LENGTH      20
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint32_t packet[PAYLOAD_LENGTH];

/* ADC conversion result variables */
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;


/**************************************************************************
 * Manifest Constants
 **************************************************************************/

/**************************************************************************
 * Macros
 **************************************************************************/

/**************************************************************************
 * Local Functions Declarations
 **************************************************************************/

/**************************************************************************
 * Global Functions Defintions
 **************************************************************************/

void *trackBattery(void *arg0)
{
    /* Battery Monitor */
    uint32_t bat_measure;
    float bat_voltage;

    /* time (in seconds) delay */
    const uint32_t LONG_DELAY = 2;
    const uint32_t SHORT_DELAY = 2;
    uint32_t time = 0;
    uint32_t delay_seconds;

    /* Initialize UART display. */
    /* So we can use Display_printf, a nice function */

    // avoid races? (need a proper semaphore here)
    //sleep(2);
    Display_Params params;
    Display_Params_init(&params);
    params.lineClearMode = DISPLAY_CLEAR_BOTH;
    Display_Handle hSerial = Display_open(Display_Type_UART, &params);
    char reset_msg[] =
            "========================\nBoard Reset\n========================";
    Display_printf(hSerial, DisplayUart_SCROLLING, 0, reset_msg);

    AONBatMonEnable();
    ADC_init();

    RF_Params rfParams;
    RF_Params_init(&rfParams);
    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
    rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
#else
    rfHandle = RF_open(&rfObject, &RF_prop,
                       (RF_RadioSetup*) &RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*) &RF_cmdFs, RF_PriorityNormal, NULL, 0);

    ADC_Handle   adc;
    ADC_Params   adcparams;
    int_fast16_t res;

    ADC_Params_init(&adcparams);


    while (1)
    {
        //GPIO_toggle(Board_GPIO_LED1);
        if (AONBatMonNewBatteryMeasureReady())
        {
            bat_measure = AONBatMonBatteryVoltageGet();
            bat_voltage = ((bat_measure >> 8) & 0x7)
                    + ((float) (bat_measure & 0xFF)) / 256.0;
            Display_printf(hSerial, DisplayUart_SCROLLING, 0, "Bat: %f",
                           bat_voltage);

        }

        adc = ADC_open(Board_ADC0, &adcparams);

        if (adc != NULL) {
            res = ADC_convert(adc, &adcValue0);

            if (res == ADC_STATUS_SUCCESS) {

                adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);

                Display_printf(hSerial, DisplayUart_SCROLLING, 0, "ADC0 raw result: %d\n", adcValue0);
                Display_printf(hSerial, DisplayUart_SCROLLING, 0, "ADC0 convert result: %d uV\n",
                    adcValue0MicroVolt);
            }
            else {
                Display_printf(hSerial, DisplayUart_SCROLLING, 0, "ADC0 convert failed\n");
            }

            ADC_close(adc);
        } else {
            Display_printf(hSerial, DisplayUart_SCROLLING, 0, "Error initializing ADC0\n");
        }

        Display_printf(hSerial, DisplayUart_SCROLLING, 0, "Time: %d", time);



        packet[0] = bat_measure;
        packet[2] = ((bat_measure >> 8) & 0x7);
        packet[1] = time;
        packet[3] = adcValue0MicroVolt;

        if (bat_voltage < 3.15) {
            delay_seconds = SHORT_DELAY;
        } else {
            delay_seconds = LONG_DELAY;
        }

        RF_cmdPropTx.pktLen = PAYLOAD_LENGTH;
        RF_cmdPropTx.pPkt = packet;
        RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

        /* Send packet */
        RF_EventMask terminationReason = RF_runCmd(rfHandle,
                                                   (RF_Op*) &RF_cmdPropTx,
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
            while (1)
                ;
        }

        sleep(delay_seconds);
        time += delay_seconds;
    }

}

/* Semaphore to block master until slave is ready for transfer */
sem_t masterSem;

/*
 *  ======== masterThread ========
 *  Master SPI sends a message to slave while simultaneously receiving a
 *  message from the slave.
 */
void *masterThread(void *arg0)
{

    WS2812_beginSPI();

    //rainbow();
    //chirstLights();
    //animationBasic();
    while (1) {
        //rainbowAnimation();
        allWhite();
        GPIO_toggle(Board_GPIO_LED1);
        sleep(1);
        //rainbow();
        //allRed();
        /*allRed();
        sleep(3);
        allBlue();
        sleep(3);
        allGreen();
        sleep(3);
        chirstLights();
        sleep(3);*/
    }
    //bounce();

}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    pthread_t           thread0;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    pthread_t thread1;
    pthread_attr_t attrs1;
    struct sched_param priParam1;

    /* Call driver init functions. */
    Display_init();
    GPIO_init();
    SPI_init();

    /* Configure the LED pins */
    GPIO_setConfig(Board_GPIO_LED0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);

    /* Create application threads */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, THREADSTACKSIZE);
    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }

    /* Create master thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    retc = pthread_create(&thread0, &attrs, masterThread, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
}
