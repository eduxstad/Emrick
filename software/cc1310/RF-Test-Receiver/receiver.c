/*
 * Copyright (c) 2015-2019, Texas Instruments Incorporated
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

/*
 *  ======== empty.c ========
 */

/* For usleep() */
#include <unistd.h>
#include <stdint.h>
#include <stddef.h>

#include <stdlib.h>
#include <stdio.h>

/* Driver Header files */

#include <ti/drivers/GPIO.h>
// #include <ti/drivers/I2C.h>
// #include <ti/drivers/SPI.h>
// #include <ti/drivers/Watchdog.h>

#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>

/* Supply Voltage Monitor */
#include <ti/devices/cc13x0/driverlib/aon_batmon.h>

#include <ti/drivers/ADC.h>


/* Board Header file */
#include "Board.h"

/* LED Strip Driver and Patterns */
#include "WS2812.h"
#include "packetReceive.h"
//#include "patterns.h"
#include <ti/drivers/SPI.h>

/* RF */

/* POSIX Header files (Threads) */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>



/* Threading */
#define THREADSTACKSIZE (1024)

/* Thread ID's */
pthread_t           thread0;
pthread_t           thread1;

/* UART Display */
Display_Handle displayHandle;



float sup_voltage;

/* Measures 3.3V supply voltage */

float supplyVoltage(Display_Handle displayHandle)
{
    uint32_t sup_measure;

    if (AONBatMonNewBatteryMeasureReady())
    {
        sup_measure = AONBatMonBatteryVoltageGet();
        sup_voltage = ((sup_measure >> 8) & 0x7)
                + ((float) (sup_measure & 0xFF)) / 256.0;


    }
    /* Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                   "No new Supply Voltage\n"); */

    return sup_voltage;
}

/* Measures supply voltage from battery from ADC */

uint32_t batteryMicroVoltage(Display_Handle displayHandle)
{
    ADC_Handle adc;
    ADC_Params adcparams;
    uint16_t adcValue0;
    uint32_t adcValue0MicroVolt;
    int_fast16_t res;

    ADC_Params_init(&adcparams);

    adc = ADC_open(Board_ADC6, &adcparams);

    if (adc != NULL)
    {
        res = ADC_convert(adc, &adcValue0);

        if (res == ADC_STATUS_SUCCESS)
        {

            adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
            ADC_close(adc);
            return adcValue0MicroVolt * 2;

            //Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "ADC0 raw result: %d", adcValue0);

        }
        else
        {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                           "ADC0 convert failed\n");
        }

    }
    else
    {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                       "Error initializing ADC0\n");
    }

    ADC_close(adc);

}


void displayResult() {
    float loss = (1.0 - rec_count / (float)last_rec) * 100;
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0,"Received packet #%d: %.2f%% loss", last_rec, loss);
    if (last_rec >= 100) {
        last_rec = 0;
        rec_count = 0.0;
    }
}


/* Creates and begins execution of receiver thread */

void createReceiverThread(pthread_attr_t attrs) {
    int retc = pthread_create(&thread0, &attrs, receivePacket, NULL);
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0)
{
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;
    pthread_attr_t attrs;

    /* Call driver init functions */
    GPIO_init();
    /* Init UART Display */
    Display_init();
    /* Supply Voltage Monitor */
    AONBatMonEnable();
    ADC_init();
    // initialize the NVS driver
    NVS_init();

    /* Configure the LED pin */
    GPIO_setConfig(Board_GPIO_LED1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(Board_GPIO_BOOST_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Turn on user LED */
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_ON);
    GPIO_toggle(Board_GPIO_LED1);


    /* Open the UART Display */
    displayHandle = Display_open(Display_Type_UART, NULL);
    if (displayHandle == NULL)
    {
        /* Display_open() failed */
        while (1)
            ;
    }

    Display_printf(
            displayHandle,
            DisplayUart_SCROLLING,
            0,
            "========================\r\nBoard Reset\r\n========================\r\nSmoketest starting up...");


    // Measure initial battery voltage
    float supply_volt = supplyVoltage(displayHandle);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                           "Supply Voltage: %f V", supply_volt);
    uint32_t bat_microVolt = batteryMicroVoltage(displayHandle);
    Display_printf(
                        displayHandle,
                        DisplayUart_SCROLLING,
                        0,
                        "Battery Voltage: %f V (random/floating value if disconnected)",
                        (float) bat_microVolt / 1000000);

    /* Create application thread(s) */
    pthread_attr_init(&attrs);

    detachState = PTHREAD_CREATE_DETACHED;
    /* Set priority and stack size attributes */
    retc = pthread_attr_setdetachstate(&attrs, detachState);
    if (retc != 0) {
        /* pthread_attr_setdetachstate() failed */
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                       "Unable to create thread stack size.");
        while (1);
    }

    retc |= pthread_attr_setstacksize(&attrs, 512);
    if (retc != 0) {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                       "Unable to create thread stack size.");
        /* pthread_attr_setstacksize() failed */
        while (1);
    }


    /* Create RX Listener thread */
    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&attrs, &priParam);

    createReceiverThread(attrs);

}

