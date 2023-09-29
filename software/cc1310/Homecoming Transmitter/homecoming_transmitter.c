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

/* Flash Read/Write */
#include <third_party/spiffs/spiffs.h>
#include <third_party/spiffs/SPIFFSNVS.h>

/* Board Header file */
#include "Board.h"

/* LED Strip Driver and Patterns */
#include "WS2812.h"
#include "patterns.h"
#include <ti/drivers/SPI.h>

/* RF */
#include <ti/drivers/rf/RF.h>
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* POSIX Header files (Threads) */
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>

/* RF */
#define PAYLOAD_LENGTH      20
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint32_t packet[PAYLOAD_LENGTH] = {1, 8, 8, 6, 2, 0, 2, 3};
RF_Params rfParams;

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             PAYLOAD_LENGTH /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


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

/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;

/* Threading */
#define THREADSTACKSIZE (1024)

/* UART Display */
Display_Handle displayHandle;

/* SPIFFS configuration parameters */
#define SPIFFS_LOGICAL_BLOCK_SIZE    (4096)
#define SPIFFS_LOGICAL_PAGE_SIZE     (256)
#define SPIFFS_FILE_DESCRIPTOR_SIZE  (44)

#define MESSAGE_LENGTH (8)
uint8_t fileArrayHeap[8] = {1, 8, 8, 6, 2, 0, 2, 3};
uint8_t fileArrayRead[8] = {0};

/*
 * SPIFFS needs RAM to perform operations on files.  It requires a work buffer
 * which MUST be (2 * LOGICAL_PAGE_SIZE) bytes.
 */
static uint8_t spiffsWorkBuffer[SPIFFS_LOGICAL_PAGE_SIZE * 2];

/* The array below will be used by SPIFFS as a file descriptor cache. */
static uint8_t spiffsFileDescriptorCache[SPIFFS_FILE_DESCRIPTOR_SIZE * 4];

/* The array below will be used by SPIFFS as a read/write cache. */
static uint8_t spiffsReadWriteCache[SPIFFS_LOGICAL_PAGE_SIZE * 2];

spiffs fs;
SPIFFSNVS_Data spiffsnvsData;

float sup_voltage;

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

uint32_t batteryMicroVoltage(Display_Handle displayHandle)
{
    ADC_Handle adc;
    ADC_Params adcparams;
    uint16_t adcValue0;
    uint32_t adcValue0MicroVolt;
    int_fast16_t res;

    ADC_Params_init(&adcparams);

    adc = ADC_open(Board_ADC0, &adcparams);

    if (adc != NULL)
    {
        res = ADC_convert(adc, &adcValue0);

        if (res == ADC_STATUS_SUCCESS)
        {

            adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
            ADC_close(adc);
            return adcValue0MicroVolt;

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


void smoketestFlash(Display_Handle displayHandle) {

    spiffs_file    fd;
    spiffs_config  fsConfig;
    int32_t        status;


#ifdef Board_wakeUpExtFlash
    Board_wakeUpExtFlash();
#endif

    /* Initialize spiffs, spiffs_config & spiffsnvsdata structures */
    status = SPIFFSNVS_config(&spiffsnvsData, Board_NVSEXTERNAL, &fs, &fsConfig,
        SPIFFS_LOGICAL_BLOCK_SIZE, SPIFFS_LOGICAL_PAGE_SIZE);
    if (status != SPIFFSNVS_STATUS_SUCCESS) {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
            "Error with SPIFFS configuration.");

        while (1);
    }

    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Mounting flash file system via SPI");

    status = SPIFFS_mount(&fs, &fsConfig, spiffsWorkBuffer,
        spiffsFileDescriptorCache, sizeof(spiffsFileDescriptorCache),
        spiffsReadWriteCache, sizeof(spiffsReadWriteCache), NULL);
    if (status != SPIFFS_OK) {
        /*
         * If SPIFFS_ERR_NOT_A_FS is returned; it means there is no existing
         * file system in memory.  In this case we must unmount, format &
         * re-mount the new file system.
         */
        if (status == SPIFFS_ERR_NOT_A_FS) {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                "File system will not be found at first. Must unmount.");

            SPIFFS_unmount(&fs);
            status = SPIFFS_format(&fs);
            if (status != SPIFFS_OK) {
                Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                    "Error formatting memory.");

                while (1);
            }

            status = SPIFFS_mount(&fs, &fsConfig, spiffsWorkBuffer,
                spiffsFileDescriptorCache, sizeof(spiffsFileDescriptorCache),
                spiffsReadWriteCache, sizeof(spiffsReadWriteCache), NULL);
            if (status != SPIFFS_OK) {
                Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                    "Error mounting file system.");

                while (1);
            }
        }
        else {
            /* Received an unexpected error when mounting file system  */
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                "Error mounting file system: %d.", status);

            while (1);
        }
    }

    /* Open a file */
    fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_RDWR, 0);
    if (fd < 0) {
        /* File not found; create a new file & write message to it */
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Creating spiffsFile...");

        fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_CREAT | SPIFFS_RDWR, 0);
        if (fd < 0) {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                "Error creating spiffsFile.");

            while (1);
        }


        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Writing to spiffsFile...");

        if (SPIFFS_write(&fs, fd, (void *) fileArrayHeap, MESSAGE_LENGTH) < 0) {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error writing spiffsFile.");

            while (1) ;
        }

        SPIFFS_close(&fs, fd);
    }

    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Reading spiffsFile...");

    fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_RDWR, 0);

    if (SPIFFS_read(&fs, fd, fileArrayRead, MESSAGE_LENGTH) < 0)
    {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error reading spiffsFile.");

        while (1);
    }

    int8_t i = 0;
    while (i < MESSAGE_LENGTH) {
        if (fileArrayRead[i] != fileArrayHeap[i]) {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error: spiffsFile does not match at index %d, expected %d and read %d", i, fileArrayHeap[i], fileArrayRead[i]);
        }
        i++;
    }

    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Removing spiffsFile...");
    status = SPIFFS_fremove(&fs, fd);
    if (status != SPIFFS_OK)
    {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error removing spiffsFile.");

        while (1);
    }

    SPIFFS_close(&fs, fd);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Closed file handle.");


    SPIFFS_unmount(&fs);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Unmounted filesystem.");

}

void sendRF(Display_Handle displayHandle, uint8_t * pkt, uint16_t length)
{
    RF_Params_init(&rfParams);

    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Sending sample packet...");
    /* Request access to the radio */
#if defined(DeviceFamily_CC26X0R2)
       rfHandle = RF_open(&rfObject, &RF_prop, (RF_RadioSetup*)&RF_cmdPropRadioSetup, &rfParams);
   #else
    rfHandle = RF_open(&rfObject, &RF_prop,
                       (RF_RadioSetup*) &RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*) &RF_cmdFs, RF_PriorityNormal, NULL, 0);

    RF_cmdPropTx.pktLen = length;
    RF_cmdPropTx.pPkt = pkt;
    RF_cmdPropTx.startTrigger.triggerType = TRIG_NOW;

    /* Send packet */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*) &RF_cmdPropTx,
                                               RF_PriorityNormal, NULL, 0);

    switch (terminationReason)
    {
    case RF_EventLastCmdDone:
        // A stand-alone radio operation command or the last radio
        // operation command in a chain finished.
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "RF command done.");
        break;
    case RF_EventCmdCancelled:
        // Command cancelled before it was started; it can be caused
        // by RF_cancelCmd() or RF_flushCmd().
    case RF_EventCmdAborted:
        // Abrupt command termination caused by RF_cancelCmd() or
        // RF_flushCmd().
    case RF_EventCmdStopped:
        // Graceful command termination caused by RF_cancelCmd() or
        // RF_flushCmd().
    default:
        // Uncaught error event
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error with RF command.");
        while (1)
            ;
    }

    uint32_t cmdStatus = ((volatile RF_Op*) &RF_cmdPropTx)->status;
    switch (cmdStatus)
    {
    case PROP_DONE_OK:
        // Packet transmitted successfully
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Packet transmitted successfully.");
        break;
    case PROP_DONE_STOPPED:
        // received CMD_STOP while transmitting packet and finished
        // transmitting packet
    case PROP_DONE_ABORT:
        // Received CMD_ABORT while transmitting packet
    case PROP_ERROR_PAR:
        // Observed illegal parameter
    case PROP_ERROR_NO_SETUP:
        // Command sent without setting up the radio in a supported
        // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
    case PROP_ERROR_NO_FS:
        // Command sent without the synthesizer being programmed
    case PROP_ERROR_TXUNF:
        // TX underflow observed during operation
    default:
        // Uncaught error event - these could come from the
        // pool of states defined in rf_mailbox.h
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error transmitting packet.");
        while (1)
            ;
    }

    RF_close(rfHandle);

}

/*
 *  ======== mainThread ========
 */
void* mainThread(void *arg0)
{

    /* Call driver init functions */
    GPIO_init();
    /* Init UART Display */
    Display_init();
    /* Supply Voltage Monitor */
    AONBatMonEnable();
    ADC_init();


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
    //smoketestFlash(displayHandle);

//    while (1) {
//        sendRF(displayHandle);
//        GPIO_toggle(Board_GPIO_LED1);
//        sleep(1);
//    }

    uint8_t pktON[1];
    pktON[0] = 0xFF;
    uint8_t pktOFF[1];
    pktOFF[0] = 0x00;


    sendRF(displayHandle, pktON, 1);
    sendRF(displayHandle, pktON, 1);
    sendRF(displayHandle, pktON, 1);
    sendRF(displayHandle, pktON, 1);
    sleep(20);
    sendRF(displayHandle, pktOFF, 1);
    sendRF(displayHandle, pktOFF, 1);
    sendRF(displayHandle, pktOFF, 1);
    sendRF(displayHandle, pktOFF, 1);
    GPIO_toggle(Board_GPIO_LED1);


    while (1) {
        sleep(3);
        GPIO_toggle(Board_GPIO_LED1);
        sleep(1);
        GPIO_toggle(Board_GPIO_LED1);
    }



//    int clock_seconds = 0;
//    int delay = 1;
//    while (delay)
//    {
//        sleep(1);
//        supply_volt = supplyVoltage(displayHandle);
//        bat_microVolt = batteryMicroVoltage(displayHandle);
//        Display_printf(displayHandle, 0, 0,
//                       "\r(%02d:%02d) <SUPPLY: %fV> <BAT: %fV> Smoketest running", clock_seconds/60, clock_seconds % 60, supply_volt, (float) bat_microVolt / 1000000);
//        GPIO_toggle(Board_GPIO_LED1);
//        clock_seconds += delay;
//    }
}

