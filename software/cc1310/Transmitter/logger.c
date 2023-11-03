/*
 * logger.c
 *
 *  Created on: Nov 3, 2023
 *      Author: alexb
 */
#include <stdlib.h>
#include "Board.h"
#include <third_party/spiffs/spiffs.h>
#include <third_party/spiffs/SPIFFSNVS.h>
#include <ti/display/Display.h>
#include <ti/display/DisplayUart.h>
#include "logger.h"



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



void addLog(Display_Handle displayHandle, char *log) {

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
    fd = SPIFFS_open(&fs, "logFile", SPIFFS_RDWR, 0);
    if (fd < 0) {
        /* File not found; create a new file & write message to it */
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Creating logFile...");

        fd = SPIFFS_open(&fs, "logFile", SPIFFS_CREAT | SPIFFS_RDWR | SPIFFS_APPEND, 0);
        if (fd < 0) {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                "Error creating logFile.");

            while (1);
        }


        Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Writing to logFile...");

        if (SPIFFS_write(&fs, fd, (void *) log, strlen(log)) < 0) {
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error writing logFile.");

            while (1) ;
        }

        SPIFFS_close(&fs, fd);
    }

    SPIFFS_close(&fs, fd);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Closed file handle.");


    SPIFFS_unmount(&fs);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Unmounted filesystem.");
}


char * readLogs(Display_Handle displayHandle) {

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
    fd = SPIFFS_open(&fs, "logFile", SPIFFS_RDWR, 0);
    if (fd < 0) {
        return 0;
    }
    char * buf = (char * ) malloc(32);
    char * str;
    uint16_t len = 0;
    while (SPIFFS_read(&fs, fd, buf, 32) > 0) {
        len += 32;
        str = (char *) realloc(str, len);
        strcat(str,buf);
    }

    SPIFFS_close(&fs, fd);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Closed file handle.");


    SPIFFS_unmount(&fs);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Unmounted filesystem.");

    return str;
}

uint16_t getLastLog(Display_Handle displayHandle) {
    char * log = readLogs(displayHandle);
    if (log != 0) {
        uint16_t index = strlen(log) - 1;
        while (log[index-1] != ' ') {
            index--;
        }
        return atoi(log+index);
    } else {
        return 0;
    }
}