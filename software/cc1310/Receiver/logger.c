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

void spiffsInit(Display_Handle displayHandle) {

    spiffs_file    fd;
    spiffs_config  fsConfig;
    int32_t        status;


#ifdef Board_wakeUpExtFlash
    Board_wakeUpExtFlash();
#endif

    /* Initialize spiffs, spiffs_config & spiffsnvsdata structures */
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Mounting flash file system via SPI");
    status = SPIFFSNVS_config(&spiffsnvsData, Board_NVSEXTERNAL, &fs, &fsConfig,
        SPIFFS_LOGICAL_BLOCK_SIZE, SPIFFS_LOGICAL_PAGE_SIZE);
    if (status != SPIFFSNVS_STATUS_SUCCESS) {
        Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
            "Error with SPIFFS configuration.");

        while (1);
    }





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
}

void removeLogs() {
    spiffs_file fd;
    fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_RDWR | SPIFFS_APPEND, 0);
    if (fd > 0) {
        SPIFFS_fremove(&fs,fd);
    }
    SPIFFS_close(&fs,fd);
    SPIFFS_format(&fs);
}

void addLog(Display_Handle displayHandle, char *log, uint16_t length) {

    spiffs_file    fd;
    spiffs_config  fsConfig;
    int32_t        status;

    /* Open a file */
        fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_RDWR | SPIFFS_APPEND, 0);
        if (fd < 0) {
            /* File not found; create a new file & write message to it */
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Creating spiffsFile...");

            fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_CREAT | SPIFFS_RDWR | SPIFFS_APPEND, 0);
            if (fd < 0) {
                Display_printf(displayHandle, DisplayUart_SCROLLING, 0,
                    "Error creating spiffsFile.");

                while (1);
            }
        }

            align8bytes(log, length, displayHandle);
            Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Writing to spiffsFile...");
            if (SPIFFS_write(&fs, fd, (void *) log, strlen(log)) < 0) {
                Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Error writing spiffsFile.");

                while (1) ;
            }



    SPIFFS_close(&fs, fd);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Closed file handle.");
}


char * readLogs(Display_Handle displayHandle) {

    spiffs_file    fd;
    spiffs_config  fsConfig;
    int32_t        status;


    /* Open a file */
    fd = SPIFFS_open(&fs, "spiffsFile", SPIFFS_RDWR, 0);
    if (fd < 0) {
        return 0;
    }
    char buf[32];
    size_t len = 0;
    char * str;
    s32_t s = SPIFFS_read(&fs, fd, buf, 8);
    uint8_t eof = 0;
    while (s > 0 || eof) {
        size_t oldLen = len;
        len += s;
        char * tmp = str;
        str = (char *) malloc(len+1);
        *(str) = '\0';
        if (len > 8) {
            strcpy(str,tmp);
        }
        *(str+oldLen) = '\0';
        strcat(str,buf);
        uint8_t i;
        for (i = 0; i < s; i++) {
            if (*(str+i) == '\0') {
                eof = 1;
            }
        }
        *(str+len) = '\0';
        s = SPIFFS_read(&fs, fd, buf, 8);
    }

    SPIFFS_close(&fs, fd);
    Display_printf(displayHandle, DisplayUart_SCROLLING, 0, "Closed file handle.");

    return str;
}

uint16_t getLastLog(Display_Handle displayHandle) {
    char * log = readLogs(displayHandle);
    if (log != 0) {
        uint16_t index = strlen(log) - 1;
        while (log[index-1] != ' ') {
            index--;
        }
        uint16_t num = atoi(log+index);
        free(log);
        return num;
    } else {
        return 0;
    }
}

char * align8bytes(char * str, uint16_t length) {
    char * tmp;
    if (length % 8 != 0) {
        int offset = 8 - (length % 8);
        tmp = (char *) malloc(length + offset);
        int i;
        for (i = 0; i < offset; i++) {
            *(tmp+i) = ' ';
        }
        *(tmp+offset) = '\0';
        strcat(tmp, str);
        strcpy(str, tmp);
        free(tmp);
    }
    return tmp;
}
