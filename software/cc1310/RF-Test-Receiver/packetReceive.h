/*
 * packetRecieve.h
 *
 *  Created on: Dec 28, 2023
 *      Author: alexb
 */
#include <ti/drivers/rf/RF.h>
#include "RFQueue.h"
#include "patterns.h"

#define PAYLOAD_LENGTH      20
static RF_Object rfObject;
static RF_Handle rfHandle;
static uint32_t packet[PAYLOAD_LENGTH];
RF_Params rfParams;

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             PAYLOAD_LENGTH /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */



/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;
float rec_count;
uint8_t last_rec;


/* used to describe structure of rf packets */
typedef struct {
    uint8_t             packet_size;
    uint16_t            strip_id;
    uint16_t            set_id;
    uint16_t            light_show_flags;
    RGB                 start_color;
    RGB                 end_color;
    uint16_t            delay;
    uint16_t            duration;
    uint8_t             function;
    uint8_t             timeout;
} PKT;


void* receivePacket(void *arg0);
void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);
