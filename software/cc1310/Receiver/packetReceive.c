#include "smartrf_settings/smartrf_settings.h"
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)
#include "packetReceive.h"


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


void* receivePacket(void *arg0)
{

    if (RFQueue_defineQueue(&dataQueue, rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer), NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while (1)
            ;
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
    rfHandle = RF_open(&rfObject, &RF_prop,
                       (RF_RadioSetup*) &RF_cmdPropRadioDivSetup, &rfParams);
#endif// DeviceFamily_CC26X0R2

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*) &RF_cmdFs, RF_PriorityNormal, NULL, 0);


    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*) &RF_cmdPropRx,
                                               RF_PriorityNormal, &callback,
                                               RF_EventRxEntryDone);

}

void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{

    if (e & RF_EventRxEntryDone)
    {
//        /* Toggle pin to indicate RX */
//        PIN_setOutputValue(ledPinHandle, Board_PIN_LED1,
//                           !PIN_getOutputValue(Board_PIN_LED1));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);

        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));



        /************************************************************
         * Packet Parsing and Pattern Switching
         ************************************************************/

        if (((PKT*) currentDataEntry)->strip_id == STRIP_ID) {
            receive_control.light_show_flags = ((PKT*) currentDataEntry)->light_show_flags;
            receive_control.delay = ((PKT*) currentDataEntry)->delay;
            receive_control.duration = ((PKT*) currentDataEntry)->duration;
            receive_control.start_color = ((PKT*) currentDataEntry)->start_color;
            receive_control.end_color = ((PKT*) currentDataEntry)->end_color;
            receive_control.timeout = ((PKT*) currentDataEntry)->timeout;
            function_flag = 0xff;
        }







//        if (packetDataPointer[0] == 0xAA) {
//            //Candy Cane
//            if (function_flag != 0) {
//                function_flag = 0;
//            }
//        } else if (packetDataPointer[0] == 0xAB) {
//            //Xmas Pulse
//            if (function_flag != 1) {
//                function_flag = 1;
//            }
//        } else if (packetDataPointer[0] == 0xBA) {
//            //Xmas Shift
//            if (function_flag != 2) {
//                function_flag = 2;
//            }
//        } else if (packetDataPointer[0] == 0xBB) {
//            //Single Pulse
//            if (function_flag != 3) {
//                function_flag = 3;
//            }
//        } else {
//            if (function_flag != 4) {
//                function_flag = 4;
//            }
//        }
        /************************************************************
         * End Packet Parsing and Pattern Switching
         ************************************************************/
        RFQueue_nextEntry();
    }

}
