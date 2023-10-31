//--------------------------
// packet.c - 
//
// Author: Cameron Hofbauer
// Version: October 30, 2023
//
//--------------------------

#include <stdlib.h>
#include "packet.h"

// Define integer types
typedef int8_t byte;
typedef int32_t int;
typedef uint8_t unsigned int;
typedef uint16_t unsigned short;
typedef uint32_t unsigned int;


// Define macro constants
#define OK 0
#define SYSERR 1
#define MAX_SIZE 128


// Define actual packet containing function pointers
// Packet will only contain the set num (and show num in the future)
// The receiver will then execute instructions based
// on what the set num indicates
typedef struct {
    // uint16_t _showNum;
    uint16_t _setNum;
    uint8_t _footer;
} packet;


// Define queue of packets
// will be implemented as a bounded buffer
packet *queue[MAX_SIZE];


// "Bounded Buffer" variables/macros
#define increment(x) (x+1)%MAX_SIZE

uint8_t startIdx = 0;
uint8_t endIdx = 0;


// Prototype definitions
int insertPacketQueue(packet *p);
packet *removePacketQueue();
int buildPacket(packet *p, uint16_t boardId, void (*func_ptr)(void));



// Functions


//
int insertPacketQueue(packet *p) {
    // Cannot insert into a full bounded buffer
    if (startIdx - 1 == endIdx || 
            (startIdx == 0 && endIdx == MAX_SIZE - 1)) {
        return SYSERR;
    }

    // Insert into startIdx
    queue[startIdx] = p;


    // Increment startIdx
    increment(startIdx);


    return OK;
}

//
packet *removePacketQueue() {

    if (startIdx == endIdx) {
        return (packet *)NULL;
    }

    packet *rmP = queue(endIdx);
    increment(endIdx);
    return rmP;
}

//
int buildPacket(packet *p, uint16_t setNum) {

    if (p == NULL) {
        return SYSERR;
    }

    // Insert show number (to be added in the future)
    //p->_showNum;

    // Insert set number
    p->_setNum = setNum;

    // Insert footer (Just a 0 for now)
    p->_footer = (uint8_t)0;

    return OK;
}


