//--------------------------
// packet.c - 
//
// Author: Cameron Hofbauer
// Version: October 2, 2023
//
//--------------------------

#include "packet.h"

// Define integer types
typedef int8_t byte;
typedef int32_t int;
typedef uint8_t unsigned int;
typedef uint16_t unsigned short;
typedef uint32_t unsigned int;

#define OK 0
#define SYSERR 1
#define MAX_SIZE 128

// Define actual packet containing function pointers
typedef struct {
    uint16_t _boardId;
    (*_pattern)(void); 
    uint8_t _footer;
} packet;

// Define queue of packets
packet queue[10];


// Prototype definitions

int insertPacketQueue(packet *p) {
    return SYSERR;
}


int buildPacket(uint16_t boardId, void (*func_ptr)(void)) {
    queue[1]->_boardId = boardId;
    queue[1]->_pattern = func_ptr;
    queue[1]->_footer = (uint8_t)0;

    return OK;
}