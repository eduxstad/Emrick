/*
 * logger.h
 *
 *  Created on: Nov 3, 2023
 *      Author: alexb
 */

#ifndef LOGGER_H_
#define LOGGER_H_



#endif /* LOGGER_H_ */

void spiffsInit(Display_Handle displayHandle);
void removeLogs(void);
void addLog(Display_Handle displayHandle, char *log, uint16_t length);
char * readLogs(Display_Handle displayHandle);
uint16_t getLastLog(Display_Handle displayHandle);
