#ifndef MY_FLASH_H
#define MY_FLASH_H
#include <Arduino.h>
void setDev(int8_t *buf);
void getDev(int8_t *buf);
void getServoPos(uint16_t *buf);
void bootTask();
extern int8_t servoDev[21];
#endif
