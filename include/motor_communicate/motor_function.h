#ifndef __MOTOR_FUNCTION_H__
#define __MOTOR_FUNCTION_H__

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

extern const uint8_t CRCHighTable[256];     //根據控制器說明書139頁生成的CRC檢索表
extern const uint8_t CRCLowTable[256];

extern int serialPort;      //儲存serial port的位置

typedef struct {
    uint8_t data[20];
    int length;
}serialData;   //創立一個新的struct資料型態，當作資料傳送的容器，data中儲存要傳送的資料，length紀錄data的長度

void serialInit();      //初始化serial port
void transmitData(serialData *transmitMsg);     //傳送data
void receiveData(serialData *receiveMsg);       //接收data
void CRC16Generate(serialData *msg);        //幫data加上crc碼

#endif
