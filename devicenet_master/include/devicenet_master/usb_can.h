//
// Created by zhoujg on 19-1-24.
//

#ifndef PROJECT_USB_CAN_H
#define PROJECT_USB_CAN_H
#include "controlcan.h"
#include <iostream>
#include <thread>

using namespace std;

typedef VCI_CAN_OBJ USB_CAN_OBJ;

int usbCan_Receive(USB_CAN_OBJ *canFrames, unsigned bufLen);

void setCanFrame(USB_CAN_OBJ *canFrame, UINT ID, BYTE DataLen, unsigned long long frameData);

void printCanFrame(USB_CAN_OBJ *canFrame,const char* FuncName="");

void canDataCpy(USB_CAN_OBJ *desCanFrame, USB_CAN_OBJ *souCanFrame);

void usbCan_Transmit(USB_CAN_OBJ *canFrames, int frameNum);

bool checkCanFrame(const USB_CAN_OBJ *canFrame, unsigned long long frameData);



#endif //PROJECT_USB_CAN_H
