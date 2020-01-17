/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2018-12-25
Description: USBCAN 功能函数二次封装
**************************************************/

#include <ros/ros.h>
#include "usb_can.h"
#include <string>

using namespace std;

/*************************************************
Function: usbCan_Receive
Description:　二次封装USBCAN接收库函数
Input: USB_CAN_OBJ *canFrames  can数据
       int          frameNum     要接受的can帧个数
Output: void
Others: void
**************************************************/
int usbCan_Receive(USB_CAN_OBJ *canFrames, unsigned bufLen)
{
    int recLen = 0;
    recLen = VCI_Receive(VCI_USBCAN2, 0, 0, canFrames, bufLen, 100/*ms*/);
    return recLen;
}

/*************************************************
Function: usbCan_Transmit
Description:　二次封装USBCAN发送库函数
Input: USB_CAN_OBJ *canFrames  can数据
       int          frameNum   要接受的can帧个数
Output: void
Others: void
**************************************************/
void usbCan_Transmit(USB_CAN_OBJ *canFrames, int frameNum)
{
    VCI_Transmit(VCI_USBCAN2, 0, 0, canFrames, (unsigned int)frameNum);
}

/*************************************************
Function: setCanFrame
Description:　设置can帧数据
Input: USB_CAN_OBJ       *canFrames  can数据
       UINT               ID         can帧中的ID
       BYTE               DataLen    can帧中数据的长度
       unsigned long long frameData  can帧中对应的数据
Output: void
Others: void
**************************************************/
void setCanFrame(USB_CAN_OBJ *canFrame, UINT ID, BYTE DataLen, unsigned long long frameData)
{
    canFrame->ID = ID;
    /*只使用标准数据帧*/
    canFrame->RemoteFlag = 0;
    canFrame->ExternFlag = 0;
    canFrame->SendType = 0;

    for(int i=0;i<3;i++)
        canFrame->Reserved[i] = 0;

    canFrame->DataLen = DataLen;
    for(int i=DataLen-1; i >= 0; i--)
    {
        canFrame->Data[i] = BYTE(frameData & 0xFF);
        frameData >>= 8;
    }
}

/*************************************************
Function: printCanFrame
Description:　设置can帧数据
Input: USB_CAN_OBJ       *canFrames  can数据
       const char        *FuncName   调用者的函数名
Output: void
Others: void
**************************************************/
void printCanFrame(USB_CAN_OBJ *canFrame, const char* FuncName)
{
    string sPrintStr;
    char buf[32];
    sPrintStr += FuncName;
    sPrintStr += "[printFrame]";

    sprintf(buf,"ID:0x%08X ", canFrame->ID);//ID
    sPrintStr += buf;

    if(canFrame->ExternFlag==0) sprintf(buf, "Standard ");//帧格式：标准帧
    if(canFrame->ExternFlag==1) sprintf(buf, "Extend   ");//帧格式：扩展帧
    sPrintStr += buf;

//    if(canFrame->RemoteFlag==0) sprintf(buf, "Data   ");//帧类型：数据帧
//    if(canFrame->RemoteFlag==1) sprintf(buf, "Remote ");//帧类型：远程帧
//    sPrintStr += buf;

    sprintf(buf, "Len:%d", canFrame->DataLen);//帧长度
    sPrintStr += buf;

    sPrintStr += " data:0x";	//数据
    for(int i = 0; i < canFrame->DataLen; i++)
    {
        sprintf(buf, " %02X", canFrame->Data[i]);
        sPrintStr += buf;
    }
//    printf(buf, " TimeStamp:0x%08X",canFrame->TimeStamp);//时间标识。
//    sPrintStr += buf;
//    printf("%s\n",sPrintStr.c_str());
    ROS_INFO("%s",sPrintStr.c_str());
}

/*************************************************
Function: canDataCpy
Description:　can帧拷贝函数
Input: USB_CAN_OBJ *desCanFrame 目标can帧
       USB_CAN_OBJ *souCanFrame 源can帧
Output: void
Others: void
**************************************************/
void canDataCpy(USB_CAN_OBJ *desCanFrame, USB_CAN_OBJ *souCanFrame)
{
    desCanFrame->ExternFlag = souCanFrame->ExternFlag;
    desCanFrame->RemoteFlag = souCanFrame->RemoteFlag;
    desCanFrame->SendType = souCanFrame->SendType;
    desCanFrame->TimeFlag = souCanFrame->TimeFlag;
    desCanFrame->TimeStamp = souCanFrame->TimeStamp;

    for(int i=0;i<3;i++)
        desCanFrame->Reserved[i] = souCanFrame->Reserved[i];

    desCanFrame->ID = souCanFrame->ID;
    desCanFrame->DataLen = souCanFrame->DataLen;
    for(int i=0; i<desCanFrame->DataLen ;i++)
    {
        desCanFrame->Data[i] = souCanFrame->Data[i];
    }
}

/*************************************************
Function: checkCanFrame
Description:　can帧拷贝函数
Input: USB_CAN_OBJ         *canFrame    待校验的CAN帧
       unsigned long long  frameData    与之校验的数据位字段
Output: void
Others: void
**************************************************/
bool checkCanFrame(const USB_CAN_OBJ *canFrame, unsigned long long frameData)
{
    unsigned int DataLen = canFrame->DataLen;

    for(int i=DataLen-1 ; i>=0 ; i--)
    {
        if(canFrame->Data[i] != u_char(frameData & 0xFF))
        {
            return false;
        }
        frameData >>= 8;
    }
    return true;
}