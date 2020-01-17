/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2018-12-25
Description: CDeviceNetCon
**************************************************/

#ifndef PROJECT_CDEVICENETCON_H
#define PROJECT_CDEVICENETCON_H

#include <iostream>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include "usb_can.h"

using namespace std;

typedef unsigned short uint16;
typedef unsigned char uint8;
//typedef unsigned int uint32;
typedef unsigned long long ULL;
typedef std::vector<USB_CAN_OBJ> CAN_QUEUE;


typedef char byte;

//#define uint uint16
//#define uint8 uint8

typedef enum _tagInterfaceType_
{
    Display_Connection,
    IO_Connection
}InterfaceType;

typedef enum _tagConObjStatusType_
{
    exist,
    nonentity
}ConObjStatusType;

extern std::mutex ReceiveMutex;//互斥锁
extern std::condition_variable ReceiveCondition;
extern std::vector<USB_CAN_OBJ> shareCanData;

class CProduceOrConsume
{
public:
    CProduceOrConsume();
    ~CProduceOrConsume();
    void SetAttribute(uint8 status, uint16 connectionId);
    void GetAttribute(uint8 *status, uint16 *connectionId);

protected:
    uint8 m_ucStatus;
    uint16 m_usConnection_id;
    std::vector<USB_CAN_OBJ> m_canQueue;

};

class CProduce:public CProduceOrConsume
{
public:
    void SendData(BYTE dataLen, ULL frameData);
    void SendData(USB_CAN_OBJ *canFrame, int frameNum);
};

class CConsume:public CProduceOrConsume
{
public:
    CConsume();
    void ReadData(CAN_QUEUE *canQueueDes, int frameNum = 1);
};

class CDeviceNetCon
{
public:
    CDeviceNetCon();
    ~CDeviceNetCon();
    CProduce dIAIProduce;
    CConsume devIAIConsume;
    void SetState(uint8 stateTmp);
    uint8 GetState();
    void SetInterfaceType(uint8 interfaceTypeTemp);
    uint8 GetInterfaceType();
    void SetProducedConID(uint16 producedConId);
    uint16 GetProducedConID();
    void SetConsumerConID(uint16 consumedConId);
    uint16 GetConsumerConID();
    void SetProducedConnectionSize(uint16 pConSize);
    uint16 GetProducedConnectionSize();
    void SetConsumedConnectionSize(uint16 cConSize);
    uint16 GetConsumedConnectionSize();

private:
    uint8 m_state;
    uint8 m_interface_type;
    char m_transportClass_trigger;
    uint16 m_produced_connection_id;
    uint16 m_consumed_connection_id;
    char m_initial_comm_characteristics;
    uint16 m_produced_connection_size;
    uint16 m_consumed_connection_size;
    uint16 m_expected_packet_rate;
    uint8 m_watch_dog_timeout_action;
    uint16 m_produced_connection_path_length;
//    epath m_produced_connection_path;
    uint16 m_consumed_connection_path_length;
//    epath m_consumed_connection_path;
    uint16 m_production_inhibit_time;
};

#endif //PROJECT_CDEVICENETCON_H
