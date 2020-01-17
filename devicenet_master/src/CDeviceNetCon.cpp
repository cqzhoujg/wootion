/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2018-12-25
Description: CProduceOrConsume
**************************************************/

#include "CDeviceNetCon.h"

std::mutex ReceiveMutex;
std::condition_variable ReceiveCondition;
std::vector<USB_CAN_OBJ> shareCanData;

CProduceOrConsume::CProduceOrConsume()
{

}
CProduceOrConsume::~CProduceOrConsume()
{

}
CConsume::CConsume()
{

}

/*************************************************
Function: CProduce::SendData
Description:　发送DN报文
Input: BYTE dataLen   can帧数据的个数
       ULL  frameData 要发送的can数据
Output: void
Others: void
**************************************************/
void CProduce::SendData(BYTE dataLen, ULL frameData)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ROS_INFO("[CProduce::SendData] begin...");
    USB_CAN_OBJ canFrame;
    setCanFrame(&canFrame, m_usConnection_id, dataLen, frameData);
    printCanFrame(&canFrame, "[CProduce][SendData]");
    usbCan_Transmit(&canFrame, 1);
    ROS_INFO("[CProduce::SendData] end...");
}

/*************************************************
Function: CProduce::SendData
Description:　发送DN报文
Input: USB_CAN_OBJ *canFrame 要发送的can帧的首地址
       int          frameNum 发送的can帧数量
Output: void
Others: void
**************************************************/
void CProduce::SendData(USB_CAN_OBJ *canFrame, int frameNum)
{
//    ROS_INFO("[CProduce::SendData] begin...");
    for(int i=0; i<frameNum; i++)
    {
        canFrame[i].ID = m_usConnection_id;
        canFrame[i].ExternFlag = 0;
        canFrame[i].RemoteFlag = 0;
        canFrame[i].SendType = 0;
        for(int j=0;j<3;j++)
            canFrame[i].Reserved[j] = 0;
    }
/*
    for(int i =0;i<frameNum;i++)
    {
        PrintCanFrame(&canFrame[i], "[CProduce][SendData]");
    }
*/
    usbCan_Transmit(canFrame, frameNum);

//    ROS_INFO("[CProduce::SendData] end...");
}

/*************************************************
Function: CProduce::ReadData
Description:　读取DN报文
Input: CAN_QUEUE *canQueueDes  can数据接收队列
       int        frameNum     要接受的can帧个数
Output: void
Others: void
**************************************************/
void CConsume::ReadData(CAN_QUEUE *canQueueDes, int frameNum)
{
    ROS_INFO("[CConsume::ReadData] start...");
    m_canQueue.clear();
    std::unique_lock<std::mutex> lock(ReceiveMutex);
    while(shareCanData.size() < frameNum)
    {
        ReceiveCondition.wait(lock);
    }
    for(auto &shareData:shareCanData)
    {
        printCanFrame(&shareData, "[CConsume][ReadData]");
        if(shareData.ID == m_usConnection_id)
        {
            m_canQueue.push_back(shareData);
            canQueueDes->push_back(shareData);
        }
    }
    shareCanData.clear();
    lock.unlock();

    if(canQueueDes->size() != frameNum)
    {
        ROS_INFO("[CConsume::ReadData] data read error");
    }
    ROS_INFO("[CConsume::ReadData] end...");
}

void CProduceOrConsume::SetAttribute(uint8 status, uint16 connectionId)
{
    m_ucStatus = status;
    m_usConnection_id = connectionId;
}

void CProduceOrConsume::GetAttribute(uint8 *status, uint16 *connectionId)
{
    *status = m_ucStatus;
    *connectionId = m_usConnection_id;
}

CDeviceNetCon::CDeviceNetCon():
    m_state(0),
    m_interface_type(0),
    m_consumed_connection_id(0),
    m_produced_connection_id(0),
    m_produced_connection_size(32),
    m_consumed_connection_size(32)
{

}
void CDeviceNetCon::SetState(uint8 stateTmp)
{
    m_state = stateTmp;
}
uint8 CDeviceNetCon::GetState()
{
    return m_state;
}
void CDeviceNetCon::SetInterfaceType(uint8 interfaceTypeTemp)
{
    m_interface_type = interfaceTypeTemp;
}
uint8 CDeviceNetCon::GetInterfaceType()
{
    return m_interface_type;
}
void CDeviceNetCon::SetProducedConID(uint16 producedConId)
{
    m_produced_connection_id = producedConId;
}
uint16 CDeviceNetCon::GetProducedConID()
{
    return m_produced_connection_id;
}
void CDeviceNetCon::SetConsumerConID(uint16 consumedConId)
{
    m_consumed_connection_id = consumedConId;
}
uint16 CDeviceNetCon::GetConsumerConID()
{
    return m_consumed_connection_id;
}
void CDeviceNetCon::SetProducedConnectionSize(uint16 pConSize)
{
    m_produced_connection_size = pConSize;
}
uint16 CDeviceNetCon::GetProducedConnectionSize()
{
    return m_produced_connection_size;
}
void CDeviceNetCon::SetConsumedConnectionSize(uint16 cConSize)
{
    m_consumed_connection_size = cConSize;
}
uint16 CDeviceNetCon::GetConsumedConnectionSize()
{
    return m_consumed_connection_size;
}
CDeviceNetCon::~CDeviceNetCon()
{

}

