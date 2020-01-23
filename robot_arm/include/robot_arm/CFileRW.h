/*************************************************
Copyright: wootion
Author: ZhouJinGang
Date: 2019-7-8
Description: CFileRW
**************************************************/

#ifndef PROJECT_CFILERW_H
#define PROJECT_CFILERW_H

#include <iostream>
#include <string>
#include <string.h>
#include <mutex>
#include <fcntl.h>
#include <unistd.h>
#include <ros/ros.h>


using namespace std;

class CFileRW
{
public:
    CFileRW()
    {
        m_bIsFileOpen = false;
    }
   ~CFileRW()
   {
       CloseFile();
   }
    int OpenFile(string &sFilePath,const int nOpenType)
    {
        if(m_bIsFileOpen)
        {
            ROS_INFO("[OpenFile] the file is already open");
            return 1;
        }
        std::unique_lock<std::mutex> lock(m_mFileMutex);

//        int nOpenType = sType.find('w') != string::npos ? O_CREAT | O_TRUNC | O_RDWR : O_RDONLY;
        m_fp = open(sFilePath.c_str(), nOpenType, 0666);

        if(m_fp == -1)
        {
            ROS_INFO("[OpenFile] open file:%s failed.nOpenType:%d",sFilePath.c_str(),nOpenType);
            return -1;
        }
        lock.unlock();
        m_bIsFileOpen = true;
        return 1;
    }

    int CloseFile()
    {
        if(!m_bIsFileOpen)
        {
            return 1;
        }
        std::unique_lock<std::mutex> lock(m_mFileMutex);
        if(close(m_fp) == -1)
        {
            return -1;
        }
        lock.unlock();
        m_bIsFileOpen = false;
        return 1;
    }

    void Output(int Value)
    {
        if(!m_bIsFileOpen)
        {
            return;
        }
        std::unique_lock<std::mutex> lock(m_mFileMutex);
        char buf[128];
        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%d", Value);
        int nRetLen = 0;
        int nLength = strlen(buf);
        if(( nRetLen = write(m_fp,buf,nLength)) != nLength)
        {
            ROS_ERROR("[CFileRW] int write length error, nRetLen = %d, nLength=%d", nRetLen, nLength);
        }
        lock.unlock();
    }
    void Output(float Value)
    {
        if(!m_bIsFileOpen)
        {
            return;
        }
        std::unique_lock<std::mutex> lock(m_mFileMutex);
        char buf[128];
        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%f", Value);
        int nRetLen = 0;
        int nLength = strlen(buf);
        if(( nRetLen = write(m_fp,buf,nLength)) != nLength)
        {
            ROS_ERROR("[CFileRW] float write length error, nRetLen = %d, nLength=%d", nRetLen, nLength);
        }
        lock.unlock();
    }
    void Output(double Value)
    {
        if(!m_bIsFileOpen)
        {
            return;
        }
        std::unique_lock<std::mutex> lock(m_mFileMutex);
        char buf[128];
        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%f", Value);
        int nRetLen = 0;
        int nLength = strlen(buf);
        if(( nRetLen = write(m_fp,buf,nLength)) != nLength)
        {
            ROS_ERROR("[CFileRW] double write length error, nRetLen = %d, nLength=%d", nRetLen, nLength);
        }
        lock.unlock();
    }

    void Output(char Value)
    {
        if(!m_bIsFileOpen)
        {
            return;
        }
        std::unique_lock<std::mutex> lock(m_mFileMutex);
        char buf[8];
        memset(buf, 0, sizeof(buf));
        sprintf(buf, "%c", Value);
        int nRetLen = 0;
        int nLength = strlen(buf);
        if(( nRetLen = write(m_fp,buf,nLength)) != nLength)
        {
            ROS_ERROR("[CFileRW] char write length error, nRetLen = %d, nLength=%d", nRetLen, nLength);
        }
        lock.unlock();
    }
    void Output(string &strOutput)
    {
        if(!m_bIsFileOpen)
        {
            return;
        }
        
        std::unique_lock<std::mutex> lock(m_mFileMutex);
        int nRetLen = 0;
        int nLength = strOutput.size();
        if(( nRetLen = write(m_fp,strOutput.c_str(),nLength)) != nLength)
        {
            ROS_ERROR("[CFileRW] string write length error, nRetLen = %d, nLength=%d", nRetLen, nLength);
        }
        lock.unlock();
    }

private:
    std::mutex m_mFileMutex;
    bool m_bIsFileOpen;
    int m_fp;
};
#endif //PROJECT_CFILERW_H
