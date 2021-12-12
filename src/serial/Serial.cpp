
/**************************************************************
MIT License
Copyright (c) 2018 SEU-SuperNova-CVRA
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
Authors:    Binyan Hu
**************************************************************/
#include "../../include/serial/serial.h"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>

#include <iostream>
#include <fstream>
#include <string>
#include <string.h>
#include <stdexcept>
#include <exception>
#include <opencv2/opencv.hpp>

//#include "../General/General.h"

using namespace std;

//namespace rm
//{
Serial::Serial() : _serialFd(-1),
                   _errorCode(OJBK)
{
    // cout << "serial()" << endl;
    static_assert(sizeof(ControlFrame) == 8, "Size of backdata is not right");

    // 初始化通讯帧结构体通用项
    _controlFrame.EOF = JetsonCommSOF;
    //  _controlFrame.frame_seq = 0;
    _controlFrame.EOF = JetsonCommEOF;
}

Serial::~Serial()
{
    tcflush(_serialFd, TCIOFLUSH);
    if (-1 == close(_serialFd))
    {
        _errorCode = SYSTEM_ERROR;
        //     cout << "Serial closing  failed." << endl;
    }
    else
    {
        _errorCode = OJBK;
    }
}

int Serial::openPort()
{

    _serialFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    //_serialFd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_serialFd == -1)
    {
        // cout << "Open serial port failed." << endl;
        return _errorCode = SYSTEM_ERROR;
        ;
    }
    std::cout<<"open sucessful "<<std::endl;

    termios tOption;                // 串口配置结构体
    tcgetattr(_serialFd, &tOption); //获取当前设置
    cfsetispeed(&tOption, B460800); // 接收波特率
    cfsetospeed(&tOption, B460800); // 发送波特率
    cfmakeraw(&tOption);
      tcflush(_serialFd, TCIOFLUSH); //TCIOFLUSH刷新输入、输出队列。
    tcflush(_serialFd, TCIOFLUSH); 

    tcsetattr(_serialFd, TCSANOW, &tOption);
    tOption.c_cflag &= ~PARENB;
    tOption.c_cflag &= ~CSTOPB;
    tOption.c_cflag &= ~CSIZE;
    tOption.c_cflag |= CS8;
    tOption.c_cflag &= ~INPCK;
    // tOption.c_cflag |= (B57600 | CLOCAL | CREAD); // 设置波特率，本地连接，接收使能
    tOption.c_cflag &= ~(INLCR | ICRNL);
    tOption.c_cflag &= ~(IXON);
    tOption.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tOption.c_oflag &= ~OPOST;
    tOption.c_oflag &= ~(ONLCR | OCRNL);
    tOption.c_iflag &= ~(ICRNL | INLCR);
    tOption.c_iflag &= ~(IXON | IXOFF | IXANY);
    tOption.c_cc[VTIME] = 1; //只有设置为阻塞时这两个参数才有效
    tOption.c_cc[VMIN] = 1;
        cfmakeraw(&tOption);
    tcflush(_serialFd, TCIOFLUSH); //TCIOFLUSH刷新输入、输出队列。

    //  cout << "Serial preparation complete." << endl;
    return _errorCode = OJBK;
}

int Serial::closePort()
{
    tcflush(_serialFd, TCIOFLUSH);
    if (-1 == close(_serialFd))
    {
        _errorCode = SYSTEM_ERROR;
        cout << "Serial closing failed." << endl;
    }
    else
    {
        _errorCode = OJBK;
    }
    return _errorCode;
}

bool Serial::isOpened() const
{
    return (_serialFd != -1);
}


int Serial::control(const ControlData &controlData)
{
    _controlFrame = pack(controlData);
    return send();
}



Serial::ControlFrame Serial::pack(const ControlData &ctrl)
{
    return ControlFrame{
        JetsonCommSOF,
        ctrl.if_find,
        ctrl.pitch_dev1,
        ctrl.pitch_dev2,
        ctrl.yaw_dev1,
        ctrl.yaw_dev2,
	    ctrl.sum,
        JetsonCommEOF};
}

int Serial::feedBack(FeedBackData &feedBackData)
{
    if (receive() == OJBK)
        {
            feedBackData = unpack(_feedBackFrame);
        }
    return _errorCode;
}

int Serial::receive()
{
    const int recived_count=16;
    uint8_t receive[recived_count] = {0};
    int readCount = -1;
    //缓存区
    uint8_t pre_data[50]={0};
    memset(&_feedBackFrame, 0, sizeof(_feedBackFrame)); //作用是将某一块内存中的内容全部设置为指定的值， 这个函数通常为新申请的内存做初始化工作。
    // 一次读recived_count个字节在缓存区里
    readCount = read(_serialFd, ((unsigned char *)(&pre_data)), recived_count);//sizeof(receive));  
    if(readCount!=recived_count){
        return READ_WRITE_ERROR;
    }
    for(int i=0;i<readCount;i++){
        std::cout<<hex<<(int)pre_data[i]<<" ";
        receive[i]=pre_data[i];
    }
    std::cout<<std::endl;
    int i = 0;
    //对数据进行分配
    while (true)
    {	
        if (receive[i%recived_count] == JetsonCommSOF&&
        receive[(i+recived_count-1)%recived_count]==JetsonCommEOF)
        {
            _feedBackFrame.SOF = receive[(i++)% recived_count];
            _feedBackFrame.u_flag = receive[(i++)% recived_count];
            _feedBackFrame.roll_dev1 = receive[(i++)% recived_count];
            _feedBackFrame.roll_dev2 = receive[(i++)% recived_count];
            _feedBackFrame.pitch_dev1 = receive[(i++)% recived_count];
            _feedBackFrame.pitch_dev2 = receive[(i++)% recived_count];
            _feedBackFrame.yaw_dev1=receive[(i++)%recived_count];
            _feedBackFrame.yaw_dev2=receive[(i++)%recived_count];
            _feedBackFrame.pitch_speed_dev1=receive[(i++)%recived_count];
            _feedBackFrame.pitch_speed_dev2=receive[(i++)%recived_count];
            _feedBackFrame.yaw_speed_dev1=receive[(i++)%recived_count];
            _feedBackFrame.yaw_speed_dev2 = receive[(i++)% recived_count];
            _feedBackFrame.ms_H = receive[(i++)% recived_count];
            _feedBackFrame.ms_M = receive[(i++)% recived_count];
            _feedBackFrame.ms_L = receive[(i++)% recived_count];
            _feedBackFrame.EOF = receive[(i++)% recived_count];
            break;
        }
        i++;
        if(i>=recived_count)
            return _errorCode = CORRUPTED_FRAME;
    }
    tcflush(_serialFd, TCIFLUSH);
    return _errorCode = OJBK;
}

FeedBackData Serial::unpack(const Serial::FeedBackFrame &fb)
{
    return FeedBackData{
        fb.u_flag,
        fb.roll_dev1,
        fb.roll_dev2,
        fb.pitch_dev1,
        fb.pitch_dev2,
        fb.yaw_dev1,
        fb.yaw_dev2,
        fb.pitch_speed_dev1,
        fb.pitch_speed_dev2,
        fb.yaw_speed_dev1,
        fb.yaw_speed_dev2,
	    fb.ms_H,
        fb.ms_M,
        fb.ms_L
    };
}

int Serial::send()
{
    tcflush(_serialFd, TCOFLUSH);
    int sendCount;
    try
    {
        sendCount = write(_serialFd, &_controlFrame, sizeof(_controlFrame));
    }
    catch (exception e)
    {
        cout << e.what() << endl;
        return _errorCode = SYSTEM_ERROR;
    }

    if (sendCount == -1)
    {
        _errorCode = READ_WRITE_ERROR;
    }
    else if (sendCount < static_cast<int>(sizeof(ControlFrame))){
        _errorCode = READ_WRITE_ERROR;
    }
    else{
        _errorCode = OJBK;
    }

    return _errorCode;
}

