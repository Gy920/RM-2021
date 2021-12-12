#pragma once

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include <mutex>
#include <chrono>
#include <thread>

/* Serial frame's EOF may conflict with somewhere else's EOF */
#undef EOF

/*
 * @Brief: 控制战车帧结构体
 */
struct ControlData
{
    uint8_t if_find;
    uint8_t pitch_dev1; //Pitch相对角度
    uint8_t pitch_dev2;
    uint8_t yaw_dev1; //Yaw相对角度
    uint8_t yaw_dev2;
    uint8_t sum;
};

struct FeedBackData
{
    uint8_t u_flag;
    uint8_t roll_dev1;
    uint8_t roll_dev2;
    uint8_t pitch_dev1;
    uint8_t pitch_dev2;
    uint8_t yaw_dev1;
    uint8_t yaw_dev2;
    uint8_t pitch_speed_dev1;
    uint8_t pitch_speed_dev2;
    uint8_t yaw_speed_dev1;
    uint8_t yaw_speed_dev2;
    uint8_t ms_H;
    uint8_t ms_M;
    uint8_t ms_L;
};

/*
 * @Brief:  Serial communication protocol and inplement
 */
class Serial
{

public:
    /*
     * @Brief: 发射方式
     */
    enum ShootMode
    {
        FIRE = (uint8_t)(0x01),
        NO_FIRE = (uint8_t)(0x00)
    };

    /* @Brief:
     *      SYSTEM_ERROR:   System error catched. May be caused by wrong port number,
     *                      fragile connection between Jetson and STM, STM shutting
     *                      down during communicating or the sockets being suddenly
     *                      plugged out.
     *      OJBK:         Everything all right
     *      PORT_OCCUPIED:  Fail to close the serial port
     *      READ_WRITE_ERROR: Fail to write to or read from the port
     *      CORRUPTED_FRAME: Wrong frame format
     *      TIME_OUT:       Receiving time out
     */
    enum ErrorCode
    {
        SYSTEM_ERROR = 1,
        OJBK = 0,
        PORT_OCCUPIED = -1,
        READ_WRITE_ERROR = -2,
        CORRUPTED_FRAME = -3,
        TIME_OUT = -4
    };

public:
    Serial();
    Serial(const Serial &right) = delete;
    Serial(Serial &&) = delete;
    ~Serial();

    /*
     * @Brief: Open serial port and config options
     */
    int openPort();

    /*
     * @Brief:  close serial port
     */
    int closePort();

    /*
     * @Brief
     */
    bool isOpened() const;

    /*
     * @Brief:  控制机器人  turn ControlData into ControlFrame and send the ControlFrame
     * @Input:  time_duration:  try for how long to send control data
     */
    template <typename _Rep, typename _Period>
    int tryControl(const ControlData &controlData, const std::chrono::duration<_Rep, _Period> &time_duration)
    {
        // std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
        // if (lockGuard.owns_lock())
        {
            control(controlData);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return _errorCode;
        }
    }

    template <typename _Rep, typename _Period>
    int tryFeedBack(FeedBackData &feedBackData, const std::chrono::duration<_Rep, _Period> &time_duration)
    {
        // std::unique_lock<std::timed_mutex> lockGuard(_mutex, time_duration);
        // if (lockGuard.owns_lock())
        {
            feedBack(feedBackData);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            return _errorCode;
        }
        // else
        // {
        //     return PORT_OCCUPIED;
        // }
    }

    /*
     * @Brief:  DO NOT call this right after the record, control or feedback because
     *          '_errorCode' will not be thread safe after they return, which means
     *          that you may not get the exact error code representing the operation state.
     *          Call this function in the case which does not worry about thread safety.
     */
    int getErrorCode() const;

private:
    /*
     * @Brief: 帧头帧尾
     */
    enum
    {
        JetsonCommSOF = (uint8_t)0x66,
        JetsonCommEOF = (uint8_t)0x88,
        // JetsonCommEOF2 = (uint8_t)0x11,
        // JetsonCommSOF2 = (uint8_t)0x22,
        // ZERO = (uint8_t)0x00
    };

private:
    /* -1 if serial port not opened */
    int _serialFd;

    /* @See: 'enum ErrorCode' */
    int _errorCode;

    std::timed_mutex _mutex; //use like xianchengsuo

    /*
     * @Brief: TX2控制战车帧结构体
     */
    struct ControlFrame
    {
        uint8_t SOF; // Start of Frame
        uint8_t if_find;
        uint8_t pitch_dev1; //Pitch相对目标角度
        uint8_t pitch_dev2;
        uint8_t yaw_dev1; //Yaw相对目标角度
        uint8_t yaw_dev2;
        uint8_t sum;
        uint8_t EOF; //End of Frame.

    } _controlFrame;
    /*
     * @Brief: 电控回传数据帧结构体
     */
    struct FeedBackFrame
    {
        uint8_t SOF;
        //part of HP
        uint8_t u_flag;
        uint8_t roll_dev1;
        uint8_t roll_dev2;
        uint8_t pitch_dev1;
        uint8_t pitch_dev2;
        uint8_t yaw_dev1;
        uint8_t yaw_dev2;
        uint8_t pitch_speed_dev1;
        uint8_t pitch_speed_dev2;
        uint8_t yaw_speed_dev1;
        uint8_t yaw_speed_dev2;
        uint8_t ms_H;
        uint8_t ms_M;
        uint8_t ms_L;
        uint8_t EOF;
    } _feedBackFrame;

    int control(const ControlData &controlData);
    ControlFrame pack(const ControlData &controlData);
    int send();

    //for receiving data from the USB STM32
    int feedBack(FeedBackData &feedBackData);
    int receive();
    FeedBackData unpack(const FeedBackFrame &FeedBackData);
};
