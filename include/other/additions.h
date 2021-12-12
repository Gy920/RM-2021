#ifndef ADDITION_H_
#define ADDITION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <time.h>
#include <string>

typedef double systime;

double getPointLengths(const cv::Point2f &p);

template <class type, int length>
class RoundQueue
{
private:
    type data[length];
    int head;
    int tail;

public:
    RoundQueue<type, length>() : head(0), tail(0){};

    int size() const
    {
        return length;
    };

    bool empty()
    {
        return head == tail;
    };

    void push(const type &obj)
    {
        data[head] = obj;
        head = (head + 1) % length;
        if (head == tail)
        {
            tail = (tail + 1) % length;
        }
    };

    bool pop(type &obj)
    {
        if (empty())
            return false;
        obj = data[tail];
        tail = (tail + 1) % length;
        return true;
    }

    type &operator[](int idx)
    {
        while (tail + idx < 0)
            idx += length;
        return data[(tail + idx) % length];
    };
};
double get_wall_time();

double centerDistance(const cv::Rect2f &rect, const cv::Point2d src_center);

std::string  GetVedioName();

#endif