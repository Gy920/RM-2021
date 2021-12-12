//
// Created by duu on 2021/11/1.
//
#include "./AntiTop.h"
#include "opencv2/opencv.hpp"

/**
 * @brief 返回小陀螺状态下的补偿角度
 * @param distance 相机测得与装甲板角度
 * @param r_angle 旋转矩阵得到的装甲板角度
 * @param m_angle 通过宽高比测量出的装甲板角度
*/
double getOffsetAngle(double distance,double r_angle ,double m_angle,double lenght=0.2);

double getOffsetAngle(double distance,double r_angle ,double m_angle,double lenght){
    /* 利用公式计算，*/
    double box_angle=0.6*m_angle+0.4*box_angle;
    double end_angle=asinf64(sin(box_angle)*lenght/distance);
    std::cout<<"box_angle     =  "<<box_angle<<std::endl;
    std::cout<<"the distance  =  "<<distance<<std::endl;
    std::cout<<"the lenght    =  "<<lenght<<std::endl;
    std::cout<<"the angle     =  "<<end_angle<<std::endl;
    return end_angle;
}

