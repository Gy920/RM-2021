//
// Created by duu on 2021/11/1.
//
#include <iostream>
#include <queue>
#include "opencv2/opencv.hpp"

#ifndef MAIN_ANTITOP_H
#define MAIN_ANTITOP_H

class TopData{

    double time;
    double yaw;
    double pitch ;
    double rect_angle ;
    double distance;
    cv::Mat rotate_mat;
public:
    TopData(double &d_time,double& d_yaw,double& d_pitch,double &d_rect_angle,const cv::Mat &r_mat,double distance =3 ):
    time(d_time),yaw(d_yaw),pitch(d_pitch),rect_angle(d_rect_angle),rotate_mat(r_mat){

    }
};

/* #小陀螺自动判断逻辑
 * 放入一个队列里面判断符合陀螺判断标准的帧数
 * 寻找十帧数据中 max_yaw  min_yaw  max_pitch min_pitch
 * 寻找yaw角度变换主趋势（左右中左（-+-），右中左（+-），即逆时针自转）,或顺时针自转,不满足则返回
 * 判断依据：
 *          旋转矩形的角度变化趋势与主趋势相同
 *          相邻n帧内pitch轴角度变化小于 D_PITCH
 *          相邻n帧间的时间差小于 DT
 *          相邻n帧的距离变化和小于 DT
 *          ...
 * #小陀螺模式下击打
 * */
class AntiTop{
    int max_len;
    bool is_anti_top;
    int flag_size;
    std::queue<TopData> top_data;
    AntiTop()=default;
    AntiTop(const int &queue_len=10):max_len(queue_len),is_anti_top(0),flag_size(0){

    }
    void push(TopData & data){
        if(top_data.size()<max_len)
            top_data.push(data);
        else{
            top_data.pop();
            top_data.push(data);
        }
    }

    void push (double d_time ,double d_yaw ,double d_pitch,double d_rect_angle ,cv::Mat rotate_mat ){
        TopData data(d_time,d_yaw,d_pitch,d_rect_angle,rotate_mat);
        if(top_data.size()<max_len)
            top_data.push(data);
        else{
            top_data.pop();
            top_data.push(data);
        }
    }

    void clear(){
        // top_data.clear();
        is_anti_top= false;
        flag_size=0;
    }

    bool isAntiTop(){
        int size=top_data.size();
        if(size<max_len)
            return false;
        //判断队列中符合小陀螺判定的数量

//         for(auto data:top_data){
// //            if(data.dt)
//         }

    }

};
#endif //MAIN_ANTITOP_H
