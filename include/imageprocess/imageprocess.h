#ifndef IMAGEPROCESS_H_
#define IMAGEPROCESS_H_
#include <time.h>
#include <queue>
#include <iostream>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include "../Finder.h"
#include "../mercure/mercure_driver.h"
#include "../other/switch.h"
#include "../other/additions.h"
#include "../solve/predict.h"

using namespace std;
using namespace cv;

//! 云台数据
struct GimblaPose
{
    double gimbla_yaw = 0;
    double gimbla_pitch = 0;
    double gimbla_roll = 0;
    double timestamp = 0;
};

//!
struct Mat_t
{
    cv::Mat image;
    GimblaPose gimbla_pose;
    double mat_timestamp = 0;
};


class ImageProcess
{
private:
    mutex m_lock;
    //！ 图像队列中
    queue<Mat_t> image_queue;
    //！ 取图片的标识
    volatile int pro_id;
    volatile int con_id;
    //！ 最大接受队列数 为3
    const int BUFFER;
    //！ 自瞄器
    Finder armor_finder;
    //！ 预测器
    Predict predict;
    //! 电控与视觉共同的时间戳
    double start_time = 0;
    //! 是否需要重新对齐时间戳
    double need_reset_time = 0;
    //! 云台的历史数据
    std::vector<GimblaPose> history_gimbla_pose;
    //! 计时用于调试
    double runtime;
    double last_time = 0;
    double pre_time = 0;

public:
    /**
     * @brief  从相机得到图片
    */
    void image_producer();

    /**
    * @brief  进行图像处理并进行决策解算
    */
    void image_consumer();

    /**
    * @brief  串口部分
    */
    void send_data();

    /**
    * @brief  接受信息  
    */
    void receive_data();

    /**
    * @brief  将相对坐标转换成陀螺仪坐标系下的绝对坐标
    * @param  x  相对坐标系下x
    * @param  y  相对坐标系下y
    * @param  z  相对坐标系下z
    * @param  pose  相机姿态
*/
    void coord_chang(double &x, double &y, double &z, GimblaPose &pose);

    /**
    * @brief  得到录制视频的名字
*/
    String GetVedioName();
    /**
    * @brief  对云台数据进行插值
*/
    bool interpolateRobotPose(Mat_t &image_t);

public:
    /**
    * @brief  进行初始化
    * @param  _pro_id  当前接受到的帧数
    * @param  _con_id  已处理的帧数 
    */
    ImageProcess(int _pro_id, int _con_id, int &color, Serial &serial, SolverParam &solver_param);
    ImageProcess() = default;
};

#endif