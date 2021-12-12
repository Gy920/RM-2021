#ifndef ARMORBOX_H_
#define ARMORBOX_H_

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <string>
#include <cmath>
#include <map>
#include "../other/switch.h"

//！ 装甲板id到名称的map
extern std::string id2name[20];

struct ArmorParam
{
    float lights_angle_max_dif;
    float lights_height_max_dif;
    float lights_center_min_ratio;
    float lights_center_max_ratio;
    float lights_lenght_min_ratio;
    float lights_lenght_max_ratio;
    float same_lights_max_dis;
    float light_min_wihe_ratio;
    float light_max_wihe_ratio;
    float light_min_area_ratio;
    float light_bigger_area_ratio;
    float blue_light_min_threshold;
    float red_light_min_threshold;
    float gray_light_min_threshold;
    void loadParam();
};

class LightBlob
{
public:
    //! 包含灯条位置（旋转）
    cv::RotatedRect rect;
    //! 轮廓与外包矩形面积比
    double area_ratio;
    //! 长轴长度
    double length;
    //！ 灯条颜色 0为红色，1为蓝色
    int blob_color;

public:
    /**
     * @brief 构造函数
     * @param r 旋转矩形
     * @param ratio  轮廓与外包矩形面积比
     * @param color 灯条颜色 0为红色，1为蓝色
    */
    LightBlob(cv::RotatedRect &r, double ratio, int color) : rect(r), area_ratio(ratio), blob_color(color)
    {
        length = std::max(rect.size.height, rect.size.width);
    };
    LightBlob() = default;
};
typedef std::vector<LightBlob> LightBlobs;

/********************* 装甲板类定义　************************/
class ArmorBox
{
public:
    //! 装甲板正矩形
    cv::Rect2d rect;
    //! 一对灯条
    LightBlobs light_Blobs;
    //！ 装甲板颜色 0为红色，1为蓝色
    int box_color;
    //float score

    //! 距离及装甲板大小
    enum ARMORSIZE
    {
        BIG = 0,
        SMALL = 2
    };
    ARMORSIZE size ;
    //！ 装甲板id
    int id = -1;
    // 装甲板灯条四个角点，左上为起点，顺时针
    cv::Point2f f_points[4];

    /**
     * @brief  构造函数
     * @param pos 装甲板正矩形
     * @param blobs 一对灯条
     * @param color  装甲板颜色 0为红色，1为蓝色
     * @param id    装甲板id
    */
    explicit ArmorBox(const cv::Rect &pos = cv::Rect2d(), const LightBlobs &blobs = LightBlobs(), int color = 0, int id = -1);

    /**
     * @brief  得到装甲板中心坐标
    */
    cv::Point2f getCenter() const;

    /**
     * @brief  得到灯条中心距
    */
    double getBlobDistance() const;

    /**
     * @brief  得到长度和中心距的比值
    */
    double lengthDistanceRatio() const;


    /**
     * @brief  装甲板优先级比较
    */
    bool operator<(const ArmorBox &box) const;
    /**
     * @brief  得到装甲板旋转角度
    */
    double getAngle() const;

    void setPoints(); 
};
typedef std::vector<ArmorBox> ArmorBoxes;

#endif
