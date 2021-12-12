#ifndef SOLVE_H_
#define SOLVE_H_
#include<opencv2/core/core.hpp>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/calib3d.hpp>
#include "../armor/Armor.h"
#include "./predict.h"
#include "../other/switch.h"


typedef struct SolverParam
{   
    cv::Mat camera_matrix=cv::Mat(3,3,CV_32F);
    cv::Mat dist_coeffs=cv::Mat(1,5,CV_32F);
    float small_armor_boxes_real_height =70.0;
    float small_armor_boxes_real_width =130.0;
    float big_armor_boxes_real_height =60.0;
    float big_armor_boxes_real_width =210.0;
    void readParam();  
}SolverParam;

class PnPSolver{
   public:
        //! 解算参数
        SolverParam _param;
        //! 侧航角（x/z）
        float yaw=0;
        //! 俯仰角（y/z）
        float pitch=0;
        //! 距离
        float distance=3;
        //! 
        cv::Point2f armor_points[4];

        //！ 旋转矩阵
        cv::Mat _rVec;
        //！ 平移矩阵
        cv::Mat _tVec;
    public:
        //！ 装甲板世界坐标（人为设置）
        std::vector<cv::Point3f> _3D_points;
        //！ 装甲板二维坐标
        std::vector<cv::Point2f> _2D_points;
        //！ 目标装甲板的中心点坐标
        cv::Point2f center_point;


        //! 装甲板大小（大装甲或小装甲）
        enum boxSize{
            small=0,
            big=1,
            unknow=2
        }big_small;//装甲板大小

        /**
         * @brief 设置2D点集
        */
        void set2DPoints();

        /**
         * @brief 设置3D点集
        */
        void set3DPoints();

        /**
         * @brief 进行解算
        */
        bool solve();
        // PnPSolver(const SolverParam &param,Predict &kalman_predict);
        
        /**
         * @brief 进行解算
        */
        PnPSolver(const SolverParam &param);
        PnPSolver(){}
        void setMode(const ArmorBox &armor);//设置参数
    private:
};

#endif





    

