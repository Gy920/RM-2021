#ifndef KALMAN_H_
#define KALMAN_H_
#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "../other/switch.h"
#include "Eigen/Dense"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Core>


	class kalmanFilter
	{
	public:

		struct Param
		{
			cv::Mat m_A;
			cv::Mat m_P;
			cv::Mat m_H;
			cv::Mat m_R;
			cv::Mat m_Q;
			double delay;
			void readParam(){
				cv::FileStorage fs;
				fs.open("../include/solve/Kalman.xml",cv::FileStorage::READ);
				if(!fs.isOpened()){
					flag=false;
					std::cout<<"read Predict param failed "<<std::endl; 
					return ;
				}
				fs["A"]>>m_A;
				fs["P"]>>m_P;
				fs["H"]>>m_H;
				fs["R"]>>m_R;
				fs["Q"]>>m_Q;
				fs["delay"]>>delay;
				flag=true;

			}
			bool flag=false;
			/* data */
		}Mat_param;
		
		//！ 状态方程维数
		int stateSize; 
		//！ 测量参数个数   
		int measSize;  
		//! 协方差矩阵
		Eigen::MatrixXd P; 
		//! 测量矩阵
		Eigen::MatrixXd H; 
		//! 测量误差
		Eigen::MatrixXd R; 
		//! 预测误差   
		Eigen::MatrixXd Q;  
	public:
		//! 状态转移矩阵
		Eigen::MatrixXd A; 
	public:
		kalmanFilter();
		~kalmanFilter(){}
		/**
		* @brief 初始化kalman预判参数
		 */
		void init(int state, int means, Eigen::MatrixXd& _A, Eigen::MatrixXd& _P, Eigen::MatrixXd& _R, Eigen::MatrixXd& _Q, Eigen::MatrixXd& _H);
		/**
		* @brief kalman预测
		* @输入输出:Eigen::VectorXd &x 状态向量
		*/
		void predict(Eigen::VectorXd &x);
		/**
		* @brief kalman更新
		* @输入: Eigen::VectorXd z_meas 测量参数向量
		* @输入输出:Eigen::VectorXd &x 状态向量
		 */
		void update(Eigen::VectorXd &x, Eigen::VectorXd z_meas);
		void reInit(int stateSize,int measureSize);
	};
	
    class Predict{
    public:
        Predict();
        ~Predict(){}
        /**
         * @brief 用于矫正发送的数据
        */
        void correct(double &yaw,double &pitch);

        double correctPitch( double &pitch,const double &distance,const double &speed);

        /**
         * @brief 利用角度进行预测
         * @param   yaw yaw和pitch为转换成陀螺仪坐标系后的角度
         * @return 陀螺仪坐标系下的yaw和pitch
        */
        cv::Point2f angle_predict(float yaw,float pitch,float time,float distance ,float bullet_rate);

        /**
         * @brief 利用坐标进行预测
         * @param  x x,y,z是转换坐标系后的值
         * @return 陀螺仪坐标系下的yaw和pitch
        */
        cv::Point2f coord_prsdict(float _x,float _y,float _z,float time,float bullet_rate);
        
        //! 卡尔曼滤波器
        kalmanFilter KF;
        Eigen::VectorXd x;
    };
#endif
