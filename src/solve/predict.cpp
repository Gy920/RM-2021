#include "../../include/solve/predict.h"
#include <math.h>

kalmanFilter::kalmanFilter()
{
    Mat_param.readParam();
}

void kalmanFilter::init(int state, int meas, Eigen::MatrixXd &_A, Eigen::MatrixXd &_P, Eigen::MatrixXd &_R, Eigen::MatrixXd &_Q, Eigen::MatrixXd &_H)
{
    A = _A;
    P = _P;
    R = _R;
    Q = _Q;
    H = _H;
}

void kalmanFilter::predict(Eigen::VectorXd &x)
{
    x = A * x;
    Eigen::MatrixXd A_T = A.transpose();
    P = A * P * A_T + Q;
}

void kalmanFilter::update(Eigen::VectorXd &x, Eigen::VectorXd z_meas)
{
    Eigen::MatrixXd temp1, temp2, Ht;
    Ht = H.transpose();
    temp1 = H * P * Ht + R;
    temp2 = temp1.inverse(); //(H*P*H'+R)^(-1)
    Eigen::MatrixXd K = P * Ht * temp2;
    Eigen::VectorXd z = H * x;
    x = x + K * (z_meas - z);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(stateSize, stateSize);
    P = (I - K * H) * P;
}

cv::Point2f Predict::angle_predict(float yaw, float pitch, float time, float distance, float bullet_rate)
{
    Eigen::VectorXd z(2);
    // 由于测距误差在0.1*distance 之内 人为进行矫正
    double m_distance=1.10*distance;
    // 弹道补偿时间+通信延迟+发弹延迟
    double time2=1.25*m_distance/bullet_rate+2.0*SETIAL_DELAY+KF.Mat_param.delay;
    // Pitch 轴上不希望云台太抖
    double time3=0.3*distance/bullet_rate;//+0.0050;
    double dt = 1*time;
    // std::cout<<dt<<std::endl;
    if(fabs(distance*x(2)/180*CV_PI)>0.6){
        //由 distance*v_angle 得到速度
        // 大于阈值证明需要重置滤波器
        //  std::cout<<"v_yaw :"<<fabs(distance*x(2)/180*CV_PI)<<std::endl;
        x(0) = (double)yaw;
        x(1) = (double)pitch;
        x(2) = 0;
        x(3) = 0;
        KF.reInit(KF.stateSize,KF.measSize);
        z << (double)yaw, (double)pitch;
    }else{
        z << (double)yaw, (double)pitch;
        Eigen::MatrixXd _A(4, 4);
        _A << 1, 0, dt, 0,
            0, 1, 0, dt,
            0, 0, 1, 0,
            0, 0, 0, 1;
        KF.A = _A;
    }
    KF.predict(x);
    KF.update(x, z);
    //这里时间需要细考虑
    // std::cout<<"angle "<<x(0)<<"  v  "<<x(2)<<std::endl<<std::endl;
    //利用角匀速模型进行预测
    double predictYaw = x(0)+time2*x(2);
    double predictPitch = (x(1)+time3*x(3)) ;

    predictPitch= correctPitch(predictPitch,distance,bullet_rate);
    //修正数据
    correct(predictYaw,predictPitch);
    return cv::Point2f(predictYaw, predictPitch);
    // return cv::Point2f(x(0),x(1)-0.5);
}

cv::Point2f Predict::coord_prsdict(float _x, float _y, float _z, float time, float bullet_rate)
{
    double yaw = atan(_x / _z) * 180 / CV_PI;
    double pitch = atan(_y / _z) * 180 / CV_PI;
    double pre_yaw = atan(x(0) / _z) * 180 / CV_PI;
    double pre_pitch = atan(x(2) / _z) * 180 / CV_PI;
    double dt;
    if (abs(pre_yaw - yaw) > 180)
    {
        x(0) = (double)_x;
        x(2) = 0;
    }
    if (abs(pre_pitch - pitch) > 180)
    {
        x(1) = (double)_y;
        x(3) = 0;
    }

    Eigen::VectorXd z(2);

    z << (double)_x, (double)_y;
    KF.update(x, z);

    Eigen::MatrixXd _A(4, 4);
    int distance = pow(_x * _x + _y * _y + _z * _z, 0.5);
    float time2 = distance / bullet_rate/1000;
    dt = time + time2 + 0.0015;
    _A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;
    KF.A = _A;
    KF.predict(x);
    //这里12为发送时间需要细考虑
    double predict_x = x(0);//+time2*x(2);
    double predict_y = x(1);//+time2*x(3);
    double predict_yaw = atan(predict_x / _z) * 180 / CV_PI;
    double predict_pitch = atan(predict_y / _z) * 180 / CV_PI;
    predict_pitch= correctPitch(predict_pitch,distance,bullet_rate);
    correct(predict_yaw,predict_pitch);
    return cv::Point2f((float)predict_yaw, (float)predict_pitch);
}

void Predict::correct(double &yaw,double &pitch){
    if(yaw<-180)
        yaw+=360;
    else if(yaw>180)
        yaw-=360;
    if(pitch<-180)
        pitch+=360;
    else if(yaw>180)
        pitch-=360;
}

double Predict::correctPitch(double &pitch,const double &distance,const double &speed){
   // 利用多段采样，进行多段插值，拟合抛物线
    double m_pitch=pitch;
    double delt =1.25;
    double delt_m=0.10;
    // return pitch;   
    //注意连接点需连续
    if (distance<1)//
        m_pitch=0.90*pitch+0.85+delt_m;
    // else if (distance<4.5)
    //     m_pitch=1.05*pitch+0.80+delt_m;
    else if (distance<3)
        m_pitch=1.07*pitch+0.76+delt_m;
        //m_pitch=1.65*pitch+0.65;
    else if (distance<3.8)
        m_pitch=1.04*pitch+0.85+delt_m;
        //m_pitch=2.05*pitch+0.60;
    else if (distance<5)
        m_pitch=0.99*pitch+1.04+delt_m;
    else if (distance<6)
        m_pitch=1.15*pitch+0.455+delt_m;
    else
        m_pitch=1.25*pitch; 
    return delt*m_pitch;
}

Predict::Predict()
{
    KF.stateSize = 4;
    KF.measSize = 2;
    KF.reInit(KF.stateSize,KF.measSize);
    x.resize(KF.stateSize);
    x << 0, 0, 0, 0;
}


void kalmanFilter::reInit(int stateSize,int measureSize){
    /*x: 0 yaw 
         1 pitch 
         2 v_yaw
         3 v_pitch
    */
    Eigen::MatrixXd A(stateSize, stateSize);
    Eigen::MatrixXd H(measureSize, stateSize);
    Eigen::MatrixXd P(stateSize, stateSize);
    Eigen::MatrixXd Q(stateSize, stateSize);
    Eigen::MatrixXd R(measureSize, measureSize);

    if(Mat_param.flag){
        cv::cv2eigen(Mat_param.m_R, R); // cv::Mat 转换成 Eigen::Matrix
        cv::cv2eigen(Mat_param.m_A, A);
        cv::cv2eigen(Mat_param.m_H, H);
        cv::cv2eigen(Mat_param.m_Q, Q);
        cv::cv2eigen(Mat_param.m_P, P);
    }else{
        
        //默认A
        A << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
        //默认H
        H << 1, 0, 0, 0,
            0, 1, 0, 0;
        //默认P
        P << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 40, 0,
            0, 0, 0, 40;
        //默认Q
        Q << 5, 0, 0, 0,
            0, 5, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;
        //默认R
        R << 200, 0,
            0, 200;
   }

    init(stateSize, measureSize, A, P, R, Q, H);
}