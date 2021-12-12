#include <iostream>
#include <math.h>

namespace offest{

double correctPitch( double &pitch,const double &yaw,const double &distance,const double &speed){
    const double CV_PI  =3.1415926;
    //相距高度
    double z=distance*sin(CV_PI*pitch/180);
    //xy 平面上的距离
    double xy=distance*cos(CV_PI*pitch/180);
    // 目标要达到的高度
    double target_height=z;
    //当前迭代指向的高度
    double temp_height=1.5*z;
    // 偏差
    double delta_height=temp_height-target_height;
    //迭代十次
    for(int i=15;i>0;i--){
        //当前的指向角度
        double angle=atan2(temp_height,xy); 
        // std::cout<<angle*180/CV_PI<<std::endl;     
        // 抛物线正向传播
        double v_x=speed*cos(angle);
        double v_y=speed*sin(angle);
        /*理想抛物线模型*/
        // double dt=xy/(v_x+0.00001);
        /*加入水平方向阻力的抛物线模型*/
        float k1=0.12;
        double dt=(exp(k1*xy)-1)/(k1*v_x);
        double real_height=v_y*dt-9.8*dt*dt/2.0;
        //更新偏差.
        delta_height=target_height-real_height;
        if(abs(delta_height)<0.0001){
            // std::cout<<"the count "<<15-i<<std::endl;
            break;
        }
        //更新指向
        temp_height+=delta_height;
        // std::cout<<"  temp height  "<<temp_height;
        // std::cout<<"  delta height  "<<delta_height<<std::endl;
    }
    double h_angle=1.0*atan2(temp_height,xy)*180/CV_PI+0.00;
    std::cout<<"  temp height   :"<<temp_height<<std::endl;
    std::cout<<"  delta height  :"<<delta_height<<std::endl;
    std::cout<<"  h_angle       :"<<h_angle<<std::endl;
    return h_angle;
}
}

int main(){
    double pitch=10,yaw=10,distance=2.9,speed=15;
    std::cout<<offest::correctPitch(pitch,yaw,distance,speed);
}