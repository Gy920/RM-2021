#include "../../include/imageprocess/imageprocess.h"
ImageProcess::ImageProcess(int _pro_id, int _con_id,
                           int &color, Serial &serial,
                           SolverParam &solver_param)
    : pro_id(_pro_id),
      con_id(_con_id),
      BUFFER(3),
      armor_finder(color, serial, "../param/", solver_param)
{
    runtime = getTickCount();
    std::cout << "1 ";
}

void ImageProcess::image_producer()
{
    camera::MercureDriver cap;
    // cv::VideoCapture cap;
    // VideoCapture cap("../../../NEWW/video/8-112No4.mp4");
#ifdef WRITEVIDEIO
    string outpath;
    outpath = GetVedioName();
    VideoWriter writer(outpath, VideoWriter::fourcc('D', 'I', 'V', 'X'), 40, Size(IMAGE_WIDTH, IMAGE_HEIGHT));
#endif
    Mat frame;
    Mat_t image_t;
    // frame.create(IMAGE_HEIGHT,IMAGE_WIDTH, CV_8UC3);
    while (true)
    {
        frame.create(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
        while (pro_id - con_id >= BUFFER)
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        m_lock.lock();
        //记录图片产生的时间点
        double timestamp = (getTickCount() - start_time) / cv::getTickFrequency();
        cap >> frame;
        m_lock.unlock();
        if (frame.empty() == true)
        {
            // std::cout<<"The frame is empty"<<std::endl;
            continue;
        }
        image_t.mat_timestamp = timestamp;
        image_t.image = frame;
        image_queue.push(image_t);
        pro_id++;

#ifdef WRITEVIDEIO
        writer.write(frame);
#endif
    }
}

void ImageProcess::image_consumer()
{
    Mat frame;
    while (true)
    {
        while (pro_id <= con_id)
            std::this_thread::sleep_for(std::chrono::milliseconds(4));
        m_lock.lock();
        // 取图和姿态
        Mat_t image_t = image_queue.front();
        image_queue.pop();
        con_id++;
        m_lock.unlock();
        armor_finder.run(image_t.image);
        std::cout<<"solver pitch"<<armor_finder.solver.pitch<<std::endl;
        std::cout<<"solver yaw"<<armor_finder.solver.yaw<<std::endl;
        if (interpolateRobotPose(image_t))
        {
            //  m_lock.unlock();
            // armor_finder.run(image_t.image);
            // cout<<"gimbal time          :"<<image_t.mat_timestamp<<std::endl;
            // // //这里转换成绝对坐标系
            double pitch = (armor_finder.solver.pitch + image_t.gimbla_pose.gimbla_pitch);
            double yaw = (armor_finder.solver.yaw + image_t.gimbla_pose.gimbla_yaw);
            // 图像处理并解算成功并识别到目标则进行预测
            if (armor_finder.shoot_code == FIND_NO_SHOOT)
            {
                //接收图片到处理好图片用的时间
                double time = (getTickCount() - start_time) / cv::getTickFrequency() - image_t.mat_timestamp;
                //std::cout<<"solver pitch"<<armor_finder.solver.pitch<<std::endl;
                //cout<<"pre  pitch  yaw  " <<pitch<<"  "<<yaw<<endl;
                //cout<<"pose pitch yaw  roll :"<<image_t.gimbla_pose.gimbla_pitch<<"  "<<image_t.gimbla_pose.gimbla_yaw<<"   "<<image_t.gimbla_pose.gimbla_roll<<endl;
                /*******利用角度进行预测*******/
                cv::Point2f yaw_pitch = predict.angle_predict(yaw, pitch, time, armor_finder.solver.distance, armor_finder.bullet_speed);
                //更改为相对角度
                armor_finder.solver.yaw = -(yaw_pitch.x - image_t.gimbla_pose.gimbla_yaw);
                armor_finder.solver.pitch = (yaw_pitch.y - image_t.gimbla_pose.gimbla_pitch);
                //判断是否能发射
                if (abs(armor_finder.solver.pitch) < 0.5 && abs(armor_finder.solver.yaw) < 0.8)
                {
                    armor_finder.shoot_code = FIND_SHOOT;
                }
                // armor_finder.solver.yaw = -yaw_pitch.x;
                // armor_finder.solver.pitch = yaw_pitch.y * 1.0;
                // cout<<"end   pitch  yaw     :" <<armor_finder.solver.pitch<<"  "<<armor_finder.solver.yaw<<endl;
                //根据电控处理手段和陀螺仪安装方向认为改变X。Y，Z的正方向
                /***************************/
            }
        }
        else
        {
            //  m_lock.unlock();
        }
#ifdef show_debug
        if (show_target_armor_class)
        {
            cv::namedWindow("target_armor_class", cv::WINDOW_NORMAL);
            showArmorBoxClass("target_armor_class", image_t.image, armor_finder.target_box);
            cv::waitKey(waite_time);
        }
#endif
    }
}

void ImageProcess::send_data()
{
    uint8_t last_shoot_code = NO_FIND_NO_SHOOT;
    //标志位计数器
    int idex = 0;
    while (1)
    {
        ControlData controlData;
        m_lock.lock();
        int16_t pi = armor_finder.solver.pitch * 32768 / 180;
        int16_t ya = armor_finder.solver.yaw * 32768 / 180;
        uint8_t shoot_code = armor_finder.shoot_code;
        //重置标志位
        armor_finder.shoot_code = NO_END_SOLVER;
        m_lock.unlock();
        if (shoot_code == NO_END_SOLVER) // 图像处理并解算成功并识别到目标
        {
            if (idex++ > 20)
            { //如果连续N次未解算完。更新上次标志位为不发射
                last_shoot_code = NO_FIND_NO_SHOOT;
                idex = 0;
            }
            shoot_code = last_shoot_code; //用上次的标志位发送
        }
        else
        {
            last_shoot_code = shoot_code;
            idex = 0;
        }
        controlData.if_find = (uint8_t)shoot_code;
        controlData.pitch_dev1 = (uint8_t)((pi >> 8) & 0xff);
        controlData.pitch_dev2 = (uint8_t)(pi & 0xff);
        controlData.yaw_dev1 = (uint8_t)((ya >> 8) & 0xff);
        controlData.yaw_dev2 = (uint8_t)(ya & 0xff);

        int sum = (controlData.pitch_dev1 + controlData.pitch_dev2 + controlData.yaw_dev1 + controlData.yaw_dev2) % 10;
        controlData.sum = (uint8_t)sum;
        if (armor_finder.serial.tryControl(controlData, chrono::milliseconds(1)) != Serial::OJBK)
        {
            //  cout<<"not sent"<<endl;
        }
        else
        {
            // std::cout << "send " << std::endl;
        }
    }
}

void ImageProcess::receive_data()
{
    //云台姿态
    GimblaPose gimbla_pose;
    start_time = cv::getTickCount();
    float yaw = 0, pitch = 0, roll = 0;
    double timestamp = 0, lastTimestamp = -1;
    while (0)
    {
        FeedBackData feedBackData;
        //接收数据
        if (armor_finder.serial.tryFeedBack(feedBackData, chrono::milliseconds(1)) == Serial::OJBK)
        {

            pitch = (float)((int16_t)(feedBackData.pitch_dev2 << 8 | feedBackData.pitch_dev1) / 32768.0 * 180);
            yaw = (float)((int16_t)(feedBackData.yaw_dev2 << 8 | feedBackData.yaw_dev1) / 32768.0 * 180.0);
            roll = (float)((int16_t)(feedBackData.roll_dev2 << 8 | feedBackData.roll_dev1) / 32768.0 * 180.0);
            int32_t ms = ((uint32_t)feedBackData.ms_L << 16) | ((uint32_t)feedBackData.ms_M << 8) | (((uint32_t)feedBackData.ms_H));
            
            int color = ((feedBackData.u_flag & 0x30) >> 4);
            int mode = ((feedBackData.u_flag >> 6));
            float pitch_speed = (float)((int16_t)(feedBackData.pitch_speed_dev2 << 8 | feedBackData.pitch_speed_dev1) / 32768.0 * 2000.0);
            float yaw_speed = (float)((int16_t)(feedBackData.yaw_speed_dev2 << 8 | feedBackData.yaw_speed_dev1) / 32768.0 * 2000.0);
            if (abs(timestamp - (cv::getTickCount() - start_time) / cv::getTickFrequency()) > 0.6 * LIMIT_TIME)
            {
                need_reset_time++;
            }
            else
            {
                need_reset_time = 0;
            }
            if (timestamp != 0)
            {
                if (lastTimestamp < timestamp && need_reset_time <= 3)
                {
                    history_gimbla_pose.push_back(gimbla_pose);
                    lastTimestamp == timestamp;
                }
                /*同步逻辑*/
                if (need_reset_time >= 3) //&& EnableSynchronizedTime)
                {
                    //同步主控时间轴，主控读取时间发回，做差求出开始的系统时间
                    start_time = cv::getTickCount() - (((double)timestamp + SETIAL_DELAY) * (double)cv::getTickFrequency());
                    // cout<<"  start_time "<<start_time<<endl;
                    // history_gimbla_pose.clear();
                    need_reset_time = 0;
                    //cout << "init TimeStamp success" << endl;
                }
            }
        }
        else
        {
            // cout << "recieva data false" << endl;
        }
    }
}

String ImageProcess::GetVedioName()
{
    time_t nowtime;
    struct tm *p;
    time(&nowtime);
    p = localtime(&nowtime);
    char _outpath[40];
    sprintf(_outpath, "../vedio/x%dmon-%dd-%dh-%dmin.avi", p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min);
    cout << _outpath << endl;
    return _outpath;
}

bool ImageProcess::interpolateRobotPose(Mat_t &image_t)
{
    double timestamp = image_t.mat_timestamp;
    int x = history_gimbla_pose.size();
    if (timestamp <= 0 || x <= 5)
    {
        // std::cout<<"the timestamp less than 0 || the size of poses too small "<<std::endl;
        return false;
    }
    /*寻找插值区间*/
    GimblaPose first_pose, second_pose, thrid_pose;
    int flag = 0;
    for (int j = 0; j < x; j++)
    {
        if (timestamp < history_gimbla_pose.at(j).timestamp)
        {
            flag = j;
            break;
        }
    }
    // x-1 x-2 flag 1 0
    if (1 <= flag && flag <= x - 2)
    {
        m_lock.lock();
        first_pose = history_gimbla_pose.at(flag - 1);
        second_pose = history_gimbla_pose.at(flag);
        thrid_pose = history_gimbla_pose.at(flag + 1);
        m_lock.unlock();
        /*限制两帧间的时间差*/
        if (thrid_pose.timestamp - first_pose.timestamp > 2 * LIMIT_TIME ||
            second_pose.timestamp - first_pose.timestamp > 1.3 * LIMIT_TIME ||
            thrid_pose.timestamp - second_pose.timestamp > 1.3 * LIMIT_TIME ||
            thrid_pose.timestamp - first_pose.timestamp <= 0 ||
            second_pose.timestamp - first_pose.timestamp <= 0 ||
            thrid_pose.timestamp - second_pose.timestamp <= 0)
        {
            // std::cout<<"Interpolate error1 :neighor pose's timestamp too big "<<std::endl;
            return false;
        }
    }
    else
    {
        // std::cout<<"Interpolate error2 :nums of poses too small "<<std::endl;
        return false;
    }
    /*中间量*/
    double d0 = (first_pose.timestamp - second_pose.timestamp) * (first_pose.timestamp - thrid_pose.timestamp);  //(x0-x1)*(x0-x2)
    double d1 = (second_pose.timestamp - first_pose.timestamp) * (second_pose.timestamp - thrid_pose.timestamp); //(x1-x0)*(x1-x2)
    double d2 = (thrid_pose.timestamp - first_pose.timestamp) * (thrid_pose.timestamp - second_pose.timestamp);  //(x2-x0)*(x2-x1)
    /*基函数*/
    double l_x0 = (timestamp - second_pose.timestamp) * (timestamp - thrid_pose.timestamp) / d0; //(x-x1)*(x-x2)/d0
    double l_x1 = (timestamp - first_pose.timestamp) * (timestamp - thrid_pose.timestamp) / d1;  //(x-x0)*(x-x2)/d1
    double l_x2 = (timestamp - first_pose.timestamp) * (timestamp - second_pose.timestamp) / d2; //(x-x0)*(x-x1)/d2
    /*采用抛物线插值方法对云台数据进行插值*/
    image_t.gimbla_pose.timestamp = timestamp;
    image_t.gimbla_pose.gimbla_yaw = l_x0 * first_pose.gimbla_yaw + l_x1 * second_pose.gimbla_yaw + l_x2 * thrid_pose.gimbla_yaw;
    image_t.gimbla_pose.gimbla_pitch = l_x0 * first_pose.gimbla_pitch + l_x1 * second_pose.gimbla_pitch + l_x2 * thrid_pose.gimbla_pitch;
    image_t.gimbla_pose.gimbla_roll = l_x0 * first_pose.gimbla_roll + l_x1 * second_pose.gimbla_roll + l_x2 * thrid_pose.gimbla_roll;

    // std::cout<<"timestamp    : "<<image_t.gimbla_pose.timestamp<<std::endl;
    // std::cout<<"gimbal_yaw   : "<<image_t.gimbla_pose.gimbla_yaw<<std::endl;
    // std::cout<<"gimbal_pitch : "<<image_t.gimbla_pose.gimbla_pitch<<std::endl;
    // std::cout<<"gimbal_roll  : "<<image_t.gimbla_pose.gimbla_roll<<std::endl;

    /*更新历史云台数据*/
    m_lock.lock();
    vector<GimblaPose>::iterator iter = history_gimbla_pose.begin();
    while (history_gimbla_pose.at(history_gimbla_pose.size() - 1).timestamp - history_gimbla_pose.at(0).timestamp > 16 * LIMIT_TIME && history_gimbla_pose.size() >= 25)
    {
        history_gimbla_pose.erase(iter);
        iter = history_gimbla_pose.begin();
    }
    m_lock.unlock();
    return true;
}

//利用坐标进行预测，需注意陀螺仪欧拉角的旋转方向，函数有问题
void ImageProcess::coord_chang(double &x, double &y, double &z, GimblaPose &pose)
{
    cout << "pre x y z" << x << " " << y << " " << z << endl;

    Eigen::MatrixXd A(3, 3), B(3, 3), C(3, 3);
    double cosp = cos(pose.gimbla_pitch), sinp = sin(pose.gimbla_pitch);
    double cosy = cos(pose.gimbla_yaw), siny = sin(pose.gimbla_yaw);
    double cosr = cos(pose.gimbla_roll), sinr = sin(pose.gimbla_roll);
    //ZYX
    //for roll
    A << 1, 0, 0,
        0, cosp, -sinp,
        0, sinp, cosp;
    //for yaw
    B << cosr, -sinr, 0,
        sinr, cosr, 0,
        0, 0, 1;
    //for pitch
    C << cosy, 0, -siny,
        0, 1, 0,
        siny, 0, cosy;

    Eigen::MatrixXd x1(3, 1);
    x1 << x, y, z;
    Eigen::MatrixXd x2(3, 1);
    x2 = (B * C * A).transpose() * x1;

    x = x2(0, 0);
    y = x2(1, 0);
    z = x2(2, 0);
    cout << "last x y z" << x << " " << y << " " << z << endl
         << endl;
}
