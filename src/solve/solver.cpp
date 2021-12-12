#include "../../include/solve/solver.h"

/**
 * @brief 初始化内参
 */
PnPSolver::PnPSolver(const SolverParam &param) : _param(param)
{
    // std::cout<<"solver init sucessfully "<<std::endl;
}

void SolverParam::readParam()
{
    cv::FileStorage fs("../include/mercure/mercure_pnp.xml", cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "cant not open solver file" << std::endl;
    }
    else
    {
        fs["Camera_Matrix"] >> camera_matrix;
        fs["Distortion_Cofficients"] >> dist_coeffs;
        fs["small_armor_boxes_real_height"] >> small_armor_boxes_real_height;
        fs["small_armor_boxes_real_width"] >> small_armor_boxes_real_width;
        fs["big_armor_boxes_real_height"] >> big_armor_boxes_real_height;
        fs["big_armor_boxes_real_width"] >> big_armor_boxes_real_width;
        // std::cout<<"big_armor_boxes_real_width  "<<big_armor_boxes_real_width<<std::endl;
        std::cout << "read camera param sucessful" << std::endl;
    }
}

/**
 * @brief 传入矩形，并判断解算模式
 */
void PnPSolver::setMode(const ArmorBox &armor)
{
    for (int i = 0; i < 4; i++)
        armor_points[i] = armor.f_points[i];
    center_point = cv::Point2f(armor.rect.x + armor.rect.width / 2.0, armor.rect.y + armor.rect.height / 2.0);
    if (armor.size == ArmorBox::ARMORSIZE::BIG)
        big_small = big;
    else
        big_small = small;
};

/**
 * @brief 根据不同模式进行解算
 */
bool PnPSolver::solve()
{
    set2DPoints();
    set3DPoints();
    cv::solvePnP(_3D_points, _2D_points, _param.camera_matrix, _param.dist_coeffs, _rVec, _tVec, false, cv::SOLVEPNP_ITERATIVE);
    distance = pow(_tVec.at<double>(0, 0) * _tVec.at<double>(0, 0) + _tVec.at<double>(1, 0) * _tVec.at<double>(1, 0) + _tVec.at<double>(2, 0) * _tVec.at<double>(2, 0), 0.5) / 1000;
    yaw = atan((_tVec.at<double>(0, 0)) / (_tVec.at<double>(2, 0) + 10)) * 180.0 / CV_PI;
    pitch = atan(-(_tVec.at<double>(1, 0) + 60) / (_tVec.at<double>(2, 0) + 10)) * 180.0 / CV_PI;
#ifdef PNPDEBUG
    if (show_yaw_pitch_tvec)
    {
        std::cout << "FOUR POINT       PITCH:  " << pitch << std::endl;
        std::cout << "FOUR POINT         YAW:  " << yaw << std::endl;
        std::cout << "FOUR POINT       t_VEC:  " << _tVec << std::endl
                  << std::endl;
    }
#endif
    return true;
}

/**
 * @brief  设置二维点集
 */
void PnPSolver::set2DPoints()
{
    _2D_points.clear();
    //设置 图片取样偏移量
    _2D_points.push_back(armor_points[0] + cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS));
    _2D_points.push_back(armor_points[1] + cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS));
    _2D_points.push_back(armor_points[2] + cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS));
    _2D_points.push_back(armor_points[3] + cv::Point2f(IMAGE_X_DIS, IMAGE_Y_DIS));
}
/**
 * @brief 设置三维点集
 */
void PnPSolver::set3DPoints()
{
    _3D_points.clear();
    int width, height;
    if (big_small)
    {
        width = _param.big_armor_boxes_real_width;
        height = _param.big_armor_boxes_real_height;
    }
    else
    {
        width = _param.small_armor_boxes_real_width;
        height = _param.small_armor_boxes_real_height;
    }
    _3D_points.push_back(cv::Point3f(-width / 2.0, -height / 2.0, 0));
    _3D_points.push_back(cv::Point3f(width / 2.0, -height / 2.0, 0));
    _3D_points.push_back(cv::Point3f(width / 2.0, height / 2.0, 0));
    _3D_points.push_back(cv::Point3f(-width / 2.0, height / 2.0, 0));
}
