#include "../../include/Finder.h"

/**
 * @brief  载入装甲板数据
 * */
void ArmorParam::loadParam()
{
    string file_name = "../include/armor/Armor.xml";
    cv::FileStorage fs(file_name, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "load armor param failed" << std::endl;
    }
    else
    {
        fs["lights_angle_max_dif"] >> lights_angle_max_dif;
        fs["lights_height_max_dif"] >> lights_height_max_dif;
        fs["lights_center_min_ratio"] >> lights_center_min_ratio;
        fs["lights_center_max_ratio"] >> lights_center_max_ratio;
        fs["lights_lenght_min_ratio"] >> lights_lenght_min_ratio;
        fs["lights_lenght_max_ratio"] >> lights_lenght_max_ratio;
        fs["same_lights_max_dis"] >> same_lights_max_dis;
        fs["light_min_wihe_ratio"] >> light_min_wihe_ratio;
        fs["light_max_wihe_ratio"] >> light_max_wihe_ratio;
        fs["light_min_area_ratio"] >> light_min_area_ratio;
        fs["light_bigger_area_ratio"] >> light_bigger_area_ratio;
        fs["blue_light_min_threshold"] >> blue_light_min_threshold;
        fs["red_light_min_threshold"] >> red_light_min_threshold;
        fs["gray_light_min_threshold"] >> gray_light_min_threshold;

        std::cout << "blue_light_min_threshold  " << blue_light_min_threshold << std::endl;
    }
}

/**
 * @brief 构造函数
 * @param blobs 一对灯条
 * @param pos 拟合装甲板的矩形
 * @param a  轮廓与外接矩形面积比
 * 
 */
ArmorBox::ArmorBox(const cv::Rect &pos, const LightBlobs &blobs, int color, int id) : rect(pos), light_Blobs(blobs), box_color(color), size(ARMORSIZE::SMALL)
{
    setPoints();
};

/**
 * @brief 得到装甲板中心坐标
 */
cv::Point2f ArmorBox::getCenter() const
{
    return cv::Point2f(rect.x + rect.width / 2, rect.y + rect.height / 2);
    /*
    */
}

/**
 * @brief 确定四个角点
 */
void ArmorBox::setPoints()
{
    try
    {
        cv::Point2f t_left[4];
        cv::Point2f t_right[4];
        if (light_Blobs.size() != 2)
        {
            this->f_points[0] = cv::Point2f(IMAGE_CENTER_X - 1, IMAGE_CENTER_Y - 1);
            this->f_points[0] = cv::Point2f(IMAGE_CENTER_X - 1, IMAGE_CENTER_Y + 1);
            this->f_points[0] = cv::Point2f(IMAGE_CENTER_X + 1, IMAGE_CENTER_Y + 1);
            this->f_points[0] = cv::Point2f(IMAGE_CENTER_X + 1, IMAGE_CENTER_Y - 1);
        }
        else
        {

            light_Blobs.at(0).rect.points(t_left);
            light_Blobs.at(1).rect.points(t_right);
            if (light_Blobs.at(0).rect.size.width < light_Blobs.at(0).rect.size.height)
            { //灯条左偏
                this->f_points[0] = (t_left[1] + t_left[2]) / 2.0;
                this->f_points[3] = (t_left[0] + t_left[3]) / 2.0;
            }
            else
            {
                this->f_points[0] = (t_left[3] + t_left[2]) / 2.0;
                this->f_points[3] = (t_left[0] + t_left[1]) / 2.0;
            }
            if (light_Blobs.at(1).rect.size.width < light_Blobs.at(1).rect.size.height)
            { //灯条左偏
                this->f_points[1] = (t_right[2] + t_right[1]) / 2.0;
                this->f_points[2] = (t_right[0] + t_right[3]) / 2.0;
            }
            else
            {
                this->f_points[1] = (t_right[2] + t_right[3]) / 2.0;
                this->f_points[2] = (t_right[0] + t_right[1]) / 2.0;
            }
        }
    }
    catch (exception e)
    {
        std::cout << "init points error : " << e.what() << std::endl;
        this->f_points[0] = cv::Point2f(IMAGE_CENTER_X - 1, IMAGE_CENTER_Y - 1);
        this->f_points[0] = cv::Point2f(IMAGE_CENTER_X - 1, IMAGE_CENTER_Y + 1);
        this->f_points[0] = cv::Point2f(IMAGE_CENTER_X + 1, IMAGE_CENTER_Y + 1);
        this->f_points[0] = cv::Point2f(IMAGE_CENTER_X + 1, IMAGE_CENTER_Y - 1);
    }
}

/**
 * @brief 得到灯条中心距
 */
double ArmorBox::getBlobDistance() const
{
    if (light_Blobs.size() == 2)
    {
        auto &x = light_Blobs[0].rect.center;
        auto &y = light_Blobs[1].rect.center;
        return sqrt((x.x - y.x) * (x.x - y.x) + (x.y - y.y) * (x.y - y.y));
    }
    else
        return 100;
}

/**
 * @brief 得到灯条长度与中心距的比值
 */
double ArmorBox::lengthDistanceRatio() const
{
    if (light_Blobs.size() == 2)
    {
        return MAX(light_Blobs[0].length, light_Blobs[1].length) / getBlobDistance();
    }
    else
    {
        return 100;
    }
}

/**
 * @brief 获得装甲板旋转角
 */
double ArmorBox::getAngle() const
{
    double realRatio;
    if (id == 0 || id == 1 || id == 6 || id == 7)
    {
        realRatio = 60 / 210.0;
    }
    else
    {
        realRatio = 70 / 130.0;
    }
    return acosf64(lengthDistanceRatio() / realRatio);
}

/**
 * @brief 运算符重载，用于装甲板的优先级比较
 */
bool ArmorBox::operator<(const ArmorBox &box) const
{
    //id={"base", "hero", "engineer", "infantry3", "infantry4", "infantry5", "sentry", "outpost", "error101", "error010", "error111"};
    int score[11] = {5, 4, 1, 2, 2, 2, 3, 3, 0, 0, 0};
    if (score[id] != score[box.id] && id != -1 && box.id != -1)
        return score[id] < score[box.id];
    auto d1 = (rect.x - IMAGE_CENTER_X) * (rect.x - IMAGE_CENTER_X) + (rect.y - IMAGE_CENTER_Y) * (rect.y - IMAGE_CENTER_Y);
    auto d2 = (box.rect.x - IMAGE_CENTER_X) * (box.rect.x - IMAGE_CENTER_X) + (box.rect.y - IMAGE_CENTER_Y) * (box.rect.y - IMAGE_CENTER_Y);
    return d1 < d2;
}
