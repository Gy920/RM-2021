#include "../../include/Finder.h"

bool Finder::isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, int enemy_color)
{
    //判断颜色
    if (light_blob_i.blob_color != enemy_color || light_blob_j.blob_color != enemy_color)
    {
        return false;
    }

    //判断两个灯条长度比
    if ((light_blob_i.length / light_blob_j.length > armor_param.lights_lenght_max_ratio ||
         light_blob_i.length / light_blob_j.length < armor_param.lights_lenght_min_ratio))
    {
        return false;
    }

    //判断两个灯条的中心距及高度差
    double side_length, maxleght;
    cv::Point2f centers = light_blob_j.rect.center - light_blob_i.rect.center;
    side_length = sqrt(centers.ddot(centers));
    maxleght = fmax(light_blob_i.length, light_blob_j.length);
    if (side_length / maxleght > armor_param.lights_center_max_ratio ||
        side_length / maxleght < armor_param.lights_center_min_ratio ||
        abs(centers.y) > armor_param.lights_height_max_dif)
    {
        return false;
    }

    //判断两个灯条的角度（与y轴方向的角度）差
    float angle_i = light_blob_i.rect.size.width > light_blob_i.rect.size.height ? light_blob_i.rect.angle : light_blob_i.rect.angle - 90;
    float angle_j = light_blob_j.rect.size.width > light_blob_j.rect.size.height ? light_blob_j.rect.angle : light_blob_j.rect.angle - 90;
    if (abs(angle_i - angle_j) > armor_param.lights_angle_max_dif)
    {
        return false;
    }
    return true;
}

bool Finder::matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes)
{
    armor_boxes.clear();
    for (int i = 0; i < light_blobs.size() - 1; i++)
    {
        for (int j = i + 1; j < light_blobs.size(); j++)
        {
            if (!isCoupleLight(light_blobs.at(i), light_blobs.at(j), enemy_color))
            {
                continue;
            }
            cv::Rect2d rect_left = light_blobs.at(i).rect.boundingRect();
            //    cv::Rect2d rect_left = light_blobs.at(static_cast<unsigned long>(i)).rect.boundingRect();
            cv::Rect2d rect_right = light_blobs.at(j).rect.boundingRect();
            double min_x, min_y, max_x, max_y;
            min_x = fmin(rect_left.x, rect_right.x) - 5;
            max_x = fmax(rect_left.x + rect_left.width, rect_right.x + rect_right.width) + 5;
            min_y = fmin(rect_left.y, rect_right.y) - 0.8 * fmax(rect_right.height, rect_left.height) - 8;
            max_y = fmax(rect_left.y + rect_left.height, rect_right.y + rect_right.height) + 0.8 * fmax(rect_right.height, rect_left.height) + 8;
            if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows)
                continue;
            int left = i, right = j;
            if (light_blobs.at(i).rect.center.x > light_blobs.at(j).rect.center.x)
            {
                left = j;
                right = i;
            }
            LightBlobs pair_blobs = {light_blobs.at(left), light_blobs.at(right)};
            armor_boxes.emplace_back(
                cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y),
                pair_blobs,
                enemy_color);
        }
    }
    return !armor_boxes.empty();
}

bool Finder::findArmorBox(const cv::Mat &src, ArmorBox &box)
{
    LightBlobs light_blobs;
    ArmorBoxes armor_boxes;

    box.rect = cv::Rect2d();
    box.id = -1;
    if (!findLightBlobs(src, light_blobs))
    {
        return false;
    }

    if (!matchArmorBoxes(src, light_blobs, armor_boxes))
    {
        return false;
    }
#ifdef show_debug
    if (show_armor_boxes_class)
    {
        showArmorBoxesClass("armor_boxes_class", src, armor_boxes);
        // cv::waitKey(waite_time);
    }
#endif
    std::cout<<"find "<<std::endl;

    if (classifier)
    {
        for (auto &armor_box : armor_boxes)
        {
            cv::Mat roi = src(armor_box.rect).clone();
            int g_nContrastValue = 140;
            int g_nBrightValue = 30;
            for (int y = 0; y < roi.rows; y++)
            {
                for (int x = 0; x < roi.cols; x++)
                {
                    for (int c = 0; c < 3; c++)
                    {
                        roi.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>((g_nContrastValue * 0.01) * (roi.at<cv::Vec3b>(y, x)[c]) + g_nBrightValue);
                    }
                }
            }
            cv::resize(roi, roi, cv::Size(28, 28), cv::INTER_LINEAR);
            cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
            int c = classifier(roi);
            armor_box.id = c;
        }
        sort(armor_boxes.begin(), armor_boxes.end(), [&](const ArmorBox &a, const ArmorBox &b)
             {
                 if (last_box.rect != cv::Rect2d())
                 {
                     return getPointLengths(a.getCenter() - last_box.getCenter()) <
                            getPointLengths(b.getCenter() - last_box.getCenter());
                 }
                 else
                 {
                     return a < b;
                 }
             });
        for (auto &one_box : armor_boxes)
        {
            if (one_box.id != -1 && box.id != 0 && box.id != 6 && box.id != 7 && one_box.id != 8 && one_box.id != 9 && one_box.id != 10)
            {
                box = one_box;
                if (box.id == 1 || box.id == 7 || box.id == 8)
                {
                    box.size = ArmorBox::BIG;
                }
                else
                    box.size = ArmorBox::BIG;
                break;
            }
        }
        if (box.rect == cv::Rect2d())
        {
            if (state == TRACKING_STATE)
                box = armor_boxes.at(0);

            else
                return false;
        }
    }
    else
    {
        box = armor_boxes[0];
    }

    return true;
}
