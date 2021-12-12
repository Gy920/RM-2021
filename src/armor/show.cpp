#include "../../include/armor/show.h"

void drawLightBlobs(cv::Mat &src, const LightBlobs &blobs)
{
    for (const auto &blob : blobs)
    {
        cv::Scalar color(0, 255, 0);
        if (blob.blob_color == BLOB_RED) //灯条为红色
            color = cv::Scalar(0, 0, 255);
        else if (blob.blob_color == BOX_BLUE) //灯条为蓝色
            color = cv::Scalar(255, 0, 0);
        cv::Point2f vertices[4];
        blob.rect.points(vertices);
        for (int j = 0; j < 4; j++)
        {
            cv::line(src, vertices[j], vertices[(j + 1) % 4], color, 2);
        }
    }
}

void showArmorBoxes(std::string windows_name, const cv::Mat &src, const ArmorBoxes &armor_boxes)
{
    static cv::Mat image2show;
    if (src.type() == CV_8UC1)
    { // 黑白图像
        cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
    }
    else if (src.type() == CV_8UC3)
    { //RGB 彩色
        image2show = src.clone();
    }

    for (auto &box : armor_boxes)
    {
        if (box.box_color == ENEMY_RED)
        { //红色
            rectangle(image2show, box.rect, cv::Scalar(0, 255, 0), 1);
            drawLightBlobs(image2show, box.light_Blobs);
        }
        else if (box.box_color == ENEMY_BLUE)
        { //蓝色
            rectangle(image2show, box.rect, cv::Scalar(255, 0, 0), 1);
            drawLightBlobs(image2show, box.light_Blobs);
        }
    }
    cv::namedWindow(windows_name, cv::WINDOW_NORMAL);
    cv::imshow(windows_name, image2show);
}

void showLightBlobs(std::string windows_name, const cv::Mat &src, const LightBlobs &light_blobs)
{
    static cv::Mat image2show;

    if (src.type() == CV_8UC1)
    { // 黑白图像
        cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
    }
    else if (src.type() == CV_8UC3)
    { //RGB 彩色
        image2show = src.clone();
    }
    for (const auto &light_blob : light_blobs)
    {
        cv::Scalar color(0, 255, 0);
        if (light_blob.blob_color == ENEMY_RED) //红色
            color = cv::Scalar(0, 255, 0);
        else if (light_blob.blob_color == ENEMY_BLUE) //蓝色
            color = cv::Scalar(0, 255, 0);
        cv::Point2f vertices[4];
        light_blob.rect.points(vertices);
        for (int j = 0; j < 4; j++)
        {
            cv::line(image2show, vertices[j], vertices[(j + 1) % 4], color, 5);
        }
    }
    cv::namedWindow(windows_name, cv::WINDOW_NORMAL);
    cv::imshow(windows_name, image2show);
}

void showArmorBoxClass(std::string windows_name, const cv::Mat &src, const ArmorBox &box)
{
    static cv::Mat image2show;
    if (box.rect == cv::Rect2d())
    {
        imshow(windows_name, src);
    }
    if (src.type() == CV_8UC1)
    { // 黑白图像
        cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
    }
    else if (src.type() == CV_8UC3)
    { //RGB 彩色
        image2show = src.clone();
    }
    cv::Scalar color;
    if (box.box_color == ENEMY_BLUE)
        color = cv::Scalar(255, 150, 0);
    else
        color = cv::Scalar(0, 150, 255);
    // cv::Rect rect(box.rect.x + box.rect.width / 9.0, box.rect.y + box.rect.height / 5.0, 7 * box.rect.width / 9, 3 * box.rect.height / 5.0);
    //  rectangle(image2show, rect, color,3);
    if (box.id != -1 && box.id != 0 && box.id != 6 && box.id != 7 && box.id != 8 && box.id != 9 && box.id != 10)
    {
        // for(int i=0;i<4;i++){
        //     cv::line(image2show,box.f_points[i],box.f_points[(i+1)%4],color,2);
        // }
        cv::Point2f center=(box.f_points[0]+box.f_points[1]+box.f_points[2]+box.f_points[3])/4;
        //  rectangle(image2show, rect, color,3);
        cv::line(image2show, box.f_points[0], box.f_points[2], color, 2);
        cv::line(image2show, box.f_points[1], box.f_points[3], color, 2);
        cv::circle(image2show,center,4,cv::Scalar(0,0,255),-1);
        drawLightBlobs(image2show, box.light_Blobs);
        putText(image2show, id2name[box.id], cv::Point(center.x - 40, center.y - 25), cv::FONT_HERSHEY_TRIPLEX, 2, color);
    }
    else
    {
        // putText(image2show, "error box", cv::Point(box.rect.x + 2, box.rect.y + 10), cv::FONT_HERSHEY_TRIPLEX, 2, color);
    }
    cv::namedWindow(windows_name, cv::WINDOW_NORMAL);
    imshow(windows_name, image2show);
}

void showArmorBoxesClass(std::string windows_name, const cv::Mat &src, const ArmorBoxes &boxes)
{
    static cv::Mat image2show;
    if (src.type() == CV_8UC1)
    { // 黑白图像
        cvtColor(src, image2show, cv::COLOR_GRAY2RGB);
    }
    else if (src.type() == CV_8UC3)
    { //RGB 彩色
        image2show = src.clone();
    }
    if (boxes.size() == 0)
    {
        cv::namedWindow(windows_name, cv::WINDOW_NORMAL);
        imshow(windows_name, src);
    }
    //不是0的装甲板
    for (const auto &box : boxes)
    {
        if (box.id != 0)
        {
            cv::Scalar color;
            if (box.box_color == ENEMY_BLUE)
                color = cv::Scalar(255, 100, 0);
            else
                color = cv::Scalar(255, 100, 0);
            rectangle(image2show, box.rect, cv::Scalar(0, 255, 0), 2);
            drawLightBlobs(image2show, box.light_Blobs);
            putText(image2show, box.id + " ", cv::Point(box.rect.x + 2, box.rect.y + 2), cv::FONT_HERSHEY_TRIPLEX, 1, color);
        }
    }
    cv::namedWindow(windows_name, cv::WINDOW_NORMAL);
    cv::imshow(windows_name, image2show);
}
