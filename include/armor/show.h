#ifndef SHOW_H_
#define SHOW_H_
#include "./Armor.h"
#include "../other/switch.h"

/**
 * @brief 显示一组灯条
 */
void drawLightBlobs(cv::Mat &src, const LightBlobs &blobs);

/**
 * @brief 显示多个灯条
 */
void showLightBlobs(std::string windows_name, const cv::Mat &src, const LightBlobs &light_blobs);

/**
 * @brief 显示多个装甲板及其灯条
 */
void showArmorBoxes(std::string windows_name, const cv::Mat &src, const ArmorBoxes &armor_boxes);

/**
 * @brief 显示单个装甲板的类别
 */
void showArmorBoxClass(std::string windows_name, const cv::Mat &src, const ArmorBox &armor_box);

/**
 * @brief 显示所有装甲板的类别
 */
void showArmorBoxesClass(std::string window_names, const cv::Mat &src, const ArmorBoxes &boxes);

#endif