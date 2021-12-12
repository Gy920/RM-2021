#include "../../include/Finder.h"

bool Finder::stateSearchingTarget(cv::Mat &src)
{ //搜索模式
    if (findArmorBox(src, target_box))
    {                                        // 在原图中寻找目标，并返回是否找到
        if (last_box.rect != cv::Rect2d() && //这里根据面积判断这个切换条件，如果是大面积的，限制要求（系数2.0）要增大（yaw摆动大），小面积反之
            (getPointLengths(last_box.getCenter() - target_box.getCenter()) > last_box.rect.width * 2.0) &&
            anti_switch_cnt++ < 3)
        {                            // 判断当前目标和上次有效目标是否为同一个目标
            target_box = ArmorBox(); // 并给３帧的时间，试图找到相同目标
            last_box = ArmorBox();
            return false; // 可以一定程度避免频繁多目标切换
        }
        else
        {
            anti_switch_cnt = 0;
            return true;
        }
    }
    else
    {
        target_box = ArmorBox();
        anti_switch_cnt++;
        return false;
    }
}

bool Finder::stateStandBy()
{
    state = SEARCHING_STATE;
    return true;
}

bool Finder::stateTrackingTarget(cv::Mat &src)
{
    auto pos = target_box.rect;
#ifdef cout_time
    double a = get_wall_time();
#endif
    cv::Rect2d bigger_rect;
    if ((pos & cv::Rect2d(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)) != pos)
    {
        target_box = ArmorBox();
        return false;
    }
    bigger_rect.x = pos.x - 0.80 * pos.width;
    bigger_rect.y = pos.y - 0.5 * pos.height;
    bigger_rect.height = pos.height * 2;
    bigger_rect.width = pos.width * 2.6;
    bigger_rect &= cv::Rect2d(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
    cv::Mat roi = src(bigger_rect).clone();

    ArmorBox box;
    // 在区域内重新搜索。
    if (findArmorBox(roi, box))
    { // 如果成功获取目标，则利用搜索区域重新更新追踪器
        target_box = box;
        target_box.rect.x += bigger_rect.x; //　添加roi偏移量
        target_box.rect.y += bigger_rect.y;
        for (auto &blob : target_box.light_Blobs)
        {
            blob.rect.center.x += bigger_rect.x;
            blob.rect.center.y += bigger_rect.y;
        }
    }
    else
    {   //缓冲区域
        bigger_rect.x -= 0.75 * bigger_rect.width;
        bigger_rect.y -= 0.25 * bigger_rect.height;
        bigger_rect.width *= 2.5;
        bigger_rect.height *= 1.5;
        bigger_rect &= cv::Rect2d(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT);
        cv::Mat bigger_roi = src(bigger_rect);
        if (findArmorBox(bigger_roi, box))
        { // 如果成功获取目标，则利用搜索区域重新更新追踪器
            target_box = box;
            target_box.rect.x += bigger_rect.x; //　添加roi偏移量
            target_box.rect.y += bigger_rect.y;
            for (auto &blob : target_box.light_Blobs)
            {
                blob.rect.center.x += bigger_rect.x;
                blob.rect.center.y += bigger_rect.y;
            }
        }
        else
        {
            target_box = ArmorBox();
            return false;
        }
    }
    last_box = target_box;
    return true;
}
