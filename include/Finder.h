#ifndef FINDER_H_
#define FINDER_H_
#include "./armor/show.h"
#include "./serial/serial.h"
#include "./solve/solver.h"
#include "./armor/Armor.h"
#include "./serial/serial.h"
#include "./other/additions.h"
#include "./other/switch.h"
#include "./armor/classfier.h"
#include <sys/time.h>
#include <time.h>

typedef enum{
   //没识别到不发子弹
   NO_FIND_NO_SHOOT=0x00,
   //识别到不发射子弹
   FIND_NO_SHOOT=0x01,
   //识别到且发射子弹
   FIND_SHOOT=0x02,
   //没解算完，与上次指令相同
   NO_END_SOLVER=0x03,
}SHOOT_ECODE;

/*********************** 自瞄类定义 ************************/
class Finder
{
   friend class ImageProcess;

public:
   //! 发送指令代码
   SHOOT_ECODE shoot_code;

private:
   //！ 自瞄状态枚举定义
   typedef enum
   {
      SEARCHING_STATE,
      TRACKING_STATE,
      STANDBY_STATE
   } State;
   //！ 自瞄状态对象实例
   State state;
   //!  装甲板参数
   ArmorParam armor_param;

   Classifier classifier;

   Serial &serial;

   Predict predict;
   PnPSolver solver;

private:
   //! 敌方颜色,0为红色，1为蓝色
   int enemy_color;
   //! 目标装甲板
   ArmorBox target_box, last_box, predict_box;

   //! 防止乱切目标计数器
   int anti_switch_cnt=0;
   //！记录跟踪状态的帧数，用于定时退出跟踪状态
   int tracking_cnt=0;

   float bullet_speed=14.5;

   //！ 用于计时，调试使用
   double allcosttime = 0;
   int nums = 0;

private:
   /**
 * @brief 判断轮廓是否为一个灯条，借助面积及面积比判断
 * @param comtour 灯条轮廓
 * @param rect   灯条旋转矩形
 * @param lw_rate  矩形长轴与宽轴比
 * @param area_ratio  轮廓面积和其最小外接矩形面积之比
 */
   bool isValidLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

   /**
 * @brief 在图像上寻找灯条
 * @param   light_blobs 得到的灯条集
 */
   bool findLightBlobs(const cv::Mat &src, LightBlobs &light_blobs);

   /**
 * @brief 判断两个灯条是否匹配
 */
   bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, int enemy_color);

   /**
 * @brief  匹配所有灯条，得出装甲板候选区
 */
   bool matchArmorBoxes(const cv::Mat &src, const LightBlobs &light_blobs, ArmorBoxes &armor_boxes);

   /**
 * @brief  得出目标装甲板
 * @param  box 得到的目标装甲板
 */
   bool findArmorBox(const cv::Mat &src, ArmorBox &box); //矫正模式

   /**
 * @brief 在给定图像上寻找所有可能的灯条，通过对通道进行两次二值化，取交叉灯条
 * @param light_blobs 储存候选灯条的容器
 * @param enemy_color 1为蓝色，0为红色
 * @return 图片为空或灯条数量小于2返回false，成功找到两个及以上的灯条返回true
 */
   bool findLightBlobss(const cv::Mat &src, LightBlobs &light_blobs);


   /**
 * @brief  通过串口改变设别颜色
 */
   void setColor(int color);

   /**
 * @brief  搜索状态，在全局进行搜索
 */
   bool stateSearchingTarget(cv::Mat &src);

   /**
 * @brief  跟踪状态，在ROI区域进行搜索
 */
   bool stateTrackingTarget(cv::Mat &src);

   /**
 * @brief  矫正状态，将来作为缓冲状态
 */
   bool stateStandBy();

   /**
 * @brief  筛选目标（未启用）
 */
   bool getTarget(ArmorBoxes &boxes, ArmorBox &box);

   /**
 * @brief  从串口设置子弹速度
 */
   void setSpeed(const uint8_t speed);

public:
   /**
 * @brief 自瞄接口
 */
   void run(cv::Mat &src);

   /**
 * @brief   初始化函数
 * @param  color 敌方颜色
 * @param   u    串口类
 * @param  paramfilename  分类器参数地址
 * @param  param  相机参数
 */
   Finder(const int &color, Serial &u, const string &paramfilename, const SolverParam &param, const float speed = 14.5);
   ~Finder() = default;
};

#endif
