#ifndef SWITCH_H
#define SWITCH_H
/*********************/
#define ENEMY_BLUE 1  
#define ENEMY_RED  0

#define BLOB_RED    ENEMY_RED
#define BLOB_BLUE   ENEMY_BLUE
#define BOX_RED     ENEMY_RED
#define BOX_BLUE    ENEMY_BLUE
#define DISTANCE_HEIGHT_5MM (10700.0)     // 单位: cm*pixel
#define DISTANCE_HEIGHT     DISTANCE_HEIGHT_5MM
#define LIMIT_TIME        (0.012)

#define IMAGE_HEIGHT        (720)//图片高度//   480   
#define IMAGE_WIDTH         (1280)//图片宽度//  640
#define IMAGE_CENTER_X      (IMAGE_WIDTH/2.0)
#define IMAGE_CENTER_Y      (IMAGE_HEIGHT/2.0)
#define IMAGE_X_DIS         ((1920-IMAGE_WIDTH)/2-20)//(160+0)   1920-2*w
#define IMAGE_Y_DIS         ((1200-IMAGE_HEIGHT)/2+70)//(150+0)   1200-2
#define SETIAL_DELAY        (0.0035)

/*************************SWITCH***************************************/
// #define   WRITEVIDEIO       //录视频
// #define COMPETE         //比赛前一定要define
#ifdef COMPETE
    #define   WRITEVIDEIO       //录视频
#else
/********** **杂项开关*************/
// #define DEBUG
    #ifdef DEBUG
    const bool show_all_armor_cnt=false;
    const bool show_now_class_num=false;
    const bool show_lights_cnt=false;           //输出 灯条颜色、两次初步寻找的灯条的数量、最终灯条集的数量
    const bool show_all_light_data=false;
    #endif
/************输出程序运行时间***************/    
//  #define cout_time
/*********输出解算内容************/
 //#define PNPDEBUG    //输出解算内容
    #ifdef PNPDEBUG
        const bool show_yaw_pitch_tvec=true;
        const bool show_predict_point=false;
    #endif

/**********显示图片**************/
#define show_debug      //显示图片
   #ifdef show_debug   
        const int  waite_time=1; 
        const bool show_light_blobs=false;//显示多个灯条
        const bool show_armor_boxes=false;//显示多个装甲板
        const bool show_armor_boxes_class=false;//显示多个装甲板及其类别//0为红色，1为蓝色
        const bool show_target_armor_class=true;//显示目標装甲板
    #endif

/************显示串口数据***********/
 //#define serial_debug     //展示串口发送数据
    #ifdef serial_debug
         const bool show_sent_yaw_and_pitch=true;
    #endif

#define cjc

#endif


#endif
