#include "../../include/Finder.h"

bool Finder::isValidLightBlob(const std::vector<cv::Point> &contour,const cv::RotatedRect &rect){
    double contour_area=cv::contourArea(contour);
    if(contour.size()<=5||contour_area<10){
        return false ;
    }
    double lw_rate= rect.size.height>rect.size.width?
                    rect.size.height/rect.size.width:
                    rect.size.width/rect.size.height;
    if( lw_rate < armor_param.light_min_wihe_ratio || lw_rate > armor_param.light_max_wihe_ratio){
        return false;
    }
    double area_ratio=contour_area/rect.size.area();
    return  (rect.size.area() < 50 && area_ratio > armor_param.light_min_area_ratio) ||
            (rect.size.area() >= 50 && area_ratio >armor_param.light_bigger_area_ratio); 
}



bool Finder::findLightBlobs(const cv::Mat &src,LightBlobs &light_blobs){
    int light_threshold_=185;
    //这里红蓝色灯条灰度阈值是否要改变？
    cv::Mat gray_img_,binary_color_img_,color2gray_img_,gray_img__;
	cv::cvtColor(src, gray_img__, cv::COLOR_BGR2GRAY);
    cv::threshold(gray_img__, gray_img_,armor_param.gray_light_min_threshold, 255, cv::THRESH_BINARY);
	std::vector<cv::Mat> bgr_channel;
	cv::split(src, bgr_channel);

    if(enemy_color==ENEMY_RED){
        cv::subtract(bgr_channel[2], bgr_channel[0], binary_color_img_);
        cv::threshold(binary_color_img_, binary_color_img_, armor_param.red_light_min_threshold, 255, cv::THRESH_BINARY);
    }else{
        cv::subtract(bgr_channel[0], bgr_channel[2], binary_color_img_);
        cv::threshold(binary_color_img_, binary_color_img_, armor_param.blue_light_min_threshold, 255, cv::THRESH_BINARY);
    }
    //灰度图
    std::vector<std::vector<cv::Point>> contours_brightness;
	cv::findContours(gray_img_, contours_brightness, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    auto element=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(4,2));
    cv::morphologyEx(gray_img_,gray_img_,cv::MORPH_DILATE,element);

    cv::Mat binary_Mat;
    if(enemy_color==ENEMY_BLUE){
       binary_Mat = binary_color_img_ & gray_img_;
    }else{
        binary_Mat=binary_color_img_;
    }
    if(binary_Mat.empty()||gray_img_.empty()){
         return false;
     }
    //  cv::imshow("gray_img",gray_img_);
    //  cv::imshow("color_gray",binary_Mat);

    std::vector<std::vector<cv::Point>> contours_light;
	cv::findContours(binary_Mat, contours_light, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<std::vector<cv::Point>> vaildlights_contours;

    for(auto contour:contours_light){
        cv::RotatedRect light=cv::minAreaRect(contour);{
            if(isValidLightBlob(contour, light)){
                    vaildlights_contours.emplace_back(contour);
            }
        }
    }

    light_blobs.reserve(vaildlights_contours.size());
    int x=0;
    std::vector<bool> is_processes(vaildlights_contours.size());
    for (unsigned int i = 0; i < contours_brightness.size(); ++i) {
        for (unsigned int j = 0; j < vaildlights_contours.size(); ++j) {
        if (!is_processes[j]&&x<=vaildlights_contours.size()) {
            if (cv::pointPolygonTest(vaildlights_contours[j], contours_brightness[i][0], true) >= 0.0) {
            cv::RotatedRect single_light = cv::minAreaRect(vaildlights_contours[j]);
                double area_ratio=cv::contourArea(vaildlights_contours[j])/single_light.size.area();
                light_blobs.emplace_back(
                            single_light, area_ratio,
                            enemy_color
                    );
                x++;
                is_processes[j] = true;
                break;
            } 
        } 
        } 
    } 

#ifdef DEBUG
    if(show_lights_cnt){
        std::cout<<"enemy_color  (0-red，1-blue)  "<<enemy_color<<std::endl;
        std::cout<<"contours_light.size()   "<<contours_light.size()<<std::endl;
        std::cout<<"contours_brightness.size()     "<<contours_brightness.size()<<std::endl;
        std::cout<<"light_blobs.size()            "<<light_blobs.size()<<std::endl;
    }

    if(show_all_light_data){
        for(auto light:light_blobs){
        std::cout<<"light.color         "<<light.blob_color<<std::endl;
        std::cout<<"light.area_ratio    "<<light.area_ratio<<std::endl;
        std::cout<<"light.lenght        "<<light.length<<std::endl;
        std::cout<<"light.rect.angle    "<<light.rect.angle<<std::endl;
        std::cout<<"light.rect.area     "<<light.rect.size.area()<<std::endl;
        std::cout<<"light.rect.height   "<<light.rect.size.height<<std::endl;
        std::cout<<"light.rect.width    "<<light.rect.size.width<<std::endl;
        }
    }
#endif

#ifdef show_debug
        if(show_light_blobs){
            showLightBlobs("lightblobs",src,light_blobs);
            cv::waitKey(waite_time);
        }
#endif
    return light_blobs.size() >= 2;

}