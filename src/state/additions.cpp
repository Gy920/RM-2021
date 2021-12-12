#include "../../include/other/additions.h"


double getPointLengths(const cv::Point2f &p)
{
  return sqrt(p.x * p.x + p.y * p.y);
}


double get_wall_time()
{
  struct timeval time;
  if (gettimeofday(&time, NULL))
  {
    return 0;
  }
  return (double)time.tv_sec + (double)time.tv_usec * .000001;
}


double centerDistance(const cv::Rect2f &rect,const cv::Point2d src_center){
    cv::Point2d center;
    center.x=rect.x+rect.width/2.0;
    center.y=rect.y+rect.height/2.0;
    return sqrt(pow(center.x-src_center.x,2)+pow(center.y-src_center.y,2));
}

std::string  GetVedioName()
{
    time_t nowtime;
    struct tm *p;
    time(&nowtime);
    p = localtime(&nowtime);
    char _outpath[40];
    sprintf(_outpath, "../vedio/x%dmon-%dd-%dh-%dmin.avi", p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min);
    std::cout << _outpath << std::endl;
    return _outpath;
}