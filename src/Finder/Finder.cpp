#include "../../include/Finder.h"

string id2name[20] = {"BA", "HR", "EG", "I3", "I4", "I5", "SY", "OP", "ER101", "ER010", "ER111"};

Finder::Finder(const int &color, Serial &u, const string &paramfilename, const SolverParam &_param, const float speed) : enemy_color(color), state(STANDBY_STATE),
                                                                                                                          classifier(paramfilename),
                                                                                                                          tracking_cnt(0),
                                                                                                                          serial(u),
                                                                                                                          solver(_param),
                                                                                                                          bullet_speed(speed),
                                                                                                                          shoot_code(NO_END_SOLVER)
{
   serial.openPort();
   armor_param.loadParam();  
}

void Finder::run(cv::Mat &src)
{
   state =SEARCHING_STATE;
   switch (state)
   {
   case SEARCHING_STATE:
   {
      if (stateSearchingTarget(src))
      {
         if ((target_box.rect & cv::Rect2d(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)) == target_box.rect)
         {
            state = TRACKING_STATE;
            tracking_cnt = 0;
         }
      }
      break;
   }
   case TRACKING_STATE:
   { //跟踪 模式
      if (!stateTrackingTarget(src) || ++tracking_cnt > 500)
      {
         state = SEARCHING_STATE;
      }
      break;
   }
   case STANDBY_STATE: //矫正模式
   default:
      stateStandBy();
   }
   if (target_box.rect != cv::Rect2d())
   {
      last_box = target_box;
      solver.setMode(target_box);
      solver.solve();
      shoot_code = FIND_NO_SHOOT;
      
   }
   else
   {
      solver.yaw = 0;
      solver.pitch = 0;
      shoot_code = NO_FIND_NO_SHOOT;
   }
}

void Finder::setSpeed(const uint8_t speed)
{
   switch (speed)
   {
   case 0x01:
      bullet_speed = 9.5;
      break;
   case 0x02:
      bullet_speed = 11.5;
      break;
   case 0x03:
      bullet_speed = 13.5;
      break;
   case 0x04:
      bullet_speed = 14.5;
      break;
   case 0x05:
      bullet_speed = 15.5;
      break;
   case 0x06:
      bullet_speed = 17;
      break;
   case 0x07:
      bullet_speed = 21;
      break;
   case 0x08:
      bullet_speed = 29;
      break;
   default:
      //如果接受错误，与上次子弹速度相同
      std::cout << "bullet speed receive error" << std::endl;
      break;
   }
}

void Finder::setColor(int color)
{
   if (color == 0x01)
      enemy_color = ENEMY_RED;
   else if (color == 0x02)
      enemy_color = ENEMY_BLUE;
   else
   {
      std::cout << "color recived error " << std::endl;
   }
}