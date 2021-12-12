#ifndef DECISION_H_
#define DECISION_H_

#include <iostream>
#include <numeric>
#include <map>

struct decision
{
    int kind;//1英雄 2工程 3步兵 4 哨兵
    int HP_level;//1low 2 middle 3 high
    int angle_scope;//1inside 0outside
};
struct decision hero_1= {1,1,1};
struct decision hero_2= {1,1,0};
struct decision hero_3= {1,2,1};
struct decision hero_4= {1,2,0};
struct decision hero_5= {1,3,1};
struct decision hero_6= {1,3,0};

struct decision infantray_1= {3,1,1};
struct decision infantray_2= {3,1,0};
struct decision infantray_3= {3,2,1};
struct decision infantray_4= {3,2,0};
struct decision infantray_5= {3,3,1};
struct decision infantray_6= {3,3,0};

//wait for the project

std:: map<struct decision,int>decision_level = {
    {hero_1,1},{infantray_1,2},{hero_2,3},{infantray_2,4},{hero_3,5},{infantray_3,6},
    {hero_4,7},{infantray_4,8},{hero_5,9},{infantray_5,10},{hero_6,11},{infantray_6,12}
};

#endif
