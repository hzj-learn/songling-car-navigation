/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-06-13 14:30:29 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-06-14 14:24:03
 */

#include "../include/nav_goals.h"

using namespace Xbot;

int main(int argv,char **argc)
{
    ros::init(argv,argc,"navigation_goals");

    ros::NodeHandle n;
    ros::NodeHandle private_node("~");
    
    NavGoal navGoal;

    while(ros::ok())
    {
        navGoal.setup(n,private_node);
    }
}