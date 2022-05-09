/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-06-13 14:15:29 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-06-14 15:42:10
 */

#ifndef _XBOT_NAVIGATION_GOALS_H_
#define _XBOT_NAVIGATION_GOALS_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace Xbot
{
class NavGoal
{
public:
    typedef struct
    {
        double x;
        double y;
        double z;
        double quat_x;
        double quat_y;
        double quat_z;
        double quat_w;
    } GoalPose;

    typedef struct
    {
        double x;
        double y;
    } Point;

    NavGoal();

    bool setup(ros::NodeHandle &n, ros::NodeHandle &private_node);
    void run();

private:
    void init_goal(GoalPose& goalPose, Point point);
    move_base_msgs::MoveBaseGoal get_goal(GoalPose goalPose);
    void set_goal();
    void send_goal();
    void wait_for_complete();

    MoveBaseClient _action_client;
    move_base_msgs::MoveBaseGoal _current_goal;
    GoalPose _goal_pose[4];
    int _num_of_goal;
};
} // namespace Xbot

#endif