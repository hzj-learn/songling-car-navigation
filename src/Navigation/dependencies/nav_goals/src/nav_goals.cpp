/*
 * @Author: Jian Xu @Xag 
 * @Date: 2019-06-14 09:24:37 
 * @Last Modified by: Jian Xu @Xag
 * @Last Modified time: 2019-06-14 15:42:27
 */

#include "../include/nav_goals.h"

using namespace Xbot;

NavGoal::NavGoal() : _action_client("move_base", true)
{
    /* x = 0.0
       y = 0.0
       z = sin(psi/2)
       w = cos(psi/2)*/
    _goal_pose[0] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    _goal_pose[1] = {0.2, 0.0, 0.0, 0.0, 0.0, -0.707, 0.707};//-90
    _goal_pose[2] = {0.2, -0.2, 0.0, 0.0, 0.0, -1.0, 0.0};//-180
    _goal_pose[3] = {0.0, -0.2, 0.0, 0.0, 0.0, -0.707, -0.707};//-270

    _num_of_goal = 0;

    _current_goal = get_goal(_goal_pose[_num_of_goal]);
}

bool NavGoal::setup(ros::NodeHandle &n, ros::NodeHandle &private_node)
{
    Point point[4];
    private_node.param<double>("left_down_point_x", point[0].x, 0.0);
    private_node.param<double>("left_down_point_y", point[0].y, 0.0);
    private_node.param<double>("right_down_point_x", point[1].x, 0.4);
    private_node.param<double>("right_down_point_y", point[1].y, 0.0);
    private_node.param<double>("right_up_point_x", point[2].x, 0.4);
    private_node.param<double>("right_up_point_y", point[2].y, -0.4);
    private_node.param<double>("left_up_point_x", point[3].x, 0.0);
    private_node.param<double>("left_up_point_y", point[3].y, -0.4);

    for(int i=0;i<4;i++)
    {
        init_goal(_goal_pose[i],point[i]);
    }

    run();
    return true;
}

void NavGoal::run()
{
    set_goal();
    send_goal();
    wait_for_complete();
}

void NavGoal::init_goal(GoalPose& goalPose, Point point)
{
    goalPose.x = point.x;
    goalPose.y = point.y;
}

void NavGoal::set_goal()
{
    while (!_action_client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for move_base action server to come up");
    }

    _current_goal = get_goal(_goal_pose[_num_of_goal]);

    ROS_INFO_STREAM("Set goal success!Current goal num is " << _num_of_goal);
}

void NavGoal::send_goal()
{
    _action_client.sendGoal(_current_goal);

    ROS_INFO("Send goal success!");
}

void NavGoal::wait_for_complete()
{
    _action_client.waitForResult();

    if (_action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM("The goal " << _num_of_goal << " was successfully completed!");
        ROS_INFO("-----------------------------------------------");
        if (_num_of_goal < 3)
        {
            _num_of_goal++;
        }
        else
        {
            _num_of_goal = 0;
        }
    }
    else
    {
        ROS_WARN_STREAM("Xbot failed to complete the " << _num_of_goal << " goal for some reason.");
        ROS_INFO("----------------------------------------------");
    }
}

move_base_msgs::MoveBaseGoal NavGoal::get_goal(GoalPose goalPose)
{
    move_base_msgs::MoveBaseGoal move_base_goal_;
    move_base_goal_.target_pose.header.frame_id = "map";
    move_base_goal_.target_pose.header.stamp = ros::Time::now();
    move_base_goal_.target_pose.pose.position.x = goalPose.x;
    move_base_goal_.target_pose.pose.position.y = goalPose.y;
    move_base_goal_.target_pose.pose.position.z = goalPose.z;
    move_base_goal_.target_pose.pose.orientation.x = goalPose.quat_x;
    move_base_goal_.target_pose.pose.orientation.y = goalPose.quat_y;
    move_base_goal_.target_pose.pose.orientation.z = goalPose.quat_z;
    move_base_goal_.target_pose.pose.orientation.w = goalPose.quat_w;

    return move_base_goal_;
}