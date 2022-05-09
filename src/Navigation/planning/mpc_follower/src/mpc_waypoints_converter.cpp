/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file mpc_waypoints_converter.cpp
 * @brief 此节点主要是接收路径信息后，截取最近点前后的若干点生成局部路径发送出去
 * 输入：
 *      最近路径点的序号
 *      原始全局路径信息
 *      最近路径点之后的路径点信息
 * 输出：
 *      发布局部路径信息（只包含最近路径点前后的若干个点）
 * 
 * @date 2020-3-27
 */
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <autoware_msgs/Lane.h>

class MPCWaypointsConverter
{
public:
  MPCWaypointsConverter()
  {
    pub_waypoints_ = nh_.advertise<autoware_msgs::Lane>("/mpc_waypoints", 1);//发布局部路径信息（只包含最近路径点前后的若干个点）
    sub_closest_waypoint_ = nh_.subscribe("/closest_waypoint", 1, &MPCWaypointsConverter::callbackClosestWaypoints, this);// 接收最近路径点的序号
    sub_base_waypoints_ = nh_.subscribe("/base_waypoints", 1, &MPCWaypointsConverter::callbackBaseWaypoints, this);// 接收原始全局路径信息
    sub_final_waypoints_ = nh_.subscribe("/final_waypoints", 1, &MPCWaypointsConverter::callbackFinalWaypoints, this);// 接收最近路径点之后的路径点信息

    closest_idx_ = 0;
    back_waypoints_num_ = 10;
    front_waypoints_num_ = 50;
  };
  ~MPCWaypointsConverter(){};

private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_waypoints_;
  ros::Subscriber sub_closest_waypoint_, sub_base_waypoints_, sub_final_waypoints_, sub_current_velocity_;

  autoware_msgs::Lane base_waypoints_;
  int closest_idx_;
  int back_waypoints_num_;
  int front_waypoints_num_;

//###################################################
//               获取最近路径点的编号
//###################################################
  void callbackClosestWaypoints(const std_msgs::Int32 msg)
  {
    closest_idx_ = msg.data;
  }

//###################################################
//               获取原始全局路径
//###################################################
  void callbackBaseWaypoints(const autoware_msgs::Lane &msg)
  {
    base_waypoints_ = msg;
  }

//###################################################
//               获取最近路径点之后的路径点信息
//###################################################
  void callbackFinalWaypoints(const autoware_msgs::Lane &final_waypoints)
  {
    if (base_waypoints_.waypoints.size() == 0 || final_waypoints.waypoints.size() == 0)
      return;

    if ((int)base_waypoints_.waypoints.size() - 1 < closest_idx_)
    {
      ROS_WARN("base_waypoints_.waypoints.size() - 1 = %d, closest_idx_ = %d", (int)base_waypoints_.waypoints.size(), closest_idx_);
      return;
    }

    auto sq_dist = [](const geometry_msgs::Point &a, const geometry_msgs::Point &b) {
      const double dx = a.x - b.x;
      const double dy = a.y - b.y;
      return dx * dx + dy * dy;
    };

    autoware_msgs::Lane mpc_waypoints;
    mpc_waypoints.header = final_waypoints.header;
    mpc_waypoints.increment = final_waypoints.increment;
    mpc_waypoints.lane_id = final_waypoints.lane_id;
    mpc_waypoints.lane_index = final_waypoints.lane_index;
    mpc_waypoints.cost = final_waypoints.cost;
    mpc_waypoints.closest_object_distance = final_waypoints.closest_object_distance;
    mpc_waypoints.closest_object_velocity = final_waypoints.closest_object_velocity;
    mpc_waypoints.is_blocked = final_waypoints.is_blocked;

    // find closest point index in base_waypoints (topic /closest_waypoints has no consistency with /final_waypoints due to delay)
    //找出/base_waypoints路径上的最近点
    int closest_idx = -1;
    for (int i = 0; i < (int)base_waypoints_.waypoints.size(); ++i) {
      const double d = sq_dist(final_waypoints.waypoints[1].pose.pose.position, base_waypoints_.waypoints[i].pose.pose.position);
      if (d < 0.01) {
        closest_idx = i;
        break;
      }
    }
    if (closest_idx == -1) {
      ROS_ERROR("cannot find closest base_waypoints' waypoint to final_waypoints.waypoint[1] !!");
    }
    // 将最近点之前的back_waypoints_num_个路径点（已经跑过的路径点）放入mpc_waypoints
    int base_start = std::max(closest_idx - back_waypoints_num_, 0);
    for (int i = base_start; i < closest_idx; ++i)
    {
      mpc_waypoints.waypoints.push_back(base_waypoints_.waypoints.at(i));
      mpc_waypoints.waypoints.back().twist = final_waypoints.waypoints[1].twist;
    }
    // 将最近点之后的front_waypoints_num_个路径点（未跑过的路径）注入mpc_waypoints
    int final_end = std::min(front_waypoints_num_ + 1, (int)final_waypoints.waypoints.size());
    for (int i = 1; i < final_end; ++i)
    {
      mpc_waypoints.waypoints.push_back(final_waypoints.waypoints.at(i));
    }
    // 发布局部路径信息
    pub_waypoints_.publish(mpc_waypoints);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mpc_waypoints_converter");
  MPCWaypointsConverter obj;
  ros::spin();
  return 0;
};
