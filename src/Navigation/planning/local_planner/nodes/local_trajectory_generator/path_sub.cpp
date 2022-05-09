#include <Eigen/Core>
#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <path_msgs/Lane.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/Waypoint.h>
#include <sstream>
#include <utils.hpp>
 
bool path_msg_received = false;
 
/*********************************滤波*********************************/
void filter_path(autoware_msgs::Lane& path, double dot)
{
    std::vector<autoware_msgs::Waypoint> temp_vec;
    double sum = 0.0;
    int length = path.waypoints.size();
    for (int i = 0; i < length - 2; i++) {
        if (sum >= dot) {
            temp_vec.push_back(path.waypoints[i]);
            sum = 0.0;
        }
        sum += util::distance2points(path.waypoints[i].pose.pose.position, path.waypoints[i + 1].pose.pose.position);
    }
    // path.waypoints.clear();
    path.waypoints.assign(temp_vec.begin(), temp_vec.end());
}
 
/*********************************修正*********************************/
void adjust_orientation(autoware_msgs::Lane& path)
{
    for (size_t i = 0; i < path.waypoints.size() - 2; i++) {
        autoware_msgs::Waypoint p_c = path.waypoints[i];
        autoware_msgs::Waypoint p_n = path.waypoints[i + 1];
        double yaw = std::atan2(p_n.pose.pose.position.y - p_c.pose.pose.position.y, p_n.pose.pose.position.x - p_c.pose.pose.position.x);
        p_c.yaw = yaw;
        Eigen::AngleAxisd rollangle(0, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd yawangle(yaw, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd pitchangle(0, Eigen::Vector3d::UnitY());
        Eigen::Quaterniond q = rollangle * yawangle * pitchangle;
        path.waypoints[i].pose.pose.orientation.x = q.x();
        path.waypoints[i].pose.pose.orientation.y = q.y();
        path.waypoints[i].pose.pose.orientation.z = q.z();
        path.waypoints[i].pose.pose.orientation.w = q.w();
    }
}
 
/*********************************回调*********************************/
void getCentralPathSection_cb(const autoware_msgs::LaneConstPtr& msg)
{
    std::cout << "getCentralPathSection_cb" << std::endl;
    centralPathSection.clear();
    for (int i = 0; i < msg->waypoints.size(); i++) {
        UtilityNS::WayPoint p;
        p.pos.x = msg->waypoints[i].pose.pose.position.x;
        p.pos.y = msg->waypoints[i].pose.pose.position.y;
        p.pos.z = msg->waypoints[i].pose.pose.position.z;
        p.pos.yaw = msg->waypoints[i].yaw;
        centralPathSection.push_back(p);
    }
}
 
 
int main(int argc, char ** argv)
{	
	ros::init(argc, argv, "path_sub");
	ros::NodeHandle nh;
 
	//Subscriber 
    sub_centralPath = nh.subscribe("centralPathSection", 1, &getCentralPathSection_cb);// 获取截取的中心路径
 
    //pub
	//ros::Publisher pub_centralPathSection = nh.advertise<autoware_msgs::Lane>("global_path0", 1);
 
	ros::Rate loopRate(10);
    /*
	while (ros::ok()) {
 
		ros::spinOnce();
		loopRate.sleep();
	}*/
    ros::spinOnce();
	return 0;
}
