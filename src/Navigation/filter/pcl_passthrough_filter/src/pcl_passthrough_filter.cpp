/**
 * @file plc_passthrough_filter.cpp
 * @brief 此节点主要是接收/velodyne_points的点云数据，通过直通滤波过滤掉一半的点云，只保留前方的点云
 * 输入：
 *      点云数据 /velodyne_points
 * 输出：
 *      过滤后的点云数据 /points_raw
 * @date 2020-4-3
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/passthrough.h>

ros::Subscriber subPointCloud;
ros::Publisher pubOutputPoints;

void call_back(const sensor_msgs::PointCloud2ConstPtr input)
{
  ROS_INFO("Received Velodyne Points!");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZ>};
  pcl::fromROSMsg(*input,*cloud_in);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("x");// 过滤哪个轴，x、y、z
  pass.setFilterLimits (0,200);// 保留点云的范围[minVec,maxVec]
  pass.setFilterLimitsNegative (false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered {new pcl::PointCloud<pcl::PointXYZ>};
  pass.filter (*cloud_filtered);

  sensor_msgs::PointCloud2 cloud_out;
  pcl::toROSMsg(*cloud_filtered,cloud_out);
  cloud_out.header = input->header;
  pubOutputPoints.publish(cloud_out);
}


int main (int argc, char** argv)
{
  ros::init (argc, argv, "pcl_passthrough_filter");
  ros::NodeHandle nh;
  ROS_INFO("Passthrough Filter Node Initialize");

  // 接收原始点云
  subPointCloud = nh.subscribe ("velodyne_points", 1, call_back);

  // 发布过滤后的点云
  pubOutputPoints = nh.advertise<sensor_msgs::PointCloud2> ("points_raw", 1);

  // Spin
  ros::spin ();

}









