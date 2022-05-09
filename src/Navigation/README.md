# 源码文件
点云降采样
roslaunch voxel_grid_filter voxel_grid_filter.launch 

地面过滤
roslaunch ray_ground_filter ray_ground_filter.launch 

障碍物感知
roslaunch lidar_detection test_lidar_detection.launch 

openPlanner
roslaunch local_planner rollout_generator.launch
roslaunch local_planner local_trajectory_generator.launch 

加载轨迹
roslaunch global_planning load_path.launch 

路径追踪
roslaunch waypoint_follower pure_persuit.launch 
