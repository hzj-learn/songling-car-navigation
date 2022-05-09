# Navigation
## 底盘启动
```sh
roslaunch ackermann_navigation rf2o_ndt_mapping.launch
```

## 录制移动轨迹
```sh
roslaunch waypoint_maker waypoint_saver.launch 
```

## ndt建图
```sh
roslaunch ndt_mapping ndt_mapping.launch
```

## 保存地图
```sh
rosrun pcl_ros pointcloud_to_pcd input:=/ndt_map prefix:=map
```

## 点云降采样
roslaunch voxel_grid_filter voxel_grid_filter.launch 

## 加载点云地图
```sh
roslaunch map_file load_map.launch
```

## 地面过滤
```sh
roslaunch ray_ground_filter ray_ground_filter.launch 
```

## 障碍物感知
```sh
roslaunch lidar_detection test_lidar_detection.launch 
```

## openPlanner
```sh
roslaunch local_planner rollout_generator.launch
roslaunch local_planner local_trajectory_generator.launch 
```

## 加载轨迹
```sh
roslaunch global_planning load_path.launch 
```

## 路径追踪
```sh
roslaunch waypoint_follower pure_persuit.launch 
```

## 话题重映射
```sh
rosrun topic_tools relay /velodyne_points /points_raw
```

## 设置TF
```sh
roslaunch static_tf static_tf_car.launch 
```

## 运行效果

Pure Pursuit
![](../img/OpenPlanner/%202020-03-19%2009:52:00.png)
OpenPlanner
![](../img/OpenPlanner/%202020-03-19%2018:31:11.png)
OpenPlanner
![](../img/OpenPlanner/%202020-03-27%2011:19:58.png)