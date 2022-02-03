To run first download bag file from [this link](https://drive.google.com/file/d/1jK9F-dGf1GVhpiu6wryobeqIlTuhT2Qb/view?usp=sharing)

Build using:
```
    catkin_make
```

Run using:
```
    roslaunch vins vins_rviz.launch
    rosrun vins vins_node ~/ws/src/VINS-Fusion/config/my_cam/mydata_mono.yaml
    rosrun ceres_optimization final
    rosbag play RGBD_LIDAR_bag_2.bag
```
