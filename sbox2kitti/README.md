# sbox2kitti

------------------------------------------------------------------------

### Introduction

*sbox2kitti 是把sbox的激光点云数据和图像数据转成kitti的数据格式


------------------------------------------------------------------------

### How to use?

把sbox2kitti放到src中, src放到一个catkin_ws下,然后在catkin_ws下执行catkin_make
然后执行source devel/setup.bash 
roscore
rosrun obstacle_detection map_generate 
rosbag play xx.bay -r 0.1
------------------------------------------------------------------------

