# Robocup@Home Pick and Place

## Overview
This Project enables The Tiago Robot to grasp the designated bottle and palce it in the center of the table.

## YOLO(V3)
You only look once (YOLO) is used to detect pre-trained classes. In order to install darknet_ros with ROS Noetic:
```bash
  cd tiago_ws/src
  git clone --recursive git@github.com:leggedrobotics/darknet_ros.git
```

Build the workspace:
```bash
  catkin build darknet_ros -DCMAKE_BUILD_TYPE=Release
  source devel/setup.bash
```

## Usage
### Point Cloud
In the local Terminal 1:
```bash
roslaunch plane_segmentation object_detection.launch 
```
In the local Terminal 2:
```bash
roslaunch plane_segmentation plane_segmentation_tiago.launch
```
In the local Terminal 3:
```bash
roslaunch object_labeling object_labeling.launch
```
### Pick and Place
In the local Terminal 4:
```bash
roslaunch grasp grasp_try.launch
```
In the local Terminal 5:

Grasp the bottle:
```bash
roslaunch grasp grasp_left.launch 
```
Grasp the cup:
```bash
roslaunch grasp grasp_right.launch 
```
In the local Terminal 6:
```bash
rosrun grasp tuck_arm.py 
```
