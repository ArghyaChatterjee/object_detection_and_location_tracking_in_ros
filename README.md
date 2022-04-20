# object_detection_and_location_tracking_in_ros
This repository contains code for object detection and location tracking in ROS with realsense d435 camera.

## Clone This Repo
```
cd ~/catkin_ws/src
git clone https://github.com/ArghyaChatterjee/object_detection_and_location_tracking_in_ros.git
```

## Realsense ROS Installation
- From Binary
```
sudo apt-get install ros-melodic-realsense2-camera
sudo apt-get install ros-melodic-realsense2-description
```
- From Source (After you have cloned the repo)
```
cd ~/catkin_ws
catkin_make --only-pkg-with-deps realsense2-camera
catkin_make --only-pkg-with-deps realsense2-description
```
## YOLO-DarkNet ROS Installation
- From Source (After you have cloned the repo)
```
cd ~/catkin_ws
catkin_make --only-pkg-with-deps -DCMAKE_BUILD_TYPE=Release darknet_ros
```

## Location Tracking package Installation
- From Source (After you have cloned the repo)
```
cd ~/catkin_ws
catkin_make --only-pkg-with-deps -DCMAKE_BUILD_TYPE=Release location_tracking
```
## Launch the whole workspace
```
cd ~/catkin_ws
source devel/setup.bash
roslaunch location_tracking object_detection_and_location_tracking.launch 
```

