# AwesomeSLAM ROS package *(WIP)*

![](https://img.shields.io/badge/Ubuntu-20.04-red)
![](https://img.shields.io/badge/ROS1-Noetic-blue)
![](https://img.shields.io/badge/Gazebo-11-green)


## Overview

EKF-SLAM and UKF-SLAM on Turtlebot3

<img src="figures/teaser.gif" alt="stream" width="900"/>


## Usage

#### Open Gazebo and Rviz
```
roslaunch awesome_slam awesome_slam.launch
```

#### Run EKF/UKF 
```
rosrun awesome_slam ekf
```
```
rosrun awesome_slam ukf
```

#### Launch turtlebot3_teleop
```
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
