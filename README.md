SLAM Project
=============================

### Implementing SLAM(Simultaneous Localization and Mapping) on a Turtlebot3 Waffle Pi

#### *Victor Ozoh -- MS in Robotics Winter Quarter Project - Northwestern University*


## Overview
The goal of this project is to implement SLAM on the Turtlebot3 Waffle Pi in Gazebo.
The two key questions that inspired the development of SLAM are:
- Where is the robot in the world?
- Where are the stationary/moving items in the world?

The SLAM problem has been solved in a probabilistic framework using the Bayes Filter.
I focus on implementing the Extended Kalman Filter(EKF) to perform the following:
- Localisation only
- Localization and Mapping with unknown correspondences. I implement the Maximum Likelihood estimator
to determine correspondences.

This implementation of the EKF SLAM is still not very robust and is currently being tested and updated.

## Introduction to the EKF Algorithm
More technical details on the working of the EKF implementation in the waffle_slam package can be found [here]()

## Setup
- Ubuntu 16.04
- ROS Kinetic
## Installation
Open the terminal and run the following commands.
- `sudo apt-get update`
- `sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers`

The eventual aim of this project is to run the EKF package on a Turtlebot3 in the real world.
After installing the ROS dependent packages, you can simply clone this repository into your workspace and run the `catkin_make` command.

## Instructions
`cd` into your workspace and source your setup file: `source devel/setup.bash`
1. To run the path tracking EKF localization implementation, run
`roslaunch waffle_slam localization.launch`

2. To run the EKF SLAM implementation run the command `roslaunch waffle_slam slam.launch`. This open up both Rviz and Gazebo. On your terminal, you can use the keyboard keys w, a, s and d to navigate while the robot builds an estimate of it's pose and of the detected features in the environment

## Future Work
I am currently working on improving the EKF SLAM so that it is more robust and can be used on a robot in the real world. In addition, I will implement the popular Particle Filter of Localization and Mapping.

## Notes
- SLAM is a key step for proper Navigation and Planning in mobile autonomous systems
### Localisation Problems
- Tracking: Initial pose is known and robot has to track it's position and orientation
- Global Localisation: Unknown initial pose
- Kidnapped Robot: Robot in operation is taken to an arbitrary position and has to localize.
### Some State Estimation problems
- Sensor Fusion: Combining data and information from different sensor measurements to get a more accurate state estimate
- Data Association: Linking uncertain measurements with known feature, path or track.
- Loop Closure Detection: Correctly asserting that a robot has visited a location previously.
- Registration: Associating collection of data into a known coordinate system
### Localisation Algorithms
- Kalman Filter, Extended Kalmn Filter and Unscented Kalman filter: Family of gaussian filters
- Histogram filter or Grid Localisation algorithm
- Markov Localisation: Utilizes discrete probability distribution as representation of state space and updates the probabilities with each iteration
- Particle filter or Monte Carlo localisation
### Kalman Filter
- Assumes linear motion and measurement models
- Assumes a unimodal gaussian
- Not suited to localisation problem of a wheeled robot. Extended Kalman filter is more appropriate for wheeled robots.
### EKF
- Does not asssume linear measurement or motion models
- Linearization of non-linear function is necessary
