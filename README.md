SLAM and Manipulation Project
=============================

### using a Turtlebot3 Waffle Pi with OpenManipulator

#### *Victor Ozoh -- MS in Robotics Winter Quarter Project - Northwestern University*


## Overview
The goal of this project is to utilize the Turtlebot3 robot to accomplish a pick an place
task with several objects. There are a few sub tasks for the robot to perform. They include:

- Perform SLAM(Simultaneous Localisation and Mapping) or use a known map and implement localisation
- Navigation to location where objects are placed
- Manipulation: picking up items and dropping them
- Obstacle avoidance

## Basic goals
- Supply robot with a known map of an area and path to follow
- Use one item/block as object to be identified and picked up. The pick up and drop off
locations will be supplied to the robot.
- Ignore obstacles. Leave area clear of fixed or moving obstacles

## Stretch goals
- Implement exploration algorithm to be used for mapping.
- Implement SLAM. Compare Particle Filter with EKF(Extended Kalman Filter)
- Incorporate stationary and moving obstacles
- Get the robot to utilize computer vision to find where items are located and where to drop them off. This may involve having to implement a solution to Loop Closure Detection which is about the process of finding a previously visited place.


## Timeline
- 1/30/2019:  Complete set up of Waffle Pi robot
- 2/6/2019:   Set up mock environment for robot and specify a map in ROS
- 2/13/2019:  Complete map and path specification. Get robot to pick up a block and place it at drop off location.
- 2/20/2019: Implement Exploration algorithm and Particle filter or EKF to make robot more autonomous
- 2/27/2019:  More SLAM work
- 3/6/2019: Implement obstacle avoidance for static objects
- 3/13/2019:  Implement loop closure detection.

## Future Work
Loop Closure Detection

## Packages
- ar_track_alvar
- gmapping
- move_base
- move_group
- AMCL

## Notes
- SLAM is a key step for proper Navigation and Planning in mobile autonomous systems
### Localisation Problems
- Tracking: Initial pose is known and robot has to track it's position and orientation
- Global Localisation: Unknown initial pose
- Kidnapped Robot: Robot in operation is taken to an arbitrary position and has to localize.
### Some State Estimation problems
- Sensor Fusion: Combining data and information from different sensor measurements to get a more accurate state estimate
- Loop Closure Detection: Correctly asserting that a robot has visited a location previously.
- Data Association: Linking uncertain measurements with known path/track
- Registration: Associating collection of data into a known coordinate system
### Localisation Algorithms
- Kalman Filter, Extended Kalmn Filter and Unscented Kalman filter: Family of gaussian filters
- Histogram filter or Grid Localisation algorithm
- Markov Localisation: Utilizes discrete probability distribution as representation of state space and updates the probabilities with each iteration
- particle filter or Monte Carlo localisation
### Kalman Filter
- Assumes linear motion and measurement models
- Assumes a unimodal gaussian
- Not suited to localisation problem of a wheeled robot. Extended Kalman filter is more appropriate for wheeled robots.
### EKF
- Does not asssume linear measurement or motion models
- Linearization of non-linear function is necessary

### Overall Structure
- Send control commands via keyboard/jotstick. Or develop exploration algorithm
- Subscribe to IMU and Odometry data
- Publish filtered pose on a new topic
- view trajectory on Rviz
