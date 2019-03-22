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

## Basic goals
- Implement SLAM using the EKF(Extended Kalman Filter)

## Stretch goals
- Implement exploration algorithm to be used for mapping.
- Use one item/block as object to be identified and picked up. The pick up and drop off
locations will be supplied to the robot.
- Incorporate stationary and moving obstacles
- Get the robot to utilize computer vision to find where items are located and where to drop them off. This may involve having to implement a solution to Loop Closure Detection which is about the process of finding a previously visited place.

## Future Work
Particle Filter implementation of Localization and Mapping

## Packages
- PlotJuggler

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
- Send control commands via keyboard/joystick. Or develop exploration algorithm
- Subscribe to IMU and Odometry data
- Publish filtered pose on a new topic
- view trajectory on Rviz
