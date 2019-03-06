SLAM Project
=============================

### Implementing SLAM(Simultaneous Localization and Mapping) on a Turtlebot3 Waffle Pi

#### *Victor Ozoh -- MS in Robotics Winter Quarter Project - Northwestern University*


## Overview
The goal of this project is to implement SLAM and utilize the Turtlebot3 Waffle Pi robot to navigate.
SLAM is a fundamental aspect of autonomy is robotics. Just like we cannot have a postal system
without zip codes or computer network without some Internet Protocol addressing, we cannot have autonomous robots without SLAM.
The two key questions that inspired the development of SLAM are:
- Where is the robot in the world?
- Where are the stationary/moving items in the world?

The answer to the two questions above form components of the state of the robot which we wish to know at all times.
By definition, the state of a robot is the collection of all aspects of the robot that impact the future. Due to uncertainty
in the dynamics and interaction of a robot and it's environment, the state of a robot is represented with a probability density function(pdf).
The challenge is then to model the propagation of the this pdf as the robot moves in it's environment.
The proper representation and modeling of the dynamics of the pdf has been solved in a probabilistic framework using the Bayes Filter.
The Kalman, Extended Kalman Filter and Particle Filters are all unique implementations of the Bayes Filter.

There are a few sub tasks for the robot to perform. They include:

- Perform SLAM(Simultaneous Localization and Mapping) or use a known map and implement localization
- Navigation to location where objects are placed
- Obstacle avoidance

## Basic goals
- Implement SLAM. Compare Particle Filter with EKF(Extended Kalman Filter)

## Stretch goals
- Implement exploration algorithm to be used for mapping.
- Use one item/block as object to be identified and picked up. The pick up and drop off
locations will be supplied to the robot.
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
- Send control commands via keyboard/joystick. Or develop exploration algorithm
- Subscribe to IMU and Odometry data
- Publish filtered pose on a new topic
- view trajectory on Rviz
