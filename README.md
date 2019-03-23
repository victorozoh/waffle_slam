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

## Installation

## Instructions

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
- Particle filter or Monte Carlo localisation
### Kalman Filter
- Assumes linear motion and measurement models
- Assumes a unimodal gaussian
- Not suited to localisation problem of a wheeled robot. Extended Kalman filter is more appropriate for wheeled robots.
### EKF
- Does not asssume linear measurement or motion models
- Linearization of non-linear function is necessary
