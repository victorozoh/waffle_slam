# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import numpy as np
import math
from numpy.random import randn
import matplotlib.pyplot as plt

# simulate 2D movement with some noise and measurement
# observations returns a tuple of our predicted state and measurements
def observations(z_var, process_var, count=1, dt=1.):
    x, vel = 0., 1.0
    z_std = math.sqrt(z_var) 
    p_std = math.sqrt(process_var)
    xs, zs = [], []
    for _ in range(count):
        v = vel + (randn() * p_std)
        x += v*dt        
        xs.append(x)
        zs.append(x + randn() * z_std)        
    return np.array(xs), np.array(zs)

dt = 1.0 # time step
R_var = 10
Q_var = 0.01
x = np.array([[10.0, 4.5]]).T
P = np.diag([500, 49]) # state covariance
A = np.array([[1, dt], # Process dynamics
              [0,  1]])
H = np.array([[1., 0.]]) # measurement dynamics
R = np.array([[R_var]]) # measurement noise
#Q = Q_discrete_white_noise(dim=2, dt=dt, var=Q_var) # process noise
Q = np.array([[0.01,0.04],[0.04, 0.02]])


count = 50
track, measurements = observations(R_var, Q_var, count)

# definition of the Kalman filter function  
def KalmanFilter(A, H, x, P, Q, R, zi):
    measurement = zi
    current_state_estimate = x  # Current state estimate
    current_cov_estimate = P  # Current probability of state estimate
    Q = Q  # Process covariance
    R = R  # Measurement covariance

    # State Prediction 
    predicted_state_estimate = np.dot(A ,current_state_estimate)
    predicted_cov_estimate = np.dot(A, current_cov_estimate).dot(A.T) + Q
    
    # Measurement update
    innovation_covariance = np.dot(H,P).dot(H.T) + R
    innovation = measurement - np.dot(H, predicted_state_estimate)
    #  compute Kalman gain
    kalman_gain = np.dot(predicted_cov_estimate, H.T).dot(np.linalg.inv(innovation_covariance))
    
    # Update 
    current_state_estimate = predicted_state_estimate + kalman_gain * innovation
    current_cov_estimate = predicted_cov_estimate - np.dot(kalman_gain,H).dot(predicted_cov_estimate)
        
    return current_state_estimate, current_cov_estimate

# loop through measurements and compute the estimated states
state_estimates = []
cov_estimates = []
for measurement in measurements:
    # predict
    estimated_x, estimated_cov = KalmanFilter(A, H, x, P, Q, R, measurement)
    
    state_estimates.append(estimated_x)
    cov_estimates.append(estimated_cov)
    
    
# Plot the track, estimates and measurements
state_estimates = np.array(state_estimates)
cov_estimates = np.array(cov_estimates)

def plot_values(state_estimates, track, measurements):
    pass

plot_values(state_estimates, track, measurements)