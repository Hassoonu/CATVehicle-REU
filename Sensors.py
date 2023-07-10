import numpy as np
import matplotlib.pyplot as plt
import json, math

# specify standard deviation of the noise
STANDARD_DEVIATION = 0.2

# add Gaussian noise to 'y' data
def addNoise(y, std=STANDARD_DEVIATION):
    y += np.random.normal(0, std)
    return y

def kalmanFilter(y, R=0.14, Q=0.05, state_est=0, prediction=0):
    curr_point = y; prediction += Q 

    # update
    kalman_gain = prediction / (prediction + R) 
    state_est += (kalman_gain * (curr_point - state_est))
    prediction *= (1 - kalman_gain) 

    return state_est, prediction

