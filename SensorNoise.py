import numpy as np
import matplotlib.pyplot as plt
import json, math
from scipy.optimize import minimize

# load the data
target_vid = 'v.1'

with open(f'vehicle_data.json', 'r') as f:
    loaded_data = json.load(f)

data = loaded_data

# specify standard deviation of the noise
noise_std_dev = 0.2

# add Gaussian noise to 'y' data
for vid, vdata in data.items():
    vdata['accelerations_noisy'] = [a + np.random.normal(0, noise_std_dev) for a in vdata['accelerations']]

def kalman_filter(y, R, Q, state_est=0, prediction=0):
    n = len(y) 
    est_buffer = np.zeros(n)

    for i in range(n):
        # predict
        curr_point = y[i]
        prediction += Q 

        # update
        kalman_gain = prediction / (prediction + R) 
        state_est += (kalman_gain * (curr_point - state_est))
        prediction *= (1 - kalman_gain) 

        est_buffer[i] = state_est

    return est_buffer

def objective(params, *args):
    R, Q = params
    y, y_original = args
    y_filtered = kalman_filter(y, R, Q)
    error = np.sqrt(np.mean((y_filtered - y_original)**2))
    return error

def optimize_parameters(y_noisy, y_original, R_init=1, Q_init=0.1):
    initial_guess = [R_init, Q_init]
    result = minimize(objective, initial_guess, args=(y_noisy, y_original))
    R_opt, Q_opt = result.x
    return R_opt, Q_opt

def main(Q):
    plt.figure(figsize=(6, 4))
    plt.plot(data[target_vid]['times'], data[target_vid]['accelerations'], label='Ground Truth', color='gray')
    plt.plot(data[target_vid]['times'], data[target_vid]['accelerations_noisy'], label='Noisy Data', linestyle='dashed')
    # plot the different optimized kalman filters
    n = len(data[target_vid]['times']) # number of data points
    for iteration in range(0, 4):
        est_buffer = np.zeros(n)
        data_buffer = np.zeros(n)
        # reinitialize state_est and prediction
        state_est = 0
        prediction = 0

        R = math.pow(math.e, iteration - 2)
        for i in range(n):
            # predict
            curr_point = data[target_vid]['accelerations_noisy'][i]
            prediction += Q  # predict the error

            # update
            kalman_gain = prediction / (prediction + R)  # calculate Kalman Gain: (0 to 1)
            state_est += (kalman_gain * (curr_point - state_est))  # Update the state value
            prediction *= (1 - kalman_gain)  # update the error

            est_buffer[i] = state_est
            data_buffer[i] = curr_point

        # add filtered data to dataset
        data[target_vid]['y_filtered' + str(iteration)] = est_buffer.tolist()
        plt.plot(data[target_vid]['times'], data[target_vid]['y_filtered' + str(iteration)], label=f'Filter [R = {R:.2f}]', linestyle='dotted')
    # plot original, noisy and filtered data
    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.title(f'Original vs Noisy vs Filtered Data, Q = {Q}')
    plt.legend()
    plt.show()
main(Q=0.01)
main(Q=.1)
main(Q=.5)
main(Q=5)