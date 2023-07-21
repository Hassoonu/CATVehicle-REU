import numpy as np
import matplotlib.pyplot as plt
import json, math
from scipy.optimize import minimize

# load the data
target_vid = 'v.0'

with open(f'vehicle_data.json', 'r') as f1:
    data = json.load(f1)

with open(f'sensor_data.json', 'r') as f2:
    sensor_data = json.load(f2)

# specify standard deviation of the noise
noise_std_dev = 0.2

# # add Gaussian noise to 'y' data
# for vid, vdata in data.items():
#     vdata['accelerations_noisy'] = [a + np.random.normal(0, noise_std_dev) for a in vdata['accelerations']]

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

def optimizeAccel():

    plt.figure(figsize=(7, 5))
    # plt.plot(data[target_vid]['times'], data[target_vid]['accelerations'], label='Ground Truth', color='gray')
    # plt.plot(data[target_vid]['times'], data[target_vid]['accelerations_noisy'], label='Noisy Data', linestyle='dashed')
    plt.plot(sensor_data['times'], sensor_data['sensor_accelerations'], label='Noise', color='gray')
    plt.plot(data[target_vid]['times'], data[target_vid]['accelerations'], label='Ground Truth', color='black')

    # plot the different optimized kalman filters
    n = len(sensor_data['times']) # number of data points
    est_buffer = np.zeros(n)
    data_buffer = np.zeros(n)
    R = 10; Q = 0.1
    R, Q = optimize_parameters(sensor_data['sensor_accelerations'], data[target_vid]['accelerations'], R, Q)
    print(f"R = {R:.3f}, Q = {Q:.3f}, R / Q = {(R / Q):.3f}")

    # reinitialize state_est and prediction
    state_est = 0
    prediction = 0
    est_buffer = kalman_filter(sensor_data['sensor_accelerations'], R, Q, state_est=state_est, prediction=prediction)
    
    # # add filtered data to dataset
    data[target_vid]['y_filtered'] = est_buffer.tolist()
    plt.plot(data[target_vid]['times'], data[target_vid]['y_filtered'], label=f'Filter [Q = {Q:.2f}, R = {R:.2f}]', color='red')
    # plot original, noisy and filtered data
    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.title(f'Original vs Noisy vs Filtered Data')
    plt.legend()
    plt.show()

def testAccel():

    plt.figure(figsize=(7, 5))
    plt.plot(sensor_data['times'], sensor_data['sensor_accelerations'], label='Noise', color='gray')
    plt.plot(data[target_vid]['times'], data[target_vid]['accelerations'], label='Ground Truth', color='black')

    # plot the different optimized kalman filters
    n = len(sensor_data['times']) # number of data points
    for iteration in range(0, 4):
        est_buffer = np.zeros(n)
        data_buffer = np.zeros(n)
        Q = 0.1
        if iteration == 0:
            R = 37 * Q
        elif iteration == 1:
            R = 7.3 * Q
        elif iteration == 2:
            R = 23.1 * Q
        elif iteration == 3:
            R = 12.16 * Q

        # reinitialize state_est and prediction
        state_est = 0
        prediction = 0
        est_buffer = kalman_filter(sensor_data['sensor_accelerations'], R, Q, state_est=state_est, prediction=prediction)

        # # add filtered data to dataset
        data[target_vid]['y_filtered' + str(iteration)] = est_buffer.tolist()
        plt.plot(data[target_vid]['times'], data[target_vid]['y_filtered' + str(iteration)], label=f'Filter [R / Q: {(R / Q):.3f}]', color='red')
    # plot original, noisy and filtered data
    plt.xlabel('Time')
    plt.ylabel('Acceleration')
    plt.title(f'Original vs Noisy vs Filtered Data')
    plt.legend()
    plt.show()

def optimizeVel():

    plt.figure(figsize=(7, 5))
    plt.plot(sensor_data['times'], sensor_data['sensor_velocities'], label='Noise', color='gray')
    plt.plot(data[target_vid]['times'], data[target_vid]['velocities'], label='Ground Truth', color='black')

    # plot the different optimized kalman filters
    n = len(sensor_data['times']) # number of data points
    for iteration in range(0, 2):
        est_buffer = np.zeros(n)
        data_buffer = np.zeros(n)
        R = math.pow(1, iteration); Q = 1
        R, Q = optimize_parameters(sensor_data['sensor_velocities'], data[target_vid]['velocities'], R, Q)
        print(f"R = {R:.3f}, Q = {Q:.3f}, R / Q = {(R / Q):.3f}")

        # reinitialize state_est and prediction
        state_est = 30
        prediction = 0
        est_buffer = kalman_filter(sensor_data['sensor_velocities'], R, Q, state_est=state_est, prediction=prediction)
        
        # # add filtered data to dataset
        data[target_vid]['y_filtered' + str(iteration)] = est_buffer.tolist()
        plt.plot(data[target_vid]['times'], data[target_vid]['y_filtered' + str(iteration)], label=f'Filter [Q = {Q:.2f}, R = {R:.2f}]', color="red")
    # plot original, noisy and filtered data
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title(f'Original vs Noisy vs Filtered Data')
    plt.legend()
    plt.show()

def testVel():

    plt.figure(figsize=(7, 5))
    plt.plot(sensor_data['times'], sensor_data['sensor_velocities'], label='Noise', color='gray')
    plt.plot(data[target_vid]['times'], data[target_vid]['velocities'], label='Ground Truth', color='black')

    # plot the different optimized kalman filters
    n = len(sensor_data['times']) # number of data points
    for iteration in range(0, 4):
        est_buffer = np.zeros(n)
        data_buffer = np.zeros(n)
        Q = 0.1
        if iteration == 0:
            R = 37 * Q
        elif iteration == 1:
            R = 7.3 * Q
        elif iteration == 2:
            R = 23.1 * Q
        elif iteration == 3:
            R = 3.02 * Q

        # reinitialize state_est and prediction
        state_est = 30
        prediction = 0
        est_buffer = kalman_filter(sensor_data['sensor_velocities'], R, Q, state_est=state_est, prediction=prediction)

        # add filtered data to dataset
        data[target_vid]['y_filtered' + str(iteration)] = est_buffer.tolist()
        plt.plot(data[target_vid]['times'], data[target_vid]['y_filtered' + str(iteration)], label=f'Filter [R / Q: {(R / Q):.3f}]', color="red")
    # plot original, noisy and filtered data
    plt.xlabel('Time')
    plt.ylabel('Velocity')
    plt.title(f'Original vs Noisy vs Filtered Data')
    plt.legend()
    plt.show()

# optimizeAccel()
values = np.array(list(sensor_data["sensor_accelerations"]))
print(f"standard deviation is: {np.std(values):.3f}")

'''
Velocity R / Q Ratios:
noise std=0.4: [0.042, 0.054, 0.053, 0.042, 0.062, 0.050, 0.038] AVG = 0.049
noise std=0.8: [0.131, 0.128, 0.121, 0.143, 0.155, 0.133, 0.191]

Acceleration R / Q Ratios:
Velocity Q = 1, R = 0.05, noise std=0.4: [31.97, 21.72, 23.17, 22.73, 24.94, 22.60] AVG = 24.5
'''