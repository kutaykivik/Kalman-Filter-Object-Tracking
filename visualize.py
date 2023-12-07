import matplotlib.pyplot as plt
import re

log_data_location = "C:/Users/oykum/Documents/Projects/KalmanFilter/src/output.txt"
# Read the log data
with open(log_data_location, "r") as file:
    log_data = file.read()

# Extract data using regular expressions
pattern = re.compile(r"Ground truth pos_x: (\d+\.\d+), Ground truth v_x: (\d+\.\d+),"
                     r" Ground truth pos_y: (\d+\.\d+), Ground truth v_y: (\d+\.\d+),"
                     r" Measurement pos_x: (\d+\.\d+) , Measurement pos_y: (\d+\.\d+),"
                     r" Estimated pos_x: (\d+\.\d+), Estimated v_x: (\d+\.\d+),"
                     r" Estimated pos_y: (\d+\.\d+), Estimated v_y: (\d+\.\d+), Kalman gain1: (\d+\.\d+),"
                     r" Kalman gain2: (\d+\.\d+), Time taken for Kalman Filter: (\d+\.\d+) seconds")

matches = pattern.findall(log_data)

# Extracted data
ground_truth_pos_x = [float(match[0]) for match in matches]
ground_truth_pos_y = [float(match[2]) for match in matches]
measurement_pos_x = [float(match[4]) for match in matches]
measurement_pos_y = [float(match[5]) for match in matches]
kalman_pos_x = [float(match[6]) for match in matches]
kalman_pos_y = [float(match[8]) for match in matches]
kalman_gain1 = [float(match[10]) for match in matches]
kalman_gain2 = [float(match[11]) for match in matches]
execution_times = [float(match[12]) for match in matches]

hertz_values = [1 / (execution_times[i + 1] - execution_times[i]) for i in range(len(execution_times) - 1)]
# Plot frequency of execution of the Kalman filter
plt.figure(figsize=(10, 5))
plt.plot(hertz_values, marker='o', linestyle='-', color='b')
plt.title('Frequency of Execution of Kalman Filter')
plt.xlabel('Iteration')
plt.ylabel('Frequency (hertz)')
plt.grid(True)
plt.show()

# Plot ground truth, measurements, and Kalman filter position outputs
plt.figure(figsize=(10, 5))
plt.plot(ground_truth_pos_x, label='Ground Truth', marker='o', linestyle='-', color=''
                                                                                    'g')
plt.plot(measurement_pos_x, label='Measurement', marker='o', linestyle='-', color='b')
plt.plot(kalman_pos_x, label='Kalman Filter', marker='o', linestyle='-', color='r')
plt.title('Position in X Dimension')
plt.xlabel('Iteration')
plt.ylabel('Position')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 5))
plt.plot(ground_truth_pos_y, label='Ground Truth', marker='o', linestyle='-', color='g')
plt.plot(measurement_pos_y, label='Measurement', marker='o', linestyle='-', color='b')
plt.plot(kalman_pos_y, label='Kalman Filter', marker='o', linestyle='-', color='r')
plt.title('Position in Y Dimension')
plt.xlabel('Iteration')
plt.ylabel('Position')
plt.legend()
plt.grid(True)
plt.show()

# Plot ground truth velocity and Kalman filter velocity estimation
ground_truth_vel_x = [float(match[1]) for match in matches]
kalman_vel_x = [float(match[7]) for match in matches]

plt.figure(figsize=(10, 5))
plt.plot(ground_truth_vel_x, label='Ground Truth', marker='o', linestyle='-', color='g')
plt.plot(kalman_vel_x, label='Kalman Filter', marker='o', linestyle='-', color='r')
plt.title('Velocity in X Dimension')
plt.xlabel('Iteration')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)
plt.show()

kalman_vel_y = [float(match[9]) for match in matches]
ground_truth_vel_y = [float(match[3]) for match in matches]

plt.figure(figsize=(10, 5))
plt.plot(ground_truth_vel_y, label='Ground Truth', marker='o', linestyle='-', color='g')
plt.plot(kalman_vel_y, label='Kalman Filter', marker='o', linestyle='-', color='r')
plt.title('Velocity in Y Dimension')
plt.xlabel('Iteration')
plt.ylabel('Velocity')
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 5))
plt.plot(kalman_gain1, label='Kalman Gain Pos', marker='o', linestyle='-', color='g')
plt.plot(kalman_gain2, label='Kalman Gain Vel', marker='o', linestyle='-', color='r')
plt.title('Kalman Gain Matrix')
plt.xlabel('Iteration')
plt.ylabel('KG')
plt.legend()
plt.grid(True)
plt.show()
