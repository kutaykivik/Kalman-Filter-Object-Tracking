# Real-Time Implementation of Kalman Filter using FreeRTOS on Windows

## Introduction
This project aims to demonstrate the implementation of a Kalman Filter for tracking a moving object in two dimensions with constant velocity using the FreeRTOS real-time operating system. Given the limitations of achieving true real-time behavior on a Windows system, the FreeRTOS Windows port was utilized to showcase the concepts and design of the solution.

### FreeRTOS Windows Port
The FreeRTOS Windows Simulator/Emulator for Visual Studio and Eclipse-MingW was chosen for this project. This allowed for the development and testing of real-time concepts in a Windows environment. The port provides an accurate representation of FreeRTOS behavior, enabling the demonstration of the Kalman Filter implementation.

[FreeRTOS Windows Port](https://www.freertos.org/FreeRTOS-Windows-Simulator-Emulator-for-Visual-Studio-and-Eclipse-MingW.html)

## Kalman Filter for Object Tracking
The Kalman Filter was implemented to track a moving object with constant velocity in two dimensions. The original constant acceleration model from the Kalman Filter example was modified for a constant velocity model.

### Measurement Simulator
A measurement simulator was developed as a separate task to test the Kalman Filter. The simulator accepts initial position, velocity, and variance of Gaussian noise as input. It generates ground truth data, adds zero-mean Gaussian noise to simulate measurements, and drops some measurements with a defined probability.

## Implementation Details
The project consists of two main tasks: the Kalman Filter task and the Measurement Simulator task. These tasks communicate using an appropriate synchronization mechanism provided by FreeRTOS.

### Task Communication
Measurements from the simulator task are passed to the Kalman Filter task, allowing real-time processing and estimation.

## Data Collection and Visualization
To analyze the performance of the Kalman Filter, data was visualized using Python (not in real-time).

### Logging
Log files were generated, including execution times for each task, providing insights into the timing behavior of the system.

### Position Output:
Ploted for each dimension, including ground truth position, measurements, and Kalman Filter position outputs:

![pos_x](https://github.com/kutaykivik/Kalman-Filter-Simulation/assets/89020731/2452d07a-0627-4ab6-97b2-a8f033ef0ba1)

![pos_y](https://github.com/kutaykivik/Kalman-Filter-Simulation/assets/89020731/d5816b3c-6d56-4944-810d-75b470d7b0e1)

Ploted for each dimension, including ground truth position, measurements, and Kalman Filter position outputs:

![vel_x](https://github.com/kutaykivik/Kalman-Filter-Simulation/assets/89020731/86ee16e8-2144-4604-9fea-4f79b868ca1f)

![vel_y](https://github.com/kutaykivik/Kalman-Filter-Simulation/assets/89020731/b9295424-3fb3-4dce-8416-3518b6610514)

Ploted for Kalman Gain:

![KG](https://github.com/kutaykivik/Kalman-Filter-Simulation/assets/89020731/3093fde9-b98f-42d7-b084-ba393247bb71)



