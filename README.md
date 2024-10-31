# KF in HLS
## Project Description
In my project, I will accelerate a recursive algorithm used to estimate a system's state from noisy measurements (Kalman Filter) to update the height of the cells of an elevation map. 
## What is implemented
This repository implements a Kalman Filter, which is part of https://github.com/ANYbotics/elevation_mapping. However, due to time limitations, all of the functionality was not added yet.
### Implemenmted
+ Simple implementaion of an elevation map
+ Streaming of new pointcloud data 
+ Update of height in the map using kalman filter principles

### NOT yet implemented
+ Tranformation from coordinate-systems
+ Jacobian derivation
+ Interfacing with PS to get sensor data in real-time

### Evaluation
Due to time time constraints the implementation was tested with made up data, which can be found in pointcloud_data.txt. The goal is to use a realsense camera to gather data but the communication between the camera and the FPGA is still in progress because the libaries needed to gather data from the camera are not compatible with HLS.

## Organization and Running
The folder found in this repository contains an HLS component. The workspace should be openable in Vitis 2023.2 and C-simulation can be performed to test validity of results. 
The HLS component uses:
### Source files
+ kalman_filter.cpp
+ kalman_filert.h
### Testbench
+ kf_test.cpp (testbench)
+ pointcloud_data.txt (data for testing)


