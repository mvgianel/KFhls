# KF in HLS
## Organization and Running
This project was built on Vitis 2023.2 using part xcu280-fsvh2892-2L-e and setting the clock to 250MHz. The testbench file uses a map file and a pointcloud file to test the created map against the golden_otput.txt file, which has the expected heights of the updated cells. Running simulation and C/RTL cosimulation resulted on the correct output. 
### The HLS component uses:
#### Source files
+ kalman_filter.cpp
+ kalman_filert.h
#### Testbench
+ kf_test.cpp (testbench)
+ pointcloud_data.txt (pointcloud data for testing)
+ map1.txt (map file for testing)
+ golden_output.txt (file with expected output)
#### Outputs
+ updated_map1.txt

## Project Description
In my project, I accelerate a recursive algorithm used to estimate a system's state from noisy measurements (Kalman Filter) to update the height of the cells of an elevation map. 

## What is implemented
This repository implements a Kalman Filter, which is part of https://github.com/ANYbotics/elevation_mapping. However, due to time limitations, the implementation was adapted to not use Robot Operating System (ROS). As a result, all capability was not implemented and data cannot be streamed from a device as was envisioned.

### Implemenmted before
+ Quantized implementaion of an elevation map which is populated based on sensor measurements  
  + Map has a defined resolution to account for cm in the measurements
  + Map has height estimate and variance of said estimate
+ Streaming of new pointcloud data 
+ Update of height in the map using kalman filter principles

### Added features
+ changed floating point operations to use fixed point
+ using HLS friendly ceil to get consistent cell partitions
  + Before the cells were not always what I expected, so changed the way the partitions were made
+ added INLINE, DATAFLOW, and PIPELINE pragmas

### To be implemented
+ Figure out a way to process a stream of pointcloud data in batches
+  Fix timing and dependency issues with code

### Will not get implemented
+ Interfacing with PS to get sensor data in real-time
  + Since sensor data would need to be streamed to the cloud from my laptop this will not be implemented

### Evaluation
Due to time time constraints the implementation was tested with made up data, which can be found in pointcloud_data.txt. This is also used when the application is deployed in the U280.




